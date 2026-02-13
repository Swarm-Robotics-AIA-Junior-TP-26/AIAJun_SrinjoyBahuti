#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

import time
from enum import Enum, auto


class ControllerState(Enum):
    WAITING_FOR_CONNECTION = auto()
    STREAMING_SETPOINTS = auto()
    REQUESTING_OFFBOARD = auto()
    ARMING = auto()
    ACTIVE = auto()
    EMERGENCY_STOP = auto()
    SHUTDOWN = auto()


class VelocityControllerNode(Node):

    LINEAR_VEL_XY = 1.0
    LINEAR_VEL_Z = 0.8
    ANGULAR_VEL_Z = 0.6

    SETPOINT_RATE_HZ = 20.0
    PRE_ARM_STREAM_SECONDS = 2.0
    SERVICE_TIMEOUT_SEC = 5.0
    SERVICE_RETRY_INTERVAL = 1.0

    def __init__(self):
        super().__init__('velocity_controller_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.state = ControllerState.WAITING_FOR_CONNECTION
        self.current_key = ''
        self.mavros_state = None
        self.setpoint_stream_start_time = None
        self.last_service_call_time = 0.0

        self.key_subscription = self.create_subscription(
            String,
            '/keypress',
            self.key_callback,
            10
        )

        self.state_subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.publish_timer = self.create_timer(
            1.0 / self.SETPOINT_RATE_HZ,
            self.control_loop_callback
        )

        self.get_logger().info('Velocity controller node started')
        self.get_logger().info('Waiting for MAVROS connection to FCU...')

    def state_callback(self, msg: State):
        self.mavros_state = msg

    def key_callback(self, msg: String):
        self.current_key = msg.data
        if msg.data == 'esc' and self.state == ControllerState.ACTIVE:
            self.get_logger().warn('ESC received - initiating emergency stop!')
            self.state = ControllerState.EMERGENCY_STOP

    def build_velocity_command(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        key = self.current_key.lower()

        if key == 'w':
            msg.twist.linear.x = self.LINEAR_VEL_XY
        elif key == 's':
            msg.twist.linear.x = -self.LINEAR_VEL_XY
        elif key == 'a':
            msg.twist.linear.y = self.LINEAR_VEL_XY
        elif key == 'd':
            msg.twist.linear.y = -self.LINEAR_VEL_XY
        elif key == 'x':
            msg.twist.linear.z = self.LINEAR_VEL_Z
        elif key == 'z':
            msg.twist.linear.z = -self.LINEAR_VEL_Z
        elif key == 'q':
            msg.twist.angular.z = self.ANGULAR_VEL_Z
        elif key == 'e':
            msg.twist.angular.z = -self.ANGULAR_VEL_Z

        return msg

    def set_mode(self, mode: str):
        if not self.set_mode_client.service_is_ready():
            self.get_logger().error('Set mode service not available!')
            return

        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request)
        future.add_done_callback(lambda f: self.set_mode_callback(f, mode))

    def set_mode_callback(self, future, mode):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info(f'Mode {mode} request sent successfully')
            else:
                self.get_logger().warn(f'Mode {mode} rejected by FCU')
        except Exception as e:
            self.get_logger().error(f'Set mode service error: {e}')

    def arm_drone(self, arm: bool):
        if not self.arming_client.service_is_ready():
            self.get_logger().error('Arming service not available!')
            return

        request = CommandBool.Request()
        request.value = arm

        future = self.arming_client.call_async(request)
        future.add_done_callback(lambda f: self.arm_callback(f, arm))

    def arm_callback(self, future, arm):
        try:
            result = future.result()
            if result.success:
                action = "Armed" if arm else "Disarmed"
                self.get_logger().info(f'{action} successfully')
            else:
                self.get_logger().warn('Arm/disarm returned success=False')
        except Exception as e:
            self.get_logger().error(f'Arming service error: {e}')

    def control_loop_callback(self):
        velocity_cmd = self.build_velocity_command()
        self.velocity_publisher.publish(velocity_cmd)

        if self.state == ControllerState.WAITING_FOR_CONNECTION:
            if self.mavros_state is not None and self.mavros_state.connected:
                self.get_logger().info('Connected to FCU!')
                self.get_logger().info(
                    'Starting setpoint stream for 2.0s...'
                )
                self.setpoint_stream_start_time = time.time()
                self.state = ControllerState.STREAMING_SETPOINTS

        elif self.state == ControllerState.STREAMING_SETPOINTS:
            elapsed = time.time() - self.setpoint_stream_start_time
            if elapsed >= self.PRE_ARM_STREAM_SECONDS:
                self.get_logger().info('Setpoint streaming complete')
                self.get_logger().info('Requesting OFFBOARD mode...')
                self.state = ControllerState.REQUESTING_OFFBOARD
                self.set_mode('OFFBOARD')
                self.last_service_call_time = time.time()

        elif self.state == ControllerState.REQUESTING_OFFBOARD:
            if self.mavros_state is not None and self.mavros_state.mode == 'OFFBOARD':
                self.get_logger().info('OFFBOARD mode active!')
                self.get_logger().info('Arming drone...')
                self.state = ControllerState.ARMING
                self.arm_drone(True)
                self.last_service_call_time = time.time()
            else:
                if time.time() - self.last_service_call_time > self.SERVICE_RETRY_INTERVAL:
                    self.get_logger().info('Retrying OFFBOARD mode request...')
                    self.set_mode('OFFBOARD')
                    self.last_service_call_time = time.time()

        elif self.state == ControllerState.ARMING:
            if self.mavros_state is not None and self.mavros_state.armed:
                self.get_logger().info('Drone armed!')
                self.get_logger().info('=' * 50)
                self.get_logger().info('READY FOR FLIGHT - Use WASD/ZX/QE to control')
                self.get_logger().info('W/S: Forward/Back | A/D: Left/Right')
                self.get_logger().info('X/Z: Up/Down | Q/E: Yaw Left/Right')
                self.get_logger().info('ESC: Emergency Stop')
                self.get_logger().info('=' * 50)
                self.state = ControllerState.ACTIVE
            else:
                if time.time() - self.last_service_call_time > self.SERVICE_RETRY_INTERVAL:
                    self.get_logger().info('Retrying arm request...')
                    self.arm_drone(True)
                    self.last_service_call_time = time.time()

        elif self.state == ControllerState.ACTIVE:
            if self.current_key:
                self.get_logger().debug(
                    f'Key: {self.current_key} -> '
                    f'vx={velocity_cmd.twist.linear.x:.1f}, '
                    f'vy={velocity_cmd.twist.linear.y:.1f}, '
                    f'vz={velocity_cmd.twist.linear.z:.1f}, '
                    f'wz={velocity_cmd.twist.angular.z:.1f}'
                )

        elif self.state == ControllerState.EMERGENCY_STOP:
            self.get_logger().warn('EMERGENCY STOP - Disarming...')
            zero_cmd = TwistStamped()
            zero_cmd.header.stamp = self.get_clock().now().to_msg()
            zero_cmd.header.frame_id = "base_link"
            self.velocity_publisher.publish(zero_cmd)
            self.arm_drone(False)
            self.state = ControllerState.SHUTDOWN
            self.get_logger().info('Drone disarmed. Shutting down...')
            raise SystemExit

    def destroy_node(self):
        self.get_logger().info('Shutting down velocity controller...')
        if (self.mavros_state is not None and
                self.mavros_state.armed and
                self.arming_client.service_is_ready()):
            self.get_logger().warn('Disarming drone before shutdown...')
            try:
                self.arm_drone(False)
            except Exception as e:
                self.get_logger().error(f'Failed to disarm: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()