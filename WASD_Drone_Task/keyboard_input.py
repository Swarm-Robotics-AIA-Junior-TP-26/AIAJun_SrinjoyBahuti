#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
from pynput import keyboard


class KeyboardInputNode(Node):

    VALID_KEYS = {'w', 's', 'a', 'd', 'z', 'x', 'q', 'e'}

    def __init__(self):
        super().__init__('keyboard_input_node')

        self.key_publisher = self.create_publisher(
            String,
            '/keypress',
            10
        )

        self.current_key = ''
        self.key_lock = threading.Lock()
        self.shutdown_requested = False

        self.publish_timer = self.create_timer(0.05, self.publish_key_callback)

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        self.get_logger().info('Keyboard input node started')
        self.get_logger().info('Controls: W/S=Fwd/Back, A/D=Left/Right, Z/X=Down/Up, Q/E=Yaw')
        self.get_logger().info('Press ESC for emergency stop')

    def on_press(self, key):
        """Called when a key is pressed."""
        try:
            k = key.char.lower()
            if k in self.VALID_KEYS:
                with self.key_lock:
                    self.current_key = k
                self.get_logger().info(f'Key pressed: {k}')
        except AttributeError:

            if key == keyboard.Key.esc:
                self.get_logger().warn('ESC pressed - emergency stop!')
                with self.key_lock:
                    self.current_key = 'esc'
                self.shutdown_requested = True

    def on_release(self, key):
        """Called when a key is released - clear velocity."""
        with self.key_lock:
            self.current_key = ''

    def publish_key_callback(self):
        """Publish current key at 20 Hz."""
        with self.key_lock:
            key_to_publish = self.current_key

        msg = String()
        msg.data = key_to_publish
        self.key_publisher.publish(msg)

        if self.shutdown_requested:
            self.get_logger().info('Shutdown requested, stopping node...')
            raise SystemExit

    def destroy_node(self):
        self.shutdown_requested = True
        if self.listener.is_alive():
            self.listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
