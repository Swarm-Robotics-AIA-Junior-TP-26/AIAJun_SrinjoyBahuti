#!/usr/bin/env python3
from pymavlink import mavutil
import time
import math
import signal

PORT = "udpin:0.0.0.0:14540"
HZ = 20
DT = 1.0 / HZ
START_TIME = time.time()

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MODE_OFFBOARD = PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16

IGNORE_PX = 1 << 0
IGNORE_PY = 1 << 1
IGNORE_PZ = 1 << 2
IGNORE_AX = 1 << 6
IGNORE_AY = 1 << 7
IGNORE_AZ = 1 << 8
IGNORE_YAW = 1 << 10
IGNORE_YAWRATE = 1 << 11
VEL_ONLY_MASK = (
    IGNORE_PX | IGNORE_PY | IGNORE_PZ |
    IGNORE_AX | IGNORE_AY | IGNORE_AZ |
    IGNORE_YAW | IGNORE_YAWRATE
)

ALT_Z = -4.0
TAKEOFF_UP_VZ = -1.0
TAKEOFF_TIME = 3.5

MAX_HORZ_V = 2.0
MAX_VERT_V = 1.0
SLOW_RADIUS = 2.0
REACHED_RADIUS = 0.6
HOLD_TIME_AT_WP = 1.0

USE_AXIS_SWAP = True

PATH_OFFSETS_XY = [
    (0,   0),
    (10,  0),
    (10,  6),
    (20,  6),
    (20,  0),
    (30,  0),
]

def now_ms():
    return int((time.time() - START_TIME) * 1000) & 0xFFFFFFFF

master = mavutil.mavlink_connection(PORT, source_system=255, source_component=190)

pos_ned = {"x": None, "y": None, "z": None, "t": 0.0}
last_hb = {"custom_mode": None, "base_mode": None, "t": 0.0}

def handle_local_position(msg):
    pos_ned["x"] = float(msg.x)
    pos_ned["y"] = float(msg.y)
    pos_ned["z"] = float(msg.z)
    pos_ned["t"] = time.time()

def handle_heartbeat(msg):
    last_hb["custom_mode"] = int(msg.custom_mode)
    last_hb["base_mode"] = int(msg.base_mode)
    last_hb["t"] = time.time()

def recv_nonblocking():
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "LOCAL_POSITION_NED":
            handle_local_position(msg)
        elif t == "HEARTBEAT":
            handle_heartbeat(msg)

def send_vel_local(vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        now_ms(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VEL_ONLY_MASK,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

def cmd_long(command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        command, 0,
        p1, p2, p3, p4, p5, p6, p7
    )

def arm():
    cmd_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

def land():
    cmd_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0)

def rtl():
    try:
        master.set_mode("RTL")
        return True
    except Exception:
        pass
    try:
        cmd_long(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0)
        return True
    except Exception:
        pass
    return False

def set_mode_offboard_once():
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MODE_OFFBOARD
    )
    cmd_long(
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MODE_OFFBOARD
    )

def is_offboard_recent():
    if last_hb["custom_mode"] is None:
        return False
    if time.time() - last_hb["t"] > 1.0:
        return False
    return last_hb["custom_mode"] == PX4_CUSTOM_MODE_OFFBOARD

should_exit = False
def on_sigint(sig, frame):
    global should_exit
    should_exit = True

signal.signal(signal.SIGINT, on_sigint)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def compute_velocity_to_wp(curr, wp):
    dx = wp[0] - curr[0]
    dy = wp[1] - curr[1]
    dz = wp[2] - curr[2]
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 1e-6:
        return 0.0, 0.0, 0.0, dist
    scale = 1.0
    if dist < SLOW_RADIUS:
        scale = max(0.2, dist / SLOW_RADIUS)
    ux, uy, uz = dx / dist, dy / dist, dz / dist
    vh = MAX_HORZ_V * scale
    vv = MAX_VERT_V * scale
    vx = clamp(ux * vh, -MAX_HORZ_V, MAX_HORZ_V)
    vy = clamp(uy * vh, -MAX_HORZ_V, MAX_HORZ_V)
    vz = clamp(uz * vv, -MAX_VERT_V, MAX_VERT_V)
    return vx, vy, vz, dist

def wait_for_pos(timeout_s=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s and not should_exit:
        recv_nonblocking()
        if pos_ned["x"] is not None:
            return True
        time.sleep(0.05)
    return False

def make_waypoints_from_offsets(home_x, home_y, alt_z):
    wps = []
    for dx, dy in PATH_OFFSETS_XY:
        if USE_AXIS_SWAP:
            x = home_x + dy
            y = home_y + dx
        else:
            x = home_x + dx
            y = home_y + dy
        wps.append((x, y, alt_z))
    return wps

def main():
    master.wait_heartbeat()
    master.target_system = 1
    master.target_component = 1

    if not wait_for_pos():
        return

    home_x = pos_ned["x"]
    home_y = pos_ned["y"]

    waypoints = make_waypoints_from_offsets(home_x, home_y, ALT_Z)

    t0 = time.time()
    while time.time() - t0 < 2.0 and not should_exit:
        send_vel_local(0.0, 0.0, 0.0)
        recv_nonblocking()
        time.sleep(DT)

    arm()
    time.sleep(1.0)

    t0 = time.time()
    while time.time() - t0 < 8.0 and not should_exit:
        send_vel_local(0.0, 0.0, 0.0)
        set_mode_offboard_once()
        recv_nonblocking()
        if is_offboard_recent():
            break
        time.sleep(0.2)

    if not is_offboard_recent():
        return

    t0 = time.time()
    while time.time() - t0 < TAKEOFF_TIME and not should_exit:
        send_vel_local(0.0, 0.0, TAKEOFF_UP_VZ)
        recv_nonblocking()
        time.sleep(DT)

    for wp in waypoints:
        hold_start = None
        while not should_exit:
            recv_nonblocking()
            curr = (pos_ned["x"], pos_ned["y"], pos_ned["z"])
            if curr[0] is None:
                time.sleep(DT)
                continue
            vx, vy, vz, d = compute_velocity_to_wp(curr, wp)
            if d <= REACHED_RADIUS:
                send_vel_local(0.0, 0.0, 0.0)
                if hold_start is None:
                    hold_start = time.time()
                if time.time() - hold_start >= HOLD_TIME_AT_WP:
                    break
            else:
                hold_start = None
                send_vel_local(vx, vy, vz)
            time.sleep(DT)

    if should_exit:
        land()
        return

    send_vel_local(0.0, 0.0, 0.0)
    time.sleep(0.2)
    rtl()

if __name__ == "__main__":
    main()
