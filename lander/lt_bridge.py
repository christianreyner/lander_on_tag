#!/usr/bin/env python3
import socket
import time
import math
from pymavlink import mavutil

# MAVLink connection to your autopilot/SITL
MAV_CONNECTION_STRING = 'udpin:127.0.0.1:14550'

# UDP input from your C sender: lines "dx dy dz\n" in meters (body frame)
BRIDGE_LISTEN_IP = '127.0.0.1'
BRIDGE_LISTEN_PORT = 6000

# Publish rate and data freshness policy
SEND_HZ = 20.0
FRESHNESS_TIMEOUT = 0.5  # seconds without new data => position_valid=0

# Set True for ArduPilot, False for PX4
TARGET_IS_ARDUPILOT = True

def connect_link(conn_str: str):
    print(f"INFO: Connecting via MAVLink to {conn_str}")
    mav = mavutil.mavlink_connection(conn_str, source_system=1)
    print("INFO: Waiting for heartbeat...")
    hb = mav.wait_heartbeat(timeout=5)
    if hb:
        print(f"INFO: Heartbeat from system {mav.target_system} component {mav.target_component}")
    else:
        print("WARN: No heartbeat; proceeding anyway.")
    return mav

def parse_dx_dy_dz(line: str):
    parts = line.strip().split()
    if len(parts) < 3:
        return None
    try:
        dx = float(parts[0]); dy = float(parts[1]); dz = float(parts[2])
        return dx, dy, dz
    except ValueError:
        return None

def send_landing_target_positional(mav, dx, dy, dz, valid, is_ardupilot=True):
    """
    Send LANDING_TARGET using MAVLink 2 positional fields only.
    - ArduPilot: BODY_FRD frame, requires distance when valid and position_valid=1.
    - PX4: LOCAL_NED frame.
    When valid==False, set position_valid=0 and distance can be 0 (ignored).
    """
    t_usec = int(time.time() * 1e6)
    frame = mavutil.mavlink.MAV_FRAME_BODY_FRD if is_ardupilot else mavutil.mavlink.MAV_FRAME_LOCAL_NED

    # Distance required for ArduPilot when valid
    distance = math.sqrt(dx*dx + dy*dy + dz*dz) if valid else 0.0
    position_valid = 1 if valid else 0

    # Orientation quaternion: identity (no orientation offset)
    q = [1.0, 0.0, 0.0, 0.0]
    target_type = mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL

    # The MAVLink 2 signature in common.xml is:
    # time_usec, target_num, frame, angle_x, angle_y, distance, size_x, size_y, x, y, z, q, type, position_valid
    # We zero the legacy angle/size fields; receiver ignores them when position_valid=1.
    try:
        mav.mav.landing_target_send(
            t_usec,
            0,                  # target_num
            frame,
            0.0, 0.0,           # angle_x, angle_y (ignored)
            float(distance),    # distance
            0.0, 0.0,           # size_x, size_y
            float(dx), float(dy), float(dz),
            q,
            target_type,
            position_valid
        )
        return
    except TypeError:
        return
 

def main():
    # UDP socket to receive dx dy dz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((BRIDGE_LISTEN_IP, BRIDGE_LISTEN_PORT))
    sock.settimeout(0.1)
    print(f"INFO: Listening for 'dx dy dz' on UDP {BRIDGE_LISTEN_IP}:{BRIDGE_LISTEN_PORT}")

    mav = connect_link(MAV_CONNECTION_STRING)

    period = 1.0 / SEND_HZ
    last_send = 0.0
    last_rx_time = 0.0
    # Initialize with zeros; marked invalid until fresh data arrives
    dx = dy = dz = 0.0

    stack = "ArduPilot (BODY_FRD)" if TARGET_IS_ARDUPILOT else "PX4 (LOCAL_NED)"
    print(f"INFO: Using positional LANDING_TARGET for {stack}.")
    print(f"INFO: Publishing at {SEND_HZ} Hz. position_valid=0 if no data within {FRESHNESS_TIMEOUT}s.")

    try:
        while True:
            # Try receive latest measurement (non-blocking)
            try:
                data, _ = sock.recvfrom(1024)
                line = data.decode('utf-8', errors='ignore')
                parsed = parse_dx_dy_dz(line)
                if parsed:
                    dx, dy, dz = parsed
                    last_rx_time = time.time()
            except socket.timeout:
                pass

            now = time.time()
            if now - last_send >= period:
                valid = (now - last_rx_time) <= FRESHNESS_TIMEOUT
                send_landing_target_positional(
                    mav,
                    dx if valid else 0.0,
                    dy if valid else 0.0,
                    dz if valid else 0.0,
                    valid=valid,
                    is_ardupilot=TARGET_IS_ARDUPILOT
                )
                last_send = now

    except KeyboardInterrupt:
        print("\nINFO: Stopped by user.")
    finally:
        try:
            mav.close()
        except:
            pass
        sock.close()
        print("INFO: Closed.")

if __name__ == "__main__":
    main()
