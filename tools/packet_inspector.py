#!/usr/bin/env python3
"""
M9 Drone Flight Deck — Standalone Packet Inspector Tool
Listens on UDP port 7099 to inspect, parse, and verify flight control packets in real-time.

Use for:
  - Pre-flight bench testing without propellers or physical drone.
  - Verifying takeoff/landing pulses (Bit 0 duration ~1000ms).
  - Verifying altitude hold bit (Bit 1 in Byte 8) and neutral hover (Throttle=128).
  - Verifying emergency stop (Bit 1 in Byte 7, Throttle=0).

Usage:
  python3 tools/packet_inspector.py
  python3 tools/packet_inspector.py --port 7099
"""

import sys
import os
import socket
import argparse
import time

# Ensure package root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from flight_deck.protocol import DroneProtocol, TC_CmdBits, TC_AuxBits, GL_CmdBits


def decode_tc_packet(pkt: bytes) -> str:
    """Decodes a 21-byte TC packet into a human-readable diagnosis string."""
    r = pkt[3]
    p = pkt[4]
    t = pkt[5]
    y = pkt[6]
    cmd = pkt[7]
    aux = pkt[8]
    chk = pkt[19]

    # Command bits
    is_takeoff_land = bool(cmd & TC_CmdBits.TAKEOFF_LAND)
    is_emergency = bool(cmd & TC_CmdBits.EMERGENCY)
    is_gyro = bool(cmd & TC_CmdBits.GYRO_CALIB)
    is_flip = bool(cmd & TC_CmdBits.FLIP)
    is_light = bool(cmd & TC_CmdBits.LIGHT_ON)

    # Aux bits
    is_headless = bool(aux & TC_AuxBits.HEADLESS)
    is_alt_hold = bool(aux & TC_AuxBits.ALTITUDE_HOLD)

    valid, reason = DroneProtocol.verify_packet(pkt)
    status_str = "VALID" if valid else f"INVALID ({reason})"

    actions = []
    if is_takeoff_land:
        actions.append("TAKEOFF/LAND_PULSE")
    if is_emergency:
        actions.append("EMERGENCY_STOP")
    if is_gyro:
        actions.append("GYRO_CALIB")
    if is_flip:
        actions.append("FLIP")
    if is_headless:
        actions.append("HEADLESS")
    if is_alt_hold:
        actions.append("ALT_HOLD")
    if not actions:
        actions.append("HOVER/MANUAL")

    action_tag = " | ".join(actions)

    return (
        f"[{status_str}] R={r:3d} P={p:3d} T={t:3d} Y={y:3d} | "
        f"Cmd=0x{cmd:02X} Aux=0x{aux:02X} Chk=0x{chk:02X} | {action_tag}"
    )


def decode_gl_packet(pkt: bytes) -> str:
    """Decodes a 9-byte GL packet into human-readable string."""
    r = pkt[2]
    p = pkt[3]
    t = pkt[4]
    y = pkt[5]
    cmd = pkt[6]
    chk = pkt[7]

    valid, reason = DroneProtocol.verify_packet(pkt)
    status_str = "VALID" if valid else f"INVALID ({reason})"

    return f"[{status_str}] R={r:3d} P={p:3d} T={t:3d} Y={y:3d} | Cmd=0x{cmd:02X} Chk=0x{chk:02X}"


def main():
    parser = argparse.ArgumentParser(description="M9 Drone Packet Inspector")
    parser.add_argument("--bind", default="0.0.0.0", help="Bind IP address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=7099, help="UDP listening port (default: 7099)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.bind, args.port))
    except Exception as e:
        print(f"[ERROR] Failed to bind to {args.bind}:{args.port} - {e}")
        sys.exit(1)

    print("=" * 78)
    print(f" M9 DRONE PACKET INSPECTOR — Listening on UDP {args.bind}:{args.port}")
    print(" Press Ctrl+C to exit.")
    print("=" * 78)

    pkt_count = 0
    try:
        while True:
            data, addr = sock.recvfrom(2048)
            pkt_count += 1
            hex_data = " ".join(f"{b:02X}" for b in data)
            timestamp = time.strftime("%H:%M:%S")

            if len(data) == 21:
                diag = decode_tc_packet(data)
                print(f"[{timestamp}] #{pkt_count:05d} {addr[0]}:{addr[1]} [21B] {diag}")
            elif len(data) == 9:
                diag = decode_gl_packet(data)
                print(f"[{timestamp}] #{pkt_count:05d} {addr[0]}:{addr[1]} [9B]  {diag}")
            elif len(data) == 2:
                if data == bytes([0x01, 0x01]):
                    tag = "HEARTBEAT_KEEPALIVE [01 01]"
                elif data == bytes([0x08, 0x01]):
                    tag = "TEARDOWN_DISCONNECT [08 01]"
                else:
                    tag = f"2-BYTE SIGNAL: {hex_data}"
                print(f"[{timestamp}] #{pkt_count:05d} {addr[0]}:{addr[1]} [2B]  {tag}")
            else:
                print(f"[{timestamp}] #{pkt_count:05d} {addr[0]}:{addr[1]} [{len(data)}B] {hex_data}")

    except KeyboardInterrupt:
        print("\n[INFO] Packet Inspector stopped by user.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
