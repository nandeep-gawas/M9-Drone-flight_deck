#!/usr/bin/env python3
"""
M9 Drone Flight Deck - Launcher Script
Launches the Linux desktop flight controller for FALAFLY F1 / M9 Ultra-Light / FLOW-UFO drones.

Usage:
    python3 run_controller.py
    python3 run_controller.py --loopback
    python3 run_controller.py --ip 192.168.1.1 --port 7099
"""

import sys
import os
import argparse
import logging

from PyQt5.QtWidgets import QApplication

# Ensure flight_deck package is in python path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from flight_deck.controller import DroneDesktopWindow

# Setup root logging
log_path = os.path.join(current_dir, "m9_flight_deck.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.FileHandler(log_path, mode="w"),
        logging.StreamHandler(sys.stdout)
    ]
)


def main():
    parser = argparse.ArgumentParser(description="M9 Drone Flight Deck — Linux Controller")
    parser.add_argument("--ip", default="192.168.1.1", help="Drone Wi-Fi IP address (default: 192.168.1.1)")
    parser.add_argument("--port", type=int, default=7099, help="UDP Control Port (default: 7099)")
    parser.add_argument("--loopback", action="store_true", help="Run in loopback bench testing mode (no drone required)")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = DroneDesktopWindow(drone_ip=args.ip, ctrl_port=args.port, loopback=args.loopback)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
