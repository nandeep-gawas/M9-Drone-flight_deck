"""
Flight State & Operational Mode Enums for M9 Drone Flight Deck.
Reverse-engineered from the official Android KY UFO application.
"""

from enum import Enum, IntEnum
from typing import Dict


class FlightState(Enum):
    """
    Explicit flight states ensuring safe command sequencing.
    Prevents conflicting commands and runaway flight loops.
    """
    DISARMED = "DISARMED"        # Ground: Motors off, throttle 0, safe to calibrate
    ARMED = "ARMED"              # Ground: Ready for takeoff, neutral hover throttle (128)
    TAKING_OFF = "TAKING_OFF"    # Transition: 1000ms pulse on Bit 0, climbing
    FLYING = "FLYING"            # Airborne: Stable optical-flow/baro hover, manual controls active
    LANDING = "LANDING"          # Transition: 1000ms pulse on Bit 0, autonomous descent
    EMERGENCY = "EMERGENCY"      # Override: Immediate motor power cut mid-air


class ProtocolType(Enum):
    """Drone protocol modes."""
    TC_21BYTE = "TC_21BYTE"  # Default for FLOW-UFO / BL60X hardware (deviceType == 10)
    GL_9BYTE = "GL_9BYTE"    # Legacy KY UFO / E58 hardware


class SpeedMode(IntEnum):
    """Client-side joystick deflection limits."""
    LOW = 0       # 30% deflection limit (Beginner / Indoor)
    MEDIUM = 1    # 60% deflection limit (Intermediate)
    HIGH = 2      # 100% full deflection (Sport / Outdoor)


SPEED_SCALES: Dict[SpeedMode, float] = {
    SpeedMode.LOW: 0.30,
    SpeedMode.MEDIUM: 0.60,
    SpeedMode.HIGH: 1.00,
}


class ControlStickMode(Enum):
    """
    Joystick layout configuration.
    Mode 2 is standard in the US/world; Mode 1 is popular in Europe/Asia.
    """
    MODE_2_US = "MODE_2"  # Left: Throttle/Yaw, Right: Pitch/Roll (Default)
    MODE_1_EU = "MODE_1"  # Left: Pitch/Yaw, Right: Throttle/Roll (Right-Hand Throttle)


class TrajectoryPattern(Enum):
    """Autonomous trajectory / track flight patterns from KY UFO app."""
    SQUARE = "SQUARE"        # Box pattern
    CIRCLE = "CIRCLE"        # 360-degree orbit
    FIGURE_8 = "FIGURE_8"    # Figure-8 / S-curve
    TRIANGLE = "TRIANGLE"    # Triangular flight path
