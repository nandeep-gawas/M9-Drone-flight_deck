"""
Flight State & Operational Mode Enums for M9 Drone Flight Deck.
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
