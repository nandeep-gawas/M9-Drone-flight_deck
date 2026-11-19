"""
M9 Drone Flight Deck Package
Clean, modular Linux desktop flight controller for KY UFO / FLOW-UFO drones.
Zero camera / video / RTSP dependencies.
"""

from .state import (
    FlightState,
    ProtocolType,
    SpeedMode,
    SPEED_SCALES,
    ControlStickMode,
    TrajectoryPattern,
)
from .protocol import (
    DroneProtocol,
    TC_CmdBits,
    TC_AuxBits,
    GL_CmdBits,
    STICK_CENTER,
    STICK_MIN,
    STICK_MAX,
    STICK_IDLE_THROTTLE,
    TRIM_MAX_OFFSET,
    TRIM_STEP,
    DURATION_TAKEOFF,
    DURATION_LAND,
    DURATION_EMERGENCY,
    DURATION_GYRO_CALIB,
    DURATION_FLIP,
    DURATION_LANDING_DESCENT,
    DURATION_RETURN_HOME,
)
from .network import DroneNetworkClient
from .controller import DroneDesktopWindow

__all__ = [
    "FlightState",
    "ProtocolType",
    "SpeedMode",
    "SPEED_SCALES",
    "ControlStickMode",
    "TrajectoryPattern",
    "DroneProtocol",
    "TC_CmdBits",
    "TC_AuxBits",
    "GL_CmdBits",
    "STICK_CENTER",
    "STICK_MIN",
    "STICK_MAX",
    "STICK_IDLE_THROTTLE",
    "TRIM_MAX_OFFSET",
    "TRIM_STEP",
    "DURATION_TAKEOFF",
    "DURATION_LAND",
    "DURATION_EMERGENCY",
    "DURATION_GYRO_CALIB",
    "DURATION_FLIP",
    "DURATION_LANDING_DESCENT",
    "DURATION_RETURN_HOME",
    "DroneNetworkClient",
    "DroneDesktopWindow",
]
