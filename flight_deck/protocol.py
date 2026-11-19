"""
M9 Drone Flight Deck - Protocol Engine
Reverse-engineered from the official Android KY UFO APK (com.cooingdv.kyufo).

Supports:
1. 21-Byte TC Protocol (FLOW-UFO / BL60X Bouffalo Lab hardware).
2. 9-Byte GL Protocol (Legacy KY UFO / E58 hardware).
3. Trim calibration offsets (Roll, Pitch, Yaw).
4. Directional flips, Headless, Altitude Hold, Lights, Gyro Calib, One-Key Return.

Zero camera / RTSP / video dependencies.
"""

from enum import IntEnum
from typing import Tuple

from .state import SpeedMode, SPEED_SCALES

# Stick Range Constants (Official Android APK RockerView)
STICK_CENTER = 128  # 0x80 (Neutral hover)
STICK_MIN = 1       # 0x01 (Minimum deflection)
STICK_MAX = 255     # 0xFF (Maximum deflection)
STICK_IDLE_THROTTLE = 0

# Max Trim Offset per Axis (steps of 2 from -24 to +24)
TRIM_MAX_OFFSET = 24
TRIM_STEP = 2

# Command Timers in seconds (Reverse-engineered from Android Dalvik bytecode)
DURATION_TAKEOFF = 1.0          # 1000 ms pulse (FlyController.setFastFly)
DURATION_LAND = 1.0             # 1000 ms pulse (FlyController.setFastDrop)
DURATION_EMERGENCY = 1.0        # 1000 ms pulse (FlyController.setEmergencyStop)
DURATION_GYRO_CALIB = 2.0       # 2000 ms pulse (FlyController.setGyroCorrection)
DURATION_FLIP = 0.6             # 600 ms pulse (DeviceBLFragment.turn360)
DURATION_LANDING_DESCENT = 4.0  # Estimated descent duration until touchdown
DURATION_RETURN_HOME = 3.0      # 3.0s auto-return pulse window


class TC_CmdBits(IntEnum):
    """
    Byte 7 bitfield for TC 21-byte protocol (FLOW-UFO / BL60X hardware).
    Byte 7 is command byte (index 6 of 20-byte inner payload).
    """
    TAKEOFF_LAND = 0x01   # 1 = Takeoff or Land active pulse (1.0s), 0 = Normal / Hover
    EMERGENCY    = 0x02   # 1 = Emergency motor cut pulse (1.0s active)
    GYRO_CALIB   = 0x04   # 1 = Gyro calibration pulse (2.0s active)
    FLIP         = 0x08   # 1 = 360-degree flip pulse (0.6s active)
    LIGHT_ON     = 0x10   # 1 = Drone LED lights active (default ON)


class TC_AuxBits(IntEnum):
    """
    Byte 8 bitfield for TC 21-byte protocol.
    Byte 8 is auxiliary byte (index 7 of 20-byte inner payload).
    """
    HEADLESS      = 0x01   # 1 = Headless flight mode enabled
    ALTITUDE_HOLD = 0x02   # 1 = Optical flow / Fixed-height mode active (default ON)


class GL_CmdBits(IntEnum):
    """Byte 6 bitfield for GL 9-byte protocol (Legacy KY UFO / E58)."""
    TAKEOFF       = 0x01   # 1 = Takeoff pulse
    LAND          = 0x02   # 1 = Land pulse
    EMERGENCY     = 0x04   # 1 = Emergency stop pulse
    FLIP          = 0x08   # 1 = 360-degree flip pulse
    HEADLESS      = 0x10   # 1 = Headless mode toggle
    RETURN_HOME   = 0x20   # 1 = One-Key Return (isFastReturn / ARG_STATE_COME_BACK)
    ARM_UNLOCK    = 0x20   # 1 = Motor unlock in GL mode
    LIGHT_ON      = 0x40   # 1 = Drone LED lights active
    GYRO_CALIB    = 0x80   # 1 = Gyro calibration pulse (2.0s)


class DroneProtocol:
    """
    Stateless encoder and validator for KY UFO / FLOW-UFO UDP packets.
    """

    @staticmethod
    def clamp_stick(val: int) -> int:
        """Clamps stick integer value between 1 and 255 (0x01 to 0xFF)."""
        return max(STICK_MIN, min(STICK_MAX, int(val)))

    @staticmethod
    def apply_trims(
        roll: int, pitch: int, yaw: int,
        trim_roll: int, trim_pitch: int, trim_yaw: int
    ) -> Tuple[int, int, int]:
        """
        Applies physical drift trim offsets to Roll, Pitch, and Yaw.
        Offsets range from -24 to +24.
        """
        r = DroneProtocol.clamp_stick(roll + trim_roll)
        p = DroneProtocol.clamp_stick(pitch + trim_pitch)
        y = DroneProtocol.clamp_stick(yaw + trim_yaw)
        return r, p, y

    @staticmethod
    def scale_deflection(neutral_val: int, deflection: int, mode: SpeedMode) -> int:
        """
        Applies client-side deflection limit based on selected speed mode.
        deflection: signed deviation from 128 (-127 to +127)
        """
        scale = SPEED_SCALES.get(mode, 1.0)
        scaled_deflection = int(round(deflection * scale))
        return DroneProtocol.clamp_stick(neutral_val + scaled_deflection)

    @staticmethod
    def build_heartbeat() -> bytes:
        """
        Builds the 1 Hz heartbeat keepalive packet.
        Wire payload: [0x01, 0x01]
        """
        return bytes([0x01, 0x01])

    @staticmethod
    def build_teardown() -> bytes:
        """
        Builds session termination / teardown packet sent on controller exit.
        Wire payload: [0x08, 0x01]
        """
        return bytes([0x08, 0x01])

    @staticmethod
    def build_tc_packet(
        roll: int = STICK_CENTER,
        pitch: int = STICK_CENTER,
        throttle: int = STICK_CENTER,
        yaw: int = STICK_CENTER,
        is_taking_off: bool = False,
        is_landing: bool = False,
        is_emergency: bool = False,
        is_gyro_calib: bool = False,
        is_flip: bool = False,
        is_light_on: bool = True,
        is_headless: bool = False,
        is_altitude_hold: bool = True,
        is_fast_return: bool = False,
        trim_roll: int = 0,
        trim_pitch: int = 0,
        trim_yaw: int = 0,
    ) -> bytes:
        """
        Constructs a 21-byte TC packet (FLOW-UFO / BL60X hardware).
        Format: [0x03, 0x66, 0x14, R, P, T, Y, Cmd, Aux, 0x00*10, Checksum, 0x99]
        """
        # Apply trim offsets
        r, p, y = DroneProtocol.apply_trims(roll, pitch, yaw, trim_roll, trim_pitch, trim_yaw)
        t = max(0, min(STICK_MAX, int(throttle)))

        # In TC mode, One-Key Return deflects pitch backward toward pilot
        if is_fast_return:
            p = min(STICK_MAX, STICK_CENTER + 60)

        # Assemble Byte 7 (cmd_byte) bitfield:
        # v1 = isFastFly ? 1 : (isFastDrop ? 1 : 0);
        # if (isEmergencyStop) v1 += 2;
        # if (isGyroCorrection) v1 += 4;
        # if (isCircleTurnEnd) v1 += 8;
        # if (isOpenLight) v1 += 16;
        cmd_byte = 0x00
        if is_taking_off or is_landing:
            cmd_byte |= TC_CmdBits.TAKEOFF_LAND   # 0x01 (Active pulse only!)
        if is_emergency:
            cmd_byte |= TC_CmdBits.EMERGENCY      # 0x02
        if is_gyro_calib:
            cmd_byte |= TC_CmdBits.GYRO_CALIB     # 0x04
        if is_flip:
            cmd_byte |= TC_CmdBits.FLIP           # 0x08
        if is_light_on:
            cmd_byte |= TC_CmdBits.LIGHT_ON       # 0x10

        # Assemble Byte 8 (aux_byte) bitfield:
        # v2 = isNoHeadMode ? 1 : 0;
        # if (isFixedHeightMode) v2 += 2;
        aux_byte = 0x00
        if is_headless:
            aux_byte |= TC_AuxBits.HEADLESS       # 0x01
        if is_altitude_hold:
            aux_byte |= TC_AuxBits.ALTITUDE_HOLD  # 0x02

        # Checksum: XOR of payload bytes 3 through 8
        checksum = r ^ p ^ t ^ y ^ cmd_byte ^ aux_byte

        packet = bytearray([
            0x03,        # Byte 0: Transport wrapper prefix
            0x66,        # Byte 1: Header
            0x14,        # Byte 2: Length (20)
            r,           # Byte 3: Roll
            p,           # Byte 4: Pitch
            t,           # Byte 5: Throttle
            y,           # Byte 6: Yaw
            cmd_byte,    # Byte 7: Command Flags
            aux_byte,    # Byte 8: Auxiliary Flags (Altitude Hold & Headless)
        ])
        packet.extend([0x00] * 10)  # Bytes 9..18: 10 reserved bytes
        packet.append(checksum)     # Byte 19: Checksum
        packet.append(0x99)         # Byte 20: Tail delimiter

        return bytes(packet)

    @staticmethod
    def build_gl_packet(
        roll: int = STICK_CENTER,
        pitch: int = STICK_CENTER,
        throttle: int = STICK_CENTER,
        yaw: int = STICK_CENTER,
        is_taking_off: bool = False,
        is_landing: bool = False,
        is_emergency: bool = False,
        is_gyro_calib: bool = False,
        is_flip: bool = False,
        is_armed: bool = True,
        is_light_on: bool = True,
        is_headless: bool = False,
        is_fast_return: bool = False,
        trim_roll: int = 0,
        trim_pitch: int = 0,
        trim_yaw: int = 0,
    ) -> bytes:
        """
        Constructs a 9-byte GL packet (Legacy KY UFO / E58 hardware).
        Format: [0x03, 0x66, R, P, T, Y, CmdFlags, Checksum, 0x99]
        """
        r, p, y = DroneProtocol.apply_trims(roll, pitch, yaw, trim_roll, trim_pitch, trim_yaw)
        t = max(0, min(STICK_MAX, int(throttle)))

        cmd_flags = 0x00
        if is_taking_off:
            cmd_flags |= GL_CmdBits.TAKEOFF     # 0x01
        if is_landing:
            cmd_flags |= GL_CmdBits.LAND        # 0x02
        if is_emergency:
            cmd_flags |= GL_CmdBits.EMERGENCY   # 0x04
        if is_flip:
            cmd_flags |= GL_CmdBits.FLIP        # 0x08
        if is_headless:
            cmd_flags |= GL_CmdBits.HEADLESS    # 0x10
        if is_fast_return:
            cmd_flags |= GL_CmdBits.RETURN_HOME # 0x20
        elif is_armed:
            cmd_flags |= GL_CmdBits.ARM_UNLOCK  # 0x20
        if is_light_on:
            cmd_flags |= GL_CmdBits.LIGHT_ON    # 0x40
        if is_gyro_calib:
            cmd_flags |= GL_CmdBits.GYRO_CALIB  # 0x80

        checksum = r ^ p ^ t ^ y ^ cmd_flags

        packet = bytes([
            0x03,       # Byte 0: Transport prefix
            0x66,       # Byte 1: Header
            r,          # Byte 2: Roll
            p,          # Byte 3: Pitch
            t,          # Byte 4: Throttle
            y,          # Byte 5: Yaw
            cmd_flags,  # Byte 6: Command Flags
            checksum,   # Byte 7: Checksum
            0x99,       # Byte 8: Tail delimiter
        ])
        return packet

    @staticmethod
    def verify_packet(packet: bytes) -> Tuple[bool, str]:
        """
        Validates packet length, headers, tails, and checksum.
        Returns (is_valid, reason).
        """
        if not packet:
            return False, "Empty packet"

        if packet[0] != 0x03 or packet[1] != 0x66:
            return False, f"Invalid header: {packet[:2].hex()}"

        if packet[-1] != 0x99:
            return False, f"Invalid tail byte: {hex(packet[-1])}"

        if len(packet) == 21:
            # 21-byte TC packet
            if packet[2] != 0x14:
                return False, f"Invalid TC length byte: {hex(packet[2])}"
            expected_chk = 0
            for b in packet[3:9]:
                expected_chk ^= b
            actual_chk = packet[19]
            if expected_chk != actual_chk:
                return False, f"TC Checksum mismatch: expected {hex(expected_chk)}, got {hex(actual_chk)}"
            return True, "Valid 21-byte TC packet"

        elif len(packet) == 9:
            # 9-byte GL packet
            expected_chk = 0
            for b in packet[2:7]:
                expected_chk ^= b
            actual_chk = packet[7]
            if expected_chk != actual_chk:
                return False, f"GL Checksum mismatch: expected {hex(expected_chk)}, got {hex(actual_chk)}"
            return True, "Valid 9-byte GL packet"

        return False, f"Unknown packet length: {len(packet)} bytes"
