"""
Unit Tests for M9 Drone Flight Deck Protocol Engine & Network Client.
Verifies byte-for-byte fidelity with the official Android KY UFO APK decompilation.
"""

import unittest
import sys
import os

# Ensure package root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from flight_deck.state import (
    FlightState,
    ProtocolType,
    SpeedMode,
    ControlStickMode,
    TrajectoryPattern,
)
from flight_deck.protocol import (
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
    DURATION_RETURN_HOME,
)
from flight_deck.network import DroneNetworkClient


class TestDroneProtocol(unittest.TestCase):

    def test_tc_packet_structure_and_length(self):
        pkt = DroneProtocol.build_tc_packet()
        self.assertEqual(len(pkt), 21, "TC packet must be exactly 21 bytes on wire")
        self.assertEqual(pkt[0], 0x03, "Byte 0 must be transport prefix 0x03")
        self.assertEqual(pkt[1], 0x66, "Byte 1 must be header 0x66")
        self.assertEqual(pkt[2], 0x14, "Byte 2 must be payload length 0x14 (20)")
        self.assertEqual(pkt[20], 0x99, "Byte 20 must be tail delimiter 0x99")
        self.assertEqual(pkt[9:19], bytes([0x00] * 10), "Bytes 9-18 must be 10 reserved 0x00 bytes")

    def test_tc_neutral_hover_checksum(self):
        # Default neutral hover: Roll=128, Pitch=128, Throttle=128, Yaw=128
        # is_taking_off=False, is_landing=False -> cmd_byte bit 0 is 0! (NO continuous takeoff pulse)
        # is_light_on=True -> cmd_byte bit 4 is 1 (0x10) -> cmd_byte = 0x10
        # is_altitude_hold=True -> aux_byte bit 1 is 1 (0x02) -> aux_byte = 0x02
        # Checksum = 128 ^ 128 ^ 128 ^ 128 ^ 0x10 ^ 0x02 = 0 ^ 0x12 = 0x12
        pkt = DroneProtocol.build_tc_packet()
        self.assertEqual(pkt[3], 128)
        self.assertEqual(pkt[4], 128)
        self.assertEqual(pkt[5], 128)
        self.assertEqual(pkt[6], 128)
        self.assertEqual(pkt[7], 0x10, "Neutral hover cmd_byte must be 0x10 (LED on, Takeoff bit 0 = 0)")
        self.assertEqual(pkt[8], 0x02, "Aux byte must be 0x02 (Altitude hold bit 1 = 1)")
        self.assertEqual(pkt[19], 0x12, "Checksum for neutral hover must be 0x12")
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_tc_takeoff_pulse(self):
        # During takeoff pulse: is_taking_off=True -> bit 0 of cmd_byte is 1 (0x01).
        # cmd_byte with light on = 0x10 | 0x01 = 0x11.
        # aux_byte = 0x02.
        # Checksum = 0 ^ 0x11 ^ 0x02 = 0x13.
        pkt = DroneProtocol.build_tc_packet(is_taking_off=True)
        self.assertEqual(pkt[7], 0x11, "Takeoff pulse cmd_byte must be 0x11 (bit 0 = 1)")
        self.assertEqual(pkt[8], 0x02, "Aux byte must remain 0x02 (Altitude hold active)")
        self.assertEqual(pkt[19], 0x13, "Checksum during takeoff pulse must be 0x13")
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_tc_landing_pulse(self):
        # During landing pulse: is_landing=True -> bit 0 of cmd_byte is 1 (0x01).
        # cmd_byte with light on = 0x10 | 0x01 = 0x11.
        # aux_byte = 0x02.
        # Checksum = 0 ^ 0x11 ^ 0x02 = 0x13.
        pkt = DroneProtocol.build_tc_packet(is_landing=True)
        self.assertEqual(pkt[7], 0x11, "Landing pulse cmd_byte must be 0x11 (bit 0 = 1)")
        self.assertEqual(pkt[8], 0x02, "Aux byte must remain 0x02 (Altitude hold active)")
        self.assertEqual(pkt[19], 0x13, "Checksum during landing pulse must be 0x13")
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_tc_emergency_stop_pulse(self):
        # Emergency stop sets bit 1 (0x02) and cuts throttle to 0
        pkt = DroneProtocol.build_tc_packet(throttle=0, is_emergency=True)
        self.assertTrue(pkt[7] & TC_CmdBits.EMERGENCY)
        self.assertEqual(pkt[5], 0, "Emergency stop throttle must be 0")
        self.assertEqual(pkt[7], 0x12, "Emergency stop cmd_byte must be 0x12 (bit 1 = 1)")
        # Checksum: 128 ^ 128 ^ 0 ^ 128 ^ 0x12 ^ 0x02 = 128 ^ 0x10 = 0x90
        self.assertEqual(pkt[19], 0x90)
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_tc_gyro_calibration_pulse(self):
        # Gyro calibration sets bit 2 (0x04) in TC mode
        pkt = DroneProtocol.build_tc_packet(is_gyro_calib=True)
        self.assertTrue(pkt[7] & TC_CmdBits.GYRO_CALIB)
        self.assertEqual(pkt[7] & 0x04, 0x04, "TC Gyro calibration bit must be 0x04 (bit 2)")
        self.assertEqual(pkt[7] & 0x80, 0x00, "Bit 7 (0x80) is reserved/unused in TC Byte 7")
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_tc_headless_mode(self):
        # Headless mode sets bit 0 (0x01) in AuxByte (Byte 8) in TC mode!
        # Combined with Altitude Hold (0x02), aux_byte = 0x03
        pkt = DroneProtocol.build_tc_packet(is_headless=True, is_altitude_hold=True)
        self.assertEqual(pkt[7], 0x10, "Headless does not alter Byte 7")
        self.assertEqual(pkt[8], 0x03, "TC Headless bit must be 0x01 in Byte 8 (aux_byte)")
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_tc_trims_application(self):
        # Roll trim +6, Pitch trim -8, Yaw trim +10
        pkt = DroneProtocol.build_tc_packet(
            trim_roll=6,
            trim_pitch=-8,
            trim_yaw=10,
        )
        self.assertEqual(pkt[3], 128 + 6)   # Roll
        self.assertEqual(pkt[4], 128 - 8)   # Pitch
        self.assertEqual(pkt[6], 128 + 10)  # Yaw
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_tc_return_home(self):
        # One-Key Return in TC deflects pitch backward
        pkt = DroneProtocol.build_tc_packet(is_fast_return=True)
        self.assertEqual(pkt[4], 128 + 60)
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_gl_packet_structure_and_length(self):
        pkt = DroneProtocol.build_gl_packet()
        self.assertEqual(len(pkt), 9, "GL packet must be exactly 9 bytes on wire")
        self.assertEqual(pkt[0], 0x03, "Byte 0 must be transport prefix 0x03")
        self.assertEqual(pkt[1], 0x66, "Byte 1 must be header 0x66")
        self.assertEqual(pkt[8], 0x99, "Byte 8 must be tail delimiter 0x99")
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_gl_gyro_calibration_pulse(self):
        # In GL mode, gyro calibration IS bit 7 (0x80)
        pkt = DroneProtocol.build_gl_packet(is_gyro_calib=True)
        self.assertTrue(pkt[6] & GL_CmdBits.GYRO_CALIB)
        self.assertEqual(pkt[6] & 0x80, 0x80, "GL Gyro bit must be 0x80 (bit 7)")
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_gl_return_home_bit(self):
        # In GL mode, return home sets Bit 5 (0x20)
        pkt = DroneProtocol.build_gl_packet(is_fast_return=True)
        self.assertTrue(pkt[6] & GL_CmdBits.RETURN_HOME)
        valid, msg = DroneProtocol.verify_packet(pkt)
        self.assertTrue(valid, msg)

    def test_directional_flips_tc(self):
        # Forward flip: Pitch = 1, Flip bit = 0x08
        fwd_pkt = DroneProtocol.build_tc_packet(pitch=STICK_MIN, is_flip=True)
        self.assertEqual(fwd_pkt[4], STICK_MIN)
        self.assertTrue(fwd_pkt[7] & TC_CmdBits.FLIP)
        valid, msg = DroneProtocol.verify_packet(fwd_pkt)
        self.assertTrue(valid, msg)

        # Backward flip: Pitch = 255, Flip bit = 0x08
        back_pkt = DroneProtocol.build_tc_packet(pitch=STICK_MAX, is_flip=True)
        self.assertEqual(back_pkt[4], STICK_MAX)
        self.assertTrue(back_pkt[7] & TC_CmdBits.FLIP)
        valid, msg = DroneProtocol.verify_packet(back_pkt)
        self.assertTrue(valid, msg)

        # Left flip: Roll = 1, Flip bit = 0x08
        left_pkt = DroneProtocol.build_tc_packet(roll=STICK_MIN, is_flip=True)
        self.assertEqual(left_pkt[3], STICK_MIN)
        self.assertTrue(left_pkt[7] & TC_CmdBits.FLIP)
        valid, msg = DroneProtocol.verify_packet(left_pkt)
        self.assertTrue(valid, msg)

        # Right flip: Roll = 255, Flip bit = 0x08
        right_pkt = DroneProtocol.build_tc_packet(roll=STICK_MAX, is_flip=True)
        self.assertEqual(right_pkt[3], STICK_MAX)
        self.assertTrue(right_pkt[7] & TC_CmdBits.FLIP)
        valid, msg = DroneProtocol.verify_packet(right_pkt)
        self.assertTrue(valid, msg)

    def test_speed_scaling(self):
        # 30% speed mode should limit deflection:
        scaled_high = DroneProtocol.scale_deflection(STICK_CENTER, +127, SpeedMode.LOW)
        self.assertAlmostEqual(scaled_high, 166, delta=1)

        scaled_low = DroneProtocol.scale_deflection(STICK_CENTER, -127, SpeedMode.LOW)
        self.assertAlmostEqual(scaled_low, 90, delta=1)

        # 100% speed mode allows full deflection
        full_high = DroneProtocol.scale_deflection(STICK_CENTER, +127, SpeedMode.HIGH)
        self.assertEqual(full_high, 255)
        full_low = DroneProtocol.scale_deflection(STICK_CENTER, -127, SpeedMode.HIGH)
        self.assertEqual(full_low, 1)

    def test_heartbeat_and_teardown(self):
        hb = DroneProtocol.build_heartbeat()
        self.assertEqual(hb, bytes([0x01, 0x01]))
        td = DroneProtocol.build_teardown()
        self.assertEqual(td, bytes([0x08, 0x01]))


class TestDroneNetworkClient(unittest.TestCase):
    """Test state machine, trims, trajectory, and safety rules of DroneNetworkClient in loopback mode."""

    def setUp(self):
        self.client = DroneNetworkClient(loopback_mode=True)
        self.client.start()

    def tearDown(self):
        self.client.stop()

    def test_initial_state_disarmed(self):
        self.assertEqual(self.client.flight_state, FlightState.DISARMED)
        self.assertFalse(self.client.is_armed)
        self.assertEqual(self.client.throttle, STICK_IDLE_THROTTLE)

    def test_arm_and_disarm(self):
        self.client.arm()
        self.assertTrue(self.client.is_armed)
        self.assertEqual(self.client.flight_state, FlightState.ARMED)
        self.assertEqual(self.client.throttle, STICK_CENTER)

        self.client.disarm()
        self.assertFalse(self.client.is_armed)
        self.assertEqual(self.client.flight_state, FlightState.DISARMED)
        self.assertEqual(self.client.throttle, STICK_IDLE_THROTTLE)

    def test_takeoff_trigger_and_state(self):
        ok = self.client.trigger_takeoff()
        self.assertTrue(ok)
        self.assertEqual(self.client.flight_state, FlightState.TAKING_OFF)
        self.assertTrue(self.client.is_armed)
        self.assertEqual(self.client.throttle, STICK_CENTER)

    def test_emergency_stop_overrides_everything(self):
        self.client.arm()
        self.client.trigger_takeoff()
        self.client.trigger_emergency_stop()
        self.assertEqual(self.client.flight_state, FlightState.EMERGENCY)
        self.assertFalse(self.client.is_armed)
        self.assertEqual(self.client.throttle, 0)

    def test_trims_adjustment_and_reset(self):
        self.client.adjust_trim("pitch", 4)
        self.client.adjust_trim("roll", -6)
        self.client.adjust_trim("yaw", 8)
        self.assertEqual(self.client.trim_pitch, 4)
        self.assertEqual(self.client.trim_roll, -6)
        self.assertEqual(self.client.trim_yaw, 8)

        self.client.reset_trims()
        self.assertEqual(self.client.trim_pitch, 0)
        self.assertEqual(self.client.trim_roll, 0)
        self.assertEqual(self.client.trim_yaw, 0)

    def test_circle_turn_toggle(self):
        self.assertFalse(self.client.is_circle_turn)
        self.client.toggle_circle_turn()
        self.assertTrue(self.client.is_circle_turn)
        self.client.toggle_circle_turn()
        self.assertFalse(self.client.is_circle_turn)

    def test_trajectory_start_and_stop(self):
        # Trajectory requires FLYING state
        self.client.arm()
        self.client.flight_state = FlightState.FLYING
        self.client.start_trajectory(TrajectoryPattern.SQUARE)
        self.assertTrue(self.client.trajectory_active)
        self.assertEqual(self.client.trajectory_pattern, TrajectoryPattern.SQUARE)

        self.client.stop_trajectory()
        self.assertFalse(self.client.trajectory_active)


if __name__ == "__main__":
    unittest.main()
