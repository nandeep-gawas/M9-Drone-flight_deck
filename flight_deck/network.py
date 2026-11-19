"""
M9 Drone Flight Deck - UDP Network Engine
Handles 20 Hz fixed-rate control transmission, 1 Hz keepalive heartbeat,
packet telemetry listening, trims, stick modes, trajectory automation,
flight timer, battery estimation, and Linux joystick support.

Zero camera / video dependencies.
"""

import socket
import threading
import time
import logging
import math
import struct
import glob
import os
from typing import Callable, Optional, Tuple, List

from .state import FlightState, ProtocolType, SpeedMode, ControlStickMode, TrajectoryPattern
from .protocol import (
    DroneProtocol,
    STICK_CENTER, STICK_MIN, STICK_MAX, STICK_IDLE_THROTTLE,
    TRIM_MAX_OFFSET, TRIM_STEP,
    DURATION_TAKEOFF, DURATION_LAND, DURATION_EMERGENCY,
    DURATION_GYRO_CALIB, DURATION_FLIP, DURATION_LANDING_DESCENT,
    DURATION_RETURN_HOME
)

log = logging.getLogger("FlightDeck.Net")


class DroneNetworkClient:
    """
    Manages socket I/O, 20 Hz control loops, 1 Hz keepalives,
    trim compensation, trajectory automation, and flight state transitions.
    """

    def __init__(
        self,
        drone_ip: str = "192.168.1.1",
        ctrl_port: int = 7099,
        loopback_mode: bool = False,
    ):
        self.drone_ip = drone_ip
        self.ctrl_port = ctrl_port
        self.loopback_mode = loopback_mode

        self.protocol_type = ProtocolType.TC_21BYTE
        self.speed_mode = SpeedMode.LOW
        self.stick_mode = ControlStickMode.MODE_2_US
        self.is_armed = False
        self.is_running = False
        self.flight_state = FlightState.DISARMED

        # Live Stick Registers (1..255, center 128)
        self.roll = STICK_CENTER
        self.pitch = STICK_CENTER
        self.throttle = STICK_IDLE_THROTTLE
        self.yaw = STICK_CENTER

        # Trims (-24 to +24)
        self.trim_roll = 0
        self.trim_pitch = 0
        self.trim_yaw = 0

        # Active Feature Flags
        self.is_light_on = True
        self.is_headless = False
        self.is_altitude_hold = True
        self.is_circle_turn = False  # Auto-spin 360 rotation

        # Timed Pulse Deadlines (monotonic seconds)
        self._takeoff_until = 0.0
        self._land_until = 0.0
        self._landing_descent_until = 0.0
        self._emergency_until = 0.0
        self._gyro_until = 0.0
        self._flip_until = 0.0
        self._return_home_until = 0.0
        self._flip_pitch = STICK_CENTER
        self._flip_roll = STICK_CENTER

        # Trajectory / Track Flight Automation
        self.trajectory_active = False
        self.trajectory_pattern = TrajectoryPattern.SQUARE
        self.trajectory_start_time = 0.0
        self.trajectory_step = 0
        self.trajectory_progress = 0.0

        # Telemetry & Battery Estimation
        self.flight_seconds = 0.0
        self.flight_start_time = 0.0
        self.battery_percent = 100
        self.BATTERY_LIFETIME_SECS = 480.0  # ~8 minutes typical for 67g toy drone

        # Statistics & Watchdog
        self.tx_count = 0
        self.hb_count = 0
        self.rx_count = 0
        self.last_tx_time = 0.0
        self.is_connected = False
        self.status_message = "Disconnected"
        self.gamepad_connected = False
        self.gamepad_name = "None"

        # Locks & Threads
        self._lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._ctrl_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._joy_thread: Optional[threading.Thread] = None

        # Callbacks
        self.on_packet_sent: Optional[Callable[[bytes, str, bool], None]] = None
        self.on_telemetry_received: Optional[Callable[[bytes], None]] = None
        self.on_status_change: Optional[Callable[[bool, str], None]] = None
        self.on_flight_state_change: Optional[Callable[[FlightState], None]] = None
        self.on_telemetry_tick: Optional[Callable[[float, int], None]] = None  # (seconds, battery%)
        self.on_trajectory_progress: Optional[Callable[[float, str], None]] = None

    def start(self):
        """Starts socket, heartbeat worker, control loop, telemetry receiver, and joystick thread."""
        with self._lock:
            if self.is_running:
                return

            self.is_running = True
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.settimeout(0.5)
                if not self.loopback_mode:
                    self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.is_connected = True
                self.status_message = "Ready (Loopback)" if self.loopback_mode else "Socket Open"
            except Exception as e:
                self.is_connected = False
                self.status_message = f"Socket Init Error: {e}"
                log.error("Socket creation failed: %s", e)

        # Spawn threads
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True, name="CtrlLoop")
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True, name="HBLoop")
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="RxLoop")
        self._joy_thread = threading.Thread(target=self._joystick_loop, daemon=True, name="JoyLoop")

        self._ctrl_thread.start()
        self._hb_thread.start()
        self._rx_thread.start()
        self._joy_thread.start()

        self._notify_status()
        log.info("DroneNetworkClient started (IP=%s:%d, Loopback=%s)",
                 self.drone_ip, self.ctrl_port, self.loopback_mode)

    def stop(self):
        """Safely disarms drone, sends teardown packet, and terminates background threads."""
        with self._lock:
            if not self.is_running:
                return
            self.is_running = False

            # If airborne or armed, force immediate motor cut
            self.is_armed = False
            self.flight_state = FlightState.DISARMED
            self.throttle = STICK_IDLE_THROTTLE
            self._clear_timed_commands()
            self.trajectory_active = False

        # Transmit session teardown datagram [0x08, 0x01]
        try:
            teardown_pkt = DroneProtocol.build_teardown()
            dest_ip = "127.0.0.1" if self.loopback_mode else self.drone_ip
            if self._sock:
                self._sock.sendto(teardown_pkt, (dest_ip, self.ctrl_port))
                log.info("Sent session teardown packet [0x08, 0x01]")
        except Exception as e:
            log.warning("Failed to send teardown packet on exit: %s", e)

        # Close socket
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

        log.info("DroneNetworkClient stopped cleanly.")

    # ── Flight Control Actions ────────────────────────────────

    def arm(self):
        """
        Arms drone on ground. Sets hover center throttle (128) in readiness for takeoff.
        """
        with self._lock:
            if self.flight_state != FlightState.DISARMED:
                return
            self.is_armed = True
            self.flight_state = FlightState.ARMED
            self.throttle = STICK_CENTER
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self.status_message = "ARMED - Neutral Hover (128) Ready for Takeoff"
            if self.flight_start_time == 0.0:
                self.flight_start_time = time.monotonic()
        log.info("ACTION: Motors ARMED. State: ARMED (Throttle=128)")
        self._notify_flight_state()
        self._notify_status()

    def disarm(self):
        """
        Disarms drone. Cuts throttle to 0 and disables all active maneuvers.
        """
        with self._lock:
            self.is_armed = False
            self.flight_state = FlightState.DISARMED
            self.throttle = STICK_IDLE_THROTTLE
            self._clear_timed_commands()
            self.trajectory_active = False
            self.status_message = "DISARMED - Motors Off"
        log.info("ACTION: Motors DISARMED. State: DISARMED (Throttle=0)")
        self._notify_flight_state()
        self._notify_status()

    def trigger_takeoff(self) -> bool:
        """
        Triggers 1000 ms takeoff pulse.
        Enforces neutral hover throttle (128) and transitions to TAKING_OFF.
        """
        now = time.monotonic()
        with self._lock:
            if self.flight_state in (FlightState.TAKING_OFF, FlightState.FLYING, FlightState.LANDING):
                log.warning("Takeoff rejected: Drone already airborne or transitioning!")
                return False

            self.is_armed = True
            self.flight_state = FlightState.TAKING_OFF
            self.throttle = STICK_CENTER  # Strictly neutral hover throttle
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self._takeoff_until = now + DURATION_TAKEOFF
            self.status_message = "TAKING OFF (1.0s Pulse Active)..."
            if self.flight_start_time == 0.0:
                self.flight_start_time = now
        log.info("ACTION: Takeoff triggered (Pulse=1000ms, Bit 0 = 1, Throttle=128)")
        self._notify_flight_state()
        self._notify_status()
        return True

    def trigger_land(self) -> bool:
        """
        Triggers 1000 ms landing pulse and enters autonomous descent.
        """
        now = time.monotonic()
        with self._lock:
            if self.flight_state not in (FlightState.FLYING, FlightState.TAKING_OFF):
                log.warning("Landing rejected: Drone is not in flight!")
                return False

            self.flight_state = FlightState.LANDING
            self.throttle = STICK_CENTER  # Hold neutral hover during command pulse
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self.trajectory_active = False
            self._land_until = now + DURATION_LAND
            self._landing_descent_until = now + DURATION_LAND + DURATION_LANDING_DESCENT
            self.status_message = "AUTONOMOUS LANDING (DESCENT) IN PROGRESS..."
        log.info("ACTION: Land triggered (Pulse=1000ms, Bit 0 = 1, Descent=4.0s)")
        self._notify_flight_state()
        self._notify_status()
        return True

    def trigger_emergency_stop(self):
        """
        EMERGENCY SHUTDOWN: Immediately cuts PWM motor power mid-air.
        Asserts Bit 1 (0x02) for 1000 ms and forces throttle strictly to 0.
        """
        now = time.monotonic()
        with self._lock:
            self.is_armed = False
            self.flight_state = FlightState.EMERGENCY
            self.throttle = STICK_IDLE_THROTTLE  # Instant cut to 0
            self._emergency_until = now + DURATION_EMERGENCY
            self._takeoff_until = 0.0
            self._land_until = 0.0
            self._landing_descent_until = 0.0
            self._flip_until = 0.0
            self._return_home_until = 0.0
            self.trajectory_active = False
            self.status_message = "!! EMERGENCY MOTOR CUT (PWM OFF) !!"
        log.warning("ACTION: EMERGENCY MOTOR CUT EXECUTED! (Bit 1 = 1, Throttle = 0)")
        self._notify_flight_state()
        self._notify_status()

    def trigger_gyro_calibration(self):
        """
        Triggers 2000 ms gyro calibration ground command.
        Prerequisite: drone disarmed on flat level ground.
        """
        now = time.monotonic()
        with self._lock:
            self._gyro_until = now + DURATION_GYRO_CALIB
            self.status_message = "CALIBRATING GYRO (2.0s)... Keep drone still."
        log.info("ACTION: Gyro calibration triggered")
        self._notify_status()

    def trigger_flip(self, direction: str):
        """
        Triggers directional 360-degree flip for 600 ms.
        direction: 'forward', 'backward', 'left', 'right'
        """
        now = time.monotonic()
        with self._lock:
            if not self.is_armed:
                self.status_message = "Flip rejected: Drone not armed!"
                self._notify_status()
                return

            self._flip_until = now + DURATION_FLIP
            self._flip_pitch = STICK_CENTER
            self._flip_roll = STICK_CENTER

            dir_lower = direction.lower()
            if dir_lower == "forward":
                self._flip_pitch = STICK_MIN  # 0x01
            elif dir_lower == "backward":
                self._flip_pitch = STICK_MAX  # 0xFF
            elif dir_lower == "left":
                self._flip_roll = STICK_MIN   # 0x01
            elif dir_lower == "right":
                self._flip_roll = STICK_MAX   # 0xFF
            else:
                self._flip_until = 0.0
                return

            self.status_message = f"360 FLIP ({direction.upper()}) ACTIVE (600ms)"
        log.info("ACTION: Flip %s triggered", direction)
        self._notify_status()

    def trigger_return_home(self):
        """
        One-Key Return / Fast Return (`isFastReturn`).
        Commands return-to-pilot orientation for 3.0 seconds.
        """
        now = time.monotonic()
        with self._lock:
            if not self.is_armed:
                self.status_message = "Return rejected: Drone not armed!"
                self._notify_status()
                return
            self._return_home_until = now + DURATION_RETURN_HOME
            self.status_message = "ONE-KEY RETURN TO HOME ACTIVE (3.0s)"
        log.info("ACTION: One-Key Return triggered")
        self._notify_status()

    def toggle_circle_turn(self) -> bool:
        """
        Toggles 360-degree continuous circle rotation (auto-spin).
        """
        with self._lock:
            self.is_circle_turn = not self.is_circle_turn
            state = "ON" if self.is_circle_turn else "OFF"
            self.status_message = f"Circle Auto-Spin: {state}"
        log.info("ACTION: Circle turn toggled to %s", state)
        self._notify_status()
        return self.is_circle_turn

    def toggle_headless(self) -> bool:
        """Toggles headless mode latching state."""
        with self._lock:
            self.is_headless = not self.is_headless
            state = "ON" if self.is_headless else "OFF"
            self.status_message = f"Headless Mode: {state}"
        self._notify_status()
        return self.is_headless

    def toggle_lights(self) -> bool:
        """Toggles drone LED lights."""
        with self._lock:
            self.is_light_on = not self.is_light_on
            state = "ON" if self.is_light_on else "OFF"
            self.status_message = f"Drone Lights: {state}"
        self._notify_status()
        return self.is_light_on

    def set_speed_mode(self, mode: SpeedMode):
        """Sets client-side speed scaling mode (Low=30%, Med=60%, High=100%)."""
        with self._lock:
            self.speed_mode = mode
            names = {SpeedMode.LOW: "30%", SpeedMode.MEDIUM: "60%", SpeedMode.HIGH: "100%"}
            self.status_message = f"Speed Mode: {names.get(mode, 'Unknown')}"
        self._notify_status()

    def set_stick_mode(self, mode: ControlStickMode):
        """Sets joystick mode (Mode 2 = US / Default, Mode 1 = European / Right-Hand Throttle)."""
        with self._lock:
            self.stick_mode = mode
            self.status_message = f"Stick Layout: {mode.value}"
        self._notify_status()

    def set_protocol_type(self, proto: ProtocolType):
        """Switches between TC (21-byte) and GL (9-byte) packet mode."""
        with self._lock:
            self.protocol_type = proto
            self.status_message = f"Protocol: {proto.value}"
        self._notify_status()

    # ── Trim System ───────────────────────────────────────────

    def adjust_trim(self, axis: str, delta: int):
        """
        Adjusts fine calibration trim offset.
        axis: 'pitch', 'roll', 'yaw'
        delta: signed step (+TRIM_STEP or -TRIM_STEP)
        """
        with self._lock:
            if axis == "pitch":
                self.trim_pitch = max(-TRIM_MAX_OFFSET, min(TRIM_MAX_OFFSET, self.trim_pitch + delta))
            elif axis == "roll":
                self.trim_roll = max(-TRIM_MAX_OFFSET, min(TRIM_MAX_OFFSET, self.trim_roll + delta))
            elif axis == "yaw":
                self.trim_yaw = max(-TRIM_MAX_OFFSET, min(TRIM_MAX_OFFSET, self.trim_yaw + delta))
            self.status_message = f"Trims: R={self.trim_roll:+d} P={self.trim_pitch:+d} Y={self.trim_yaw:+d}"
        self._notify_status()

    def reset_trims(self):
        """Zeros all physical drift trims."""
        with self._lock:
            self.trim_roll = 0
            self.trim_pitch = 0
            self.trim_yaw = 0
            self.status_message = "All trims reset to 0 (Neutral Center)"
        self._notify_status()

    # ── Trajectory / Track Flight Automation ──────────────────

    def start_trajectory(self, pattern: TrajectoryPattern):
        """
        Initiates automated track flight maneuver (Square, Circle, Fig-8, Triangle).
        Prerequisite: drone is in FLYING state.
        """
        with self._lock:
            if self.flight_state != FlightState.FLYING:
                self.status_message = "Trajectory rejected: Drone must be in stable hover!"
                self._notify_status()
                return
            self.trajectory_active = True
            self.trajectory_pattern = pattern
            self.trajectory_start_time = time.monotonic()
            self.trajectory_step = 0
            self.status_message = f"EXECUTING TRAJECTORY: {pattern.value}"
        log.info("ACTION: Trajectory %s started", pattern.value)
        self._notify_status()

    def stop_trajectory(self):
        """Cancels automated flight and returns to manual hover."""
        with self._lock:
            self.trajectory_active = False
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self.status_message = "Trajectory aborted -> Stable Hover"
        log.info("ACTION: Trajectory aborted")
        self._notify_status()

    # ── Stick Control ─────────────────────────────────────────

    def set_sticks(self, roll: int, pitch: int, throttle: int, yaw: int):
        """Sets manual joystick deflection registers."""
        with self._lock:
            if not self.trajectory_active:
                self.roll = int(roll)
                self.pitch = int(pitch)
                self.yaw = int(yaw)
            self.throttle = int(throttle)

    def center_sticks(self):
        """Centers roll, pitch, and yaw to 128 (neutral hover)."""
        with self._lock:
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            if self.is_armed:
                self.throttle = STICK_CENTER

    # ── Internal Loops ────────────────────────────────────────

    def _clear_timed_commands(self):
        self._takeoff_until = 0.0
        self._land_until = 0.0
        self._landing_descent_until = 0.0
        self._emergency_until = 0.0
        self._gyro_until = 0.0
        self._flip_until = 0.0
        self._return_home_until = 0.0

    def _update_trajectory(self, now: float) -> Tuple[int, int]:
        """
        Computes Pitch and Roll deflections for automated trajectory patterns.
        Returns (roll, pitch).
        """
        elapsed = now - self.trajectory_start_time
        pattern = self.trajectory_pattern
        r, p = STICK_CENTER, STICK_CENTER

        if pattern == TrajectoryPattern.SQUARE:
            # 8-second square: 2s forward, 2s right, 2s back, 2s left
            cycle = elapsed % 8.0
            self.trajectory_progress = min(1.0, elapsed / 8.0)
            if cycle < 2.0:
                p = STICK_CENTER - 35  # Forward
            elif cycle < 4.0:
                r = STICK_CENTER + 35  # Right
            elif cycle < 6.0:
                p = STICK_CENTER + 35  # Backward
            else:
                r = STICK_CENTER - 35  # Left
            if elapsed >= 8.0:
                self.trajectory_active = False
                self.status_message = "Square trajectory complete"

        elif pattern == TrajectoryPattern.CIRCLE:
            # 6-second circle orbit using sin/cos
            duration = 6.0
            self.trajectory_progress = min(1.0, elapsed / duration)
            angle = (elapsed / duration) * 2.0 * math.pi
            r = STICK_CENTER + int(35 * math.sin(angle))
            p = STICK_CENTER - int(35 * math.cos(angle))
            if elapsed >= duration:
                self.trajectory_active = False
                self.status_message = "Circle orbit complete"

        elif pattern == TrajectoryPattern.FIGURE_8:
            # 8-second figure-8 (Lissajous curve)
            duration = 8.0
            self.trajectory_progress = min(1.0, elapsed / duration)
            t = (elapsed / duration) * 2.0 * math.pi
            r = STICK_CENTER + int(35 * math.sin(t))
            p = STICK_CENTER - int(35 * math.sin(2.0 * t))
            if elapsed >= duration:
                self.trajectory_active = False
                self.status_message = "Figure-8 trajectory complete"

        elif pattern == TrajectoryPattern.TRIANGLE:
            # 6-second triangle: 2s forward, 2s back-right, 2s back-left
            cycle = elapsed % 6.0
            self.trajectory_progress = min(1.0, elapsed / 6.0)
            if cycle < 2.0:
                p = STICK_CENTER - 35
            elif cycle < 4.0:
                p = STICK_CENTER + 20
                r = STICK_CENTER + 35
            else:
                p = STICK_CENTER + 20
                r = STICK_CENTER - 35
            if elapsed >= 6.0:
                self.trajectory_active = False
                self.status_message = "Triangle trajectory complete"

        if self.on_trajectory_progress:
            self.on_trajectory_progress(self.trajectory_progress, pattern.value)

        return r, p

    def _control_loop(self):
        """Fixed 20 Hz transmission loop (50 ms cycle period)."""
        interval = 0.050  # 50 ms = 20 Hz
        next_tick = time.monotonic()

        while self.is_running:
            now = time.monotonic()

            with self._lock:
                # Check timed command expirations
                is_takeoff = now < self._takeoff_until
                is_land = now < self._land_until
                is_emergency = now < self._emergency_until
                is_gyro = now < self._gyro_until
                is_flip = now < self._flip_until
                is_fast_return = now < self._return_home_until

                # Auto-transition: TAKING_OFF -> FLYING once 1.0s takeoff pulse finishes
                if self.flight_state == FlightState.TAKING_OFF and not is_takeoff:
                    self.flight_state = FlightState.FLYING
                    self.status_message = "TAKEOFF COMPLETE -> HOVER STABILIZED (FLYING)"
                    log.info("FlightState: TAKING_OFF -> FLYING (Hover lock active, Bit 0 = 0)")
                    self._notify_flight_state()
                    self._notify_status()

                # Auto-transition: LANDING -> DISARMED once landing pulse + descent window completes
                if self.flight_state == FlightState.LANDING and now >= self._landing_descent_until:
                    self.flight_state = FlightState.DISARMED
                    self.is_armed = False
                    self.throttle = STICK_IDLE_THROTTLE
                    self.status_message = "TOUCHDOWN COMPLETE -> MOTORS DISARMED"
                    log.info("FlightState: LANDING -> DISARMED (Touchdown complete, throttle cut to 0)")
                    self._notify_flight_state()
                    self._notify_status()

                # Update flight duration timer and battery estimation
                if self.is_armed and self.flight_start_time > 0:
                    self.flight_seconds = now - self.flight_start_time
                    # Toy drone discharge rate: 100% down to 0% over ~8 minutes
                    discharged = (self.flight_seconds / self.BATTERY_LIFETIME_SECS) * 100.0
                    self.battery_percent = max(0, min(100, int(100.0 - discharged)))
                    if self.on_telemetry_tick:
                        self.on_telemetry_tick(self.flight_seconds, self.battery_percent)

                # Determine active stick values
                r, p, t, y = self.roll, self.pitch, self.throttle, self.yaw

                # If Trajectory is active, override pitch and roll
                if self.trajectory_active and self.flight_state == FlightState.FLYING:
                    r, p = self._update_trajectory(now)

                # If Flip is active, apply full directional stick deflection
                if is_flip:
                    p = self._flip_pitch
                    r = self._flip_roll

                # If Auto-spin / Circle Turn is active, apply continuous yaw
                if self.is_circle_turn and self.flight_state == FlightState.FLYING:
                    y = STICK_CENTER + 45  # Controlled yaw rotation

                # In Emergency, throttle is forced strictly to 0
                if is_emergency:
                    t = 0

                # Build wire packet
                if self.protocol_type == ProtocolType.TC_21BYTE:
                    packet = DroneProtocol.build_tc_packet(
                        roll=r,
                        pitch=p,
                        throttle=t,
                        yaw=y,
                        is_taking_off=is_takeoff,
                        is_landing=is_land,
                        is_emergency=is_emergency,
                        is_gyro_calib=is_gyro,
                        is_flip=is_flip,
                        is_light_on=self.is_light_on,
                        is_headless=self.is_headless,
                        is_altitude_hold=self.is_altitude_hold,
                        is_fast_return=is_fast_return,
                        trim_roll=self.trim_roll,
                        trim_pitch=self.trim_pitch,
                        trim_yaw=self.trim_yaw,
                    )
                else:
                    packet = DroneProtocol.build_gl_packet(
                        roll=r,
                        pitch=p,
                        throttle=t,
                        yaw=y,
                        is_taking_off=is_takeoff,
                        is_landing=is_land,
                        is_emergency=is_emergency,
                        is_gyro_calib=is_gyro,
                        is_flip=is_flip,
                        is_armed=self.is_armed,
                        is_light_on=self.is_light_on,
                        is_headless=self.is_headless,
                        is_fast_return=is_fast_return,
                        trim_roll=self.trim_roll,
                        trim_pitch=self.trim_pitch,
                        trim_yaw=self.trim_yaw,
                    )

            # Transmit packet over UDP
            dest_ip = "127.0.0.1" if self.loopback_mode else self.drone_ip
            try:
                if self._sock:
                    self._sock.sendto(packet, (dest_ip, self.ctrl_port))
                    self.tx_count += 1
                    self.last_tx_time = now
                    if self.on_packet_sent:
                        hex_str = " ".join(f"{b:02X}" for b in packet)
                        self.on_packet_sent(packet, hex_str, False)
            except Exception as e:
                log.warning("UDP send error: %s", e)

            # Maintain strict 20 Hz timing
            next_tick += interval
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    def _heartbeat_loop(self):
        """1 Hz keepalive worker sending [0x01, 0x01] datagram."""
        while self.is_running:
            time.sleep(1.0)
            if not self.is_running:
                break

            hb_packet = DroneProtocol.build_heartbeat()
            dest_ip = "127.0.0.1" if self.loopback_mode else self.drone_ip
            try:
                if self._sock:
                    self._sock.sendto(hb_packet, (dest_ip, self.ctrl_port))
                    self.hb_count += 1
                    if self.on_packet_sent:
                        hex_str = " ".join(f"{b:02X}" for b in hb_packet)
                        self.on_packet_sent(hb_packet, hex_str, True)
            except Exception as e:
                log.warning("Heartbeat send failed: %s", e)

    def _rx_loop(self):
        """Listens for returning telemetry or ACK packets on UDP socket."""
        while self.is_running:
            try:
                if not self._sock:
                    time.sleep(0.1)
                    continue

                data, addr = self._sock.recvfrom(2048)
                if data:
                    self.rx_count += 1
                    if self.on_telemetry_received:
                        self.on_telemetry_received(data)
            except socket.timeout:
                continue
            except Exception:
                if self.is_running:
                    time.sleep(0.1)

    def _joystick_loop(self):
        """
        Polls standard Linux joystick interface (/dev/input/js0..js3).
        Provides plug-and-play support for physical gamepads (Xbox, PlayStation, etc.).
        """
        js_file = None
        current_js_dev = None

        while self.is_running:
            if js_file is None:
                # Probe /dev/input/js*
                devs = sorted(glob.glob("/dev/input/js*"))
                if devs:
                    try:
                        js_file = open(devs[0], "rb", buffering=0)
                        current_js_dev = devs[0]
                        self.gamepad_connected = True
                        self.gamepad_name = os.path.basename(current_js_dev)
                        self.status_message = f"Gamepad Connected: {self.gamepad_name}"
                        self._notify_status()
                        log.info("Gamepad found: %s", current_js_dev)
                    except Exception:
                        js_file = None
                        time.sleep(1.0)
                        continue
                else:
                    self.gamepad_connected = False
                    self.gamepad_name = "None"
                    time.sleep(1.0)
                    continue

            # Read 8-byte Linux js_event: (time: u32, value: s16, type: u8, number: u8)
            try:
                event_data = js_file.read(8)
                if not event_data or len(event_data) < 8:
                    time.sleep(0.01)
                    continue

                _, val, evt_type, evt_num = struct.unpack("IhBB", event_data)

                # Axis event (type & 0x02)
                if evt_type & 0x02:
                    norm = val / 32767.0  # -1.0 to +1.0
                    if evt_num == 0:    # Left Stick X: Yaw
                        self.yaw = STICK_CENTER + int(norm * 127)
                    elif evt_num == 1:  # Left Stick Y: Throttle (Inverted: up is negative)
                        if self.is_armed:
                            self.throttle = STICK_CENTER - int(norm * 127)
                    elif evt_num == 2 or evt_num == 3:  # Right Stick X: Roll
                        self.roll = STICK_CENTER + int(norm * 127)
                    elif evt_num == 3 or evt_num == 4:  # Right Stick Y: Pitch (Inverted)
                        self.pitch = STICK_CENTER + int(norm * 127)

                # Button event (type & 0x01)
                elif evt_type & 0x01 and val == 1:  # Button Down
                    if evt_num == 0:    # A / Cross: Takeoff
                        self.trigger_takeoff()
                    elif evt_num == 1:  # B / Circle: Land
                        self.trigger_land()
                    elif evt_num == 2:  # X / Square: Flip Front
                        self.trigger_flip("forward")
                    elif evt_num == 3:  # Y / Triangle: Emergency Stop
                        self.trigger_emergency_stop()
                    elif evt_num == 4:  # LB: Headless toggle
                        self.toggle_headless()
                    elif evt_num == 5:  # RB: Altitude hold toggle
                        self.is_altitude_hold = not self.is_altitude_hold

            except Exception as e:
                # Device disconnected
                if js_file:
                    try:
                        js_file.close()
                    except Exception:
                        pass
                js_file = None
                self.gamepad_connected = False
                self.gamepad_name = "None"
                time.sleep(1.0)

    # ── Notification Helpers ──────────────────────────────────

    def _notify_status(self):
        if self.on_status_change:
            self.on_status_change(self.is_armed, self.status_message)

    def _notify_flight_state(self):
        if self.on_flight_state_change:
            self.on_flight_state_change(self.flight_state)
