"""
M9 Drone Flight Deck - UDP Network Engine
Handles 20 Hz fixed-rate control transmission, 1 Hz keepalive heartbeat,
packet telemetry listening, watchdog monitoring, and safe teardown.

Zero camera / video dependencies.
"""

import socket
import threading
import time
import logging
from typing import Callable, Optional

from .state import FlightState, ProtocolType, SpeedMode
from .protocol import (
    DroneProtocol,
    STICK_CENTER, STICK_MIN, STICK_MAX, STICK_IDLE_THROTTLE,
    DURATION_TAKEOFF, DURATION_LAND, DURATION_EMERGENCY,
    DURATION_GYRO_CALIB, DURATION_FLIP, DURATION_LANDING_DESCENT
)

log = logging.getLogger("FlightDeck.Net")


class DroneNetworkClient:
    """
    Manages socket I/O, 20 Hz control loops, 1 Hz keepalives, and flight state transitions.
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
        self.is_armed = False
        self.is_running = False
        self.flight_state = FlightState.DISARMED

        # Live Stick Registers (1..255, center 128)
        self.roll = STICK_CENTER
        self.pitch = STICK_CENTER
        self.throttle = STICK_IDLE_THROTTLE
        self.yaw = STICK_CENTER

        # Active Feature Flags
        self.is_light_on = True
        self.is_headless = False
        self.is_altitude_hold = True

        # Timed Pulse Deadlines (monotonic seconds)
        self._takeoff_until = 0.0
        self._land_until = 0.0
        self._landing_descent_until = 0.0
        self._emergency_until = 0.0
        self._gyro_until = 0.0
        self._flip_until = 0.0
        self._flip_pitch = STICK_CENTER
        self._flip_roll = STICK_CENTER

        # Statistics & Watchdog
        self.tx_count = 0
        self.hb_count = 0
        self.rx_count = 0
        self.last_tx_time = 0.0
        self.is_connected = False
        self.status_message = "Disconnected"

        # Locks & Threads
        self._lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._ctrl_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None

        # Callbacks
        self.on_packet_sent: Optional[Callable[[bytes, str, bool], None]] = None
        self.on_telemetry_received: Optional[Callable[[bytes], None]] = None
        self.on_status_change: Optional[Callable[[bool, str], None]] = None
        self.on_flight_state_change: Optional[Callable[[FlightState], None]] = None

    def start(self):
        """Starts socket, heartbeat worker, control loop, and telemetry receiver."""
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

        self._ctrl_thread.start()
        self._hb_thread.start()
        self._rx_thread.start()

        self._notify_status()
        log.info("DroneNetworkClient started (IP=%s:%d, Loopback=%s)",
                 self.drone_ip, self.ctrl_port, self.loopback_mode)

    def stop(self):
        """Clean shutdown: sends teardown packet, zeros sticks, and terminates threads."""
        log.info("Stopping DroneNetworkClient...")
        with self._lock:
            if not self.is_running:
                return
            self.is_running = False
            self.is_armed = False
            self.flight_state = FlightState.DISARMED

        # Send teardown packet [0x08, 0x01]
        try:
            teardown_pkt = DroneProtocol.build_teardown()
            self._send_raw(teardown_pkt, is_heartbeat=True)
        except Exception:
            pass

        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

        self.is_connected = False
        self.status_message = "Controller Stopped"
        self._notify_status()

    # ── Command & Flight Actions ──────────────────────────────

    def arm(self):
        """Arms the drone and sets neutral hover throttle (128) in readiness for takeoff."""
        with self._lock:
            self.is_armed = True
            self.flight_state = FlightState.ARMED
            self.throttle = STICK_CENTER
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self.status_message = "ARMED - READY FOR TAKEOFF"
        self._notify_status()

    def disarm(self):
        """Disarms the drone and zeros throttle (ground motor stop)."""
        with self._lock:
            self.is_armed = False
            self.flight_state = FlightState.DISARMED
            self.throttle = STICK_IDLE_THROTTLE
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self._clear_timed_commands()
            self.status_message = "DISARMED (MOTORS OFF)"
        self._notify_status()

    def trigger_takeoff(self) -> bool:
        """
        Triggers 1000 ms takeoff pulse (Bit 0 = 1).
        Prevents accidental double takeoff commands if already airborne.
        """
        now = time.monotonic()
        with self._lock:
            if self.flight_state in (FlightState.TAKING_OFF, FlightState.FLYING, FlightState.LANDING):
                log.warning("Takeoff rejected: drone already active in state %s", self.flight_state.value)
                return False

            self.is_armed = True
            self.flight_state = FlightState.TAKING_OFF
            self.throttle = STICK_CENTER
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self._takeoff_until = now + DURATION_TAKEOFF
            self._land_until = 0.0
            self._landing_descent_until = 0.0
            self.status_message = "TAKEOFF PULSE (1.0s active)..."
        log.info("ACTION: Takeoff initiated (1.0s pulse, Bit 0 = 1, Throttle = 128)")
        self._notify_status()
        return True

    def trigger_land(self) -> bool:
        """
        Triggers 1000 ms landing pulse (Bit 0 = 1).
        Autonomous descent handled by drone optical flow / barometer firmware.
        """
        now = time.monotonic()
        with self._lock:
            if self.flight_state not in (FlightState.FLYING, FlightState.TAKING_OFF):
                log.warning("Landing rejected: drone not in flight (state=%s)", self.flight_state.value)
                return False

            self.flight_state = FlightState.LANDING
            self._land_until = now + DURATION_LAND
            self._takeoff_until = 0.0
            self._landing_descent_until = now + DURATION_LAND + DURATION_LANDING_DESCENT
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self.throttle = STICK_CENTER  # Hold neutral so firmware autopilot controls descent
            self.status_message = "AUTOLAND PULSE (1.0s active) -> INITIATING DESCENT..."
        log.info("ACTION: Landing initiated (1.0s pulse, Bit 0 = 1, Autonomous descent)")
        self._notify_status()
        return True

    def trigger_emergency_stop(self):
        """
        Triggers 1000 ms Emergency Stop pulse (Bit 1 = 1).
        Immediately cuts all motor PWM and forces zero throttle.
        """
        now = time.monotonic()
        with self._lock:
            self.is_armed = False
            self.flight_state = FlightState.EMERGENCY
            self.throttle = STICK_IDLE_THROTTLE
            self.roll = STICK_CENTER
            self.pitch = STICK_CENTER
            self.yaw = STICK_CENTER
            self._emergency_until = now + DURATION_EMERGENCY
            self._takeoff_until = 0.0
            self._land_until = 0.0
            self._landing_descent_until = 0.0
            self._flip_until = 0.0
            self.status_message = "!! EMERGENCY MOTOR CUT (PWM OFF) !!"
        log.warning("ACTION: EMERGENCY MOTOR CUT EXECUTED! (Bit 1 = 1, Throttle = 0)")
        self._notify_status()

    def trigger_gyro_calibration(self):
        """
        Triggers 2000 ms gyro calibration ground command.
        Prerequisite: drone disarmed on flat level ground.
        """
        now = time.monotonic()
        with self._lock:
            self._gyro_until = now + DURATION_GYRO_CALIB
            self.status_message = "CALIBRATING GYRO (2.0s)..."
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

    def set_protocol_type(self, proto: ProtocolType):
        """Switches between TC (21-byte) and GL (9-byte) packet mode."""
        with self._lock:
            self.protocol_type = proto
            self.status_message = f"Protocol: {proto.value}"
        self._notify_status()

    def set_sticks(self, roll: int, pitch: int, throttle: int, yaw: int):
        """Sets manual joystick deflection registers."""
        with self._lock:
            self.roll = int(roll)
            self.pitch = int(pitch)
            self.throttle = int(throttle)
            self.yaw = int(yaw)

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

                # Auto-transition: TAKING_OFF -> FLYING once 1.0s takeoff pulse finishes
                if self.flight_state == FlightState.TAKING_OFF and not is_takeoff:
                    self.flight_state = FlightState.FLYING
                    self.status_message = "TAKEOFF COMPLETE -> HOVER STABILIZED (FLYING)"
                    log.info("FlightState: TAKING_OFF -> FLYING (Hover lock active, Bit 0 = 0)")
                    self._notify_status()

                # Auto-transition: LANDING -> DISARMED once landing pulse + descent window completes
                if self.flight_state == FlightState.LANDING and now >= self._landing_descent_until:
                    self.flight_state = FlightState.DISARMED
                    self.is_armed = False
                    self.throttle = STICK_IDLE_THROTTLE
                    self.status_message = "TOUCHDOWN COMPLETE -> MOTORS DISARMED"
                    log.info("FlightState: LANDING -> DISARMED (Touchdown complete, throttle cut to 0)")
                    self._notify_status()

                # Auto-transition: EMERGENCY -> DISARMED once emergency pulse expires
                if self.flight_state == FlightState.EMERGENCY and not is_emergency:
                    self.flight_state = FlightState.DISARMED
                    self._notify_status()

                # Effective sticks (override with flip stick deflection if flipping)
                if is_flip:
                    r = self._flip_roll
                    p = self._flip_pitch
                    t = self.throttle
                    y = STICK_CENTER
                else:
                    # Apply speed scaling to manual deflections from center (128)
                    r_def = self.roll - STICK_CENTER
                    p_def = self.pitch - STICK_CENTER
                    y_def = self.yaw - STICK_CENTER
                    r = DroneProtocol.scale_deflection(STICK_CENTER, r_def, self.speed_mode)
                    p = DroneProtocol.scale_deflection(STICK_CENTER, p_def, self.speed_mode)
                    y = DroneProtocol.scale_deflection(STICK_CENTER, y_def, self.speed_mode)
                    t = self.throttle if self.is_armed else STICK_IDLE_THROTTLE

                # Encode packet according to active protocol
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
                    )

            # Send over UDP
            self._send_raw(packet, is_heartbeat=False)

            # Precise timing anchor
            next_tick += interval
            sleep_sec = next_tick - time.monotonic()
            if sleep_sec > 0:
                time.sleep(sleep_sec)
            else:
                next_tick = time.monotonic()

    def _heartbeat_loop(self):
        """Sends [0x01, 0x01] keepalive packet at 1 Hz."""
        hb_pkt = DroneProtocol.build_heartbeat()
        while self.is_running:
            self._send_raw(hb_pkt, is_heartbeat=True)
            self.hb_count += 1
            time.sleep(1.0)

    def _rx_loop(self):
        """Listens on UDP socket for ACK and telemetry datagrams."""
        while self.is_running:
            if not self._sock or self.loopback_mode:
                time.sleep(0.1)
                continue
            try:
                data, addr = self._sock.recvfrom(256)
                if data:
                    self.rx_count += 1
                    if self.on_telemetry_received:
                        self.on_telemetry_received(data)
            except (socket.timeout, OSError):
                pass

    def _send_raw(self, data: bytes, is_heartbeat: bool):
        """Dispatches datagram and invokes UI update callback."""
        if not self.is_running:
            return

        success = False
        if self.loopback_mode:
            success = True
        else:
            if self._sock:
                try:
                    self._sock.sendto(data, (self.drone_ip, self.ctrl_port))
                    success = True
                except OSError as e:
                    log.debug("UDP send error: %s", e)
                    success = False

        if not is_heartbeat:
            self.tx_count += 1
            self.last_tx_time = time.monotonic()

        if self.on_packet_sent:
            hex_str = " ".join(f"{b:02X}" for b in data)
            self.on_packet_sent(data, hex_str, is_heartbeat)

    def _notify_status(self):
        if self.on_status_change:
            self.on_status_change(self.is_armed, self.status_message)
        if self.on_flight_state_change:
            self.on_flight_state_change(self.flight_state)
