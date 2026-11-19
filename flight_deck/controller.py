"""
M9 Drone Flight Deck - Full Cockpit Desktop GUI Controller
PyQt5 Cockpit Window for KY UFO / FLOW-UFO Drone Control on Linux.

Features:
- Complete flight action control (Takeoff, Land, Emergency, Gyro, Flips)
- One-Key Return to Home & 360° Auto-Spin / Orbit
- Autonomous Trajectory / Track Flight (Square, Circle, Fig-8, Triangle)
- Digital Trim System (Pitch, Roll, Yaw fine tuning & zero-reset)
- Dual Stick Modes: Mode 2 (US standard) & Mode 1 (European)
- Live Telemetry: Flight Duration Timer, Battery Gauge & Voltage Status
- Linux Gamepad / USB Joystick auto-detection
- 2D Crosshair Virtual Stick Gauges
- 20 Hz Live Hex Packet Inspector & Logging Console

Zero camera / video / RTSP dependencies.
"""

import sys
import os
import time
import logging
from typing import Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette, QKeyEvent
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QProgressBar, QTextEdit,
    QGroupBox, QRadioButton, QButtonGroup, QCheckBox, QSlider,
    QFrame, QMessageBox, QComboBox
)

from .state import FlightState, ProtocolType, SpeedMode, ControlStickMode, TrajectoryPattern
from .protocol import (
    DroneProtocol,
    STICK_CENTER, STICK_MIN, STICK_MAX, STICK_IDLE_THROTTLE,
    TRIM_STEP, TRIM_MAX_OFFSET,
    DURATION_TAKEOFF, DURATION_LAND, DURATION_EMERGENCY,
    DURATION_GYRO_CALIB, DURATION_FLIP
)
from .network import DroneNetworkClient

log = logging.getLogger("M9-FlightDeck.GUI")


class NetSignals(QObject):
    """Bridge for cross-thread Qt signals from the background network worker."""
    packet_sent = pyqtSignal(bytes, str, bool)
    status_changed = pyqtSignal(bool, str)
    telemetry_received = pyqtSignal(bytes)
    flight_state_changed = pyqtSignal(object)
    telemetry_tick = pyqtSignal(float, int)
    trajectory_progress = pyqtSignal(float, str)


class DroneDesktopWindow(QMainWindow):
    """
    Main application window for the M9 Drone Flight Deck.
    """

    def __init__(self, drone_ip: str = "192.168.1.1", ctrl_port: int = 7099, loopback: bool = False):
        super().__init__()
        self.setWindowTitle("M9 Drone Flight Deck — KY UFO / FLOW-UFO Pro Cockpit")
        self.resize(1200, 860)
        self.setMinimumSize(1020, 740)

        # Network layer
        self.net_signals = NetSignals()
        self.client = DroneNetworkClient(drone_ip=drone_ip, ctrl_port=ctrl_port, loopback_mode=loopback)

        # Keyboard press state tracking for smooth spring-back sticks
        self.pressed_keys = set()
        self.stick_step = 25  # Deflection step per key tick

        # Propeller-removed bench testing safety state
        self.propeller_removed_mode = True

        # Connect network signals
        self.client.on_packet_sent = lambda pkt, hx, hb: self.net_signals.packet_sent.emit(pkt, hx, hb)
        self.client.on_status_change = lambda armed, msg: self.net_signals.status_changed.emit(armed, msg)
        self.client.on_telemetry_received = lambda data: self.net_signals.telemetry_received.emit(data)
        self.client.on_flight_state_change = lambda state: self.net_signals.flight_state_changed.emit(state)
        self.client.on_telemetry_tick = lambda secs, bat: self.net_signals.telemetry_tick.emit(secs, bat)
        self.client.on_trajectory_progress = lambda prog, pat: self.net_signals.trajectory_progress.emit(prog, pat)

        self.net_signals.packet_sent.connect(self._on_packet_sent)
        self.net_signals.status_changed.connect(self._on_status_changed)
        self.net_signals.telemetry_received.connect(self._on_telemetry_received)
        self.net_signals.flight_state_changed.connect(self._on_flight_state_changed)
        self.net_signals.telemetry_tick.connect(self._on_telemetry_tick)
        self.net_signals.trajectory_progress.connect(self._on_trajectory_progress)

        # Build UI
        self._setup_dark_theme()
        self._init_ui()

        # Keyboard input polling timer (50 Hz / 20 ms for responsive stick input)
        self.key_timer = QTimer(self)
        self.key_timer.timeout.connect(self._poll_keyboard_sticks)
        self.key_timer.start(20)

        # Start network engine
        self.client.start()
        self._log_info("Flight Deck initialized. Safe Startup: Drone DISARMED.")
        self._log_info(f"Target: {self.client.drone_ip}:{self.client.ctrl_port} | Loopback Mode: {loopback}")

    # ── UI Construction ───────────────────────────────────────

    def _init_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(12, 10, 12, 10)
        main_layout.setSpacing(8)

        # 1. Top Safety, Battery & Telemetry Bar
        top_bar = self._create_top_safety_bar()
        main_layout.addWidget(top_bar)

        # 2. Middle Section (Split: Controls Left, Trims & Trajectory Center, Gauges Right)
        mid_layout = QHBoxLayout()
        mid_layout.setSpacing(10)

        left_panel = self._create_flight_control_panel()
        mid_layout.addWidget(left_panel, stretch=3)

        center_panel = self._create_trims_and_trajectory_panel()
        mid_layout.addWidget(center_panel, stretch=2)

        right_panel = self._create_gauges_and_dpad_panel()
        mid_layout.addWidget(right_panel, stretch=2)

        main_layout.addLayout(mid_layout, stretch=5)

        # 3. Bottom Section: Live Hex Packet Inspector & Logging Console
        bottom_panel = self._create_inspector_and_log_panel()
        main_layout.addWidget(bottom_panel, stretch=3)

    def _create_top_safety_bar(self) -> QWidget:
        container = QFrame()
        container.setFrameShape(QFrame.StyledPanel)
        container.setStyleSheet("""
            QFrame {
                background-color: #1e222b;
                border: 1px solid #2d3342;
                border-radius: 6px;
            }
        """)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(12)

        # Connection Indicator
        self.lbl_conn_status = QLabel("● NETWORK READY")
        self.lbl_conn_status.setStyleSheet("color: #00e676; font-weight: bold; font-size: 12px;")
        layout.addWidget(self.lbl_conn_status)

        # Flight Duration Timer
        self.lbl_flight_timer = QLabel("⏱ FLIGHT: 00:00")
        self.lbl_flight_timer.setStyleSheet("color: #4fc3f7; font-weight: bold; font-family: monospace; font-size: 13px;")
        layout.addWidget(self.lbl_flight_timer)

        # Battery Indicator
        layout.addWidget(QLabel("BAT:"))
        self.bar_battery = QProgressBar()
        self.bar_battery.setRange(0, 100)
        self.bar_battery.setValue(100)
        self.bar_battery.setFixedWidth(80)
        self.bar_battery.setFixedHeight(16)
        self.bar_battery.setTextVisible(True)
        self.bar_battery.setFormat("%p%")
        self.bar_battery.setStyleSheet("""
            QProgressBar {
                border: 1px solid #455a64;
                border-radius: 3px;
                text-align: center;
                color: white;
                font-size: 10px;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #00e676;
            }
        """)
        layout.addWidget(self.bar_battery)

        # Gamepad Indicator
        self.lbl_gamepad = QLabel("🎮 GAMEPAD: NONE")
        self.lbl_gamepad.setStyleSheet("color: #78909c; font-size: 11px; font-weight: bold;")
        layout.addWidget(self.lbl_gamepad)

        # Armed Status Badge
        self.lbl_armed_status = QLabel("[ DISARMED ]")
        self.lbl_armed_status.setStyleSheet("""
            background-color: #37474f;
            color: #eceff1;
            padding: 4px 8px;
            border-radius: 4px;
            font-weight: bold;
            font-size: 12px;
        """)
        layout.addWidget(self.lbl_armed_status)

        layout.addStretch()

        # Bench Test Propeller-Removed Safety Toggle
        self.chk_prop_test = QCheckBox("BENCH TEST MODE")
        self.chk_prop_test.setChecked(True)
        self.chk_prop_test.setStyleSheet("color: #ffb74d; font-weight: bold; font-size: 11px;")
        self.chk_prop_test.toggled.connect(self._on_prop_mode_toggled)
        layout.addWidget(self.chk_prop_test)

        # Arm / Disarm Master Button
        self.btn_arm_toggle = QPushButton("ARM (C)")
        self.btn_arm_toggle.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                color: white;
                font-weight: bold;
                padding: 6px 12px;
                border-radius: 4px;
            }
            QPushButton:hover { background-color: #388e3c; }
        """)
        self.btn_arm_toggle.clicked.connect(self._toggle_arm)
        layout.addWidget(self.btn_arm_toggle)

        return container

    def _create_flight_control_panel(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Primary Takeoff / Landing / Emergency Stop Row
        btn_grid = QGridLayout()
        btn_grid.setSpacing(8)

        self.btn_takeoff = QPushButton("TAKEOFF (T)")
        self.btn_takeoff.setStyleSheet("""
            QPushButton {
                background-color: #0288d1;
                color: white;
                font-weight: bold;
                font-size: 13px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #039be5; }
        """)
        self.btn_takeoff.clicked.connect(self._on_takeoff_clicked)
        btn_grid.addWidget(self.btn_takeoff, 0, 0)

        self.btn_land = QPushButton("LAND (L)")
        self.btn_land.setEnabled(False)
        self.btn_land.setStyleSheet("""
            QPushButton {
                background-color: #00897b;
                color: white;
                font-weight: bold;
                font-size: 13px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #009688; }
            QPushButton:disabled { background-color: #263238; color: #546e7a; }
        """)
        self.btn_land.clicked.connect(self._on_land_clicked)
        btn_grid.addWidget(self.btn_land, 0, 1)

        # BIG RED EMERGENCY STOP BUTTON
        self.btn_emergency = QPushButton("!! EMERGENCY STOP (E) !!\n(CUT MOTOR POWER)")
        self.btn_emergency.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                font-weight: bold;
                font-size: 12px;
                padding: 8px;
                border-radius: 5px;
                border: 2px solid #ff5252;
            }
            QPushButton:hover { background-color: #f44336; }
            QPushButton:pressed { background-color: #b71c1c; }
        """)
        self.btn_emergency.clicked.connect(self._on_emergency_clicked)
        btn_grid.addWidget(self.btn_emergency, 0, 2)

        layout.addLayout(btn_grid)

        # Secondary Actions: Gyro Calib, Headless, Lights, Altitude Hold, Return Home, Orbit
        aux_group = QGroupBox("Flight Features & System Controls")
        aux_layout = QGridLayout(aux_group)
        aux_layout.setSpacing(6)

        self.btn_gyro = QPushButton("Calibrate Gyro (G)")
        self.btn_gyro.setStyleSheet("padding: 7px; font-weight: bold;")
        self.btn_gyro.clicked.connect(self._on_gyro_clicked)
        aux_layout.addWidget(self.btn_gyro, 0, 0)

        self.btn_headless = QPushButton("Headless: OFF (H)")
        self.btn_headless.setStyleSheet("padding: 7px; font-weight: bold;")
        self.btn_headless.clicked.connect(self._on_headless_clicked)
        aux_layout.addWidget(self.btn_headless, 0, 1)

        self.btn_lights = QPushButton("LEDs: ON")
        self.btn_lights.setStyleSheet("padding: 7px; font-weight: bold; color: #ffeb3b;")
        self.btn_lights.clicked.connect(self._on_lights_clicked)
        aux_layout.addWidget(self.btn_lights, 0, 2)

        self.btn_alt_hold = QPushButton("Alt Hold: ON")
        self.btn_alt_hold.setStyleSheet("padding: 7px; font-weight: bold; color: #4fc3f7;")
        self.btn_alt_hold.clicked.connect(self._on_alt_hold_clicked)
        aux_layout.addWidget(self.btn_alt_hold, 1, 0)

        self.btn_return = QPushButton("Return Home (R)")
        self.btn_return.setStyleSheet("background-color: #ef6c00; color: white; padding: 7px; font-weight: bold;")
        self.btn_return.clicked.connect(self._on_return_home_clicked)
        aux_layout.addWidget(self.btn_return, 1, 1)

        self.btn_circle_turn = QPushButton("Auto-Spin: OFF (O)")
        self.btn_circle_turn.setStyleSheet("padding: 7px; font-weight: bold;")
        self.btn_circle_turn.clicked.connect(self._on_circle_turn_clicked)
        aux_layout.addWidget(self.btn_circle_turn, 1, 2)

        layout.addWidget(aux_group)

        # Stick Layout Mode (Mode 2 vs Mode 1)
        mode_group = QGroupBox("Joystick Layout Mode")
        mode_layout = QHBoxLayout(mode_group)
        self.rb_mode2 = QRadioButton("Mode 2 (US: Left=Thr/Yaw, Right=Pitch/Roll)")
        self.rb_mode1 = QRadioButton("Mode 1 (EU: Left=Pitch/Yaw, Right=Thr/Roll)")
        self.rb_mode2.setChecked(True)
        self.rb_mode2.toggled.connect(lambda chk: self._on_stick_mode_changed(ControlStickMode.MODE_2_US if chk else ControlStickMode.MODE_1_EU))
        mode_layout.addWidget(self.rb_mode2)
        mode_layout.addWidget(self.rb_mode1)
        layout.addWidget(mode_group)

        # Speed Mode Selector
        spd_group = QGroupBox("Speed Mode (Deflection Limit)")
        spd_layout = QHBoxLayout(spd_group)
        self.spd_btn_group = QButtonGroup(self)
        self.rb_spd_low = QRadioButton("30% Low (Beginner)")
        self.rb_spd_med = QRadioButton("60% Medium (Cruising)")
        self.rb_spd_high = QRadioButton("100% High (Sport)")
        self.rb_spd_low.setChecked(True)

        self.spd_btn_group.addButton(self.rb_spd_low, SpeedMode.LOW)
        self.spd_btn_group.addButton(self.rb_spd_med, SpeedMode.MEDIUM)
        self.spd_btn_group.addButton(self.rb_spd_high, SpeedMode.HIGH)
        self.spd_btn_group.buttonClicked[int].connect(self._on_speed_changed)

        spd_layout.addWidget(self.rb_spd_low)
        spd_layout.addWidget(self.rb_spd_med)
        spd_layout.addWidget(self.rb_spd_high)
        layout.addWidget(spd_group)

        # Hardware Protocol Architecture
        proto_group = QGroupBox("Hardware Protocol Architecture")
        proto_layout = QHBoxLayout(proto_group)
        self.proto_btn_group = QButtonGroup(self)
        self.rb_proto_tc = QRadioButton("21-Byte TC (FLOW-UFO / BL60X)")
        self.rb_proto_gl = QRadioButton("9-Byte GL (Legacy KY UFO / E58)")
        self.rb_proto_tc.setChecked(True)

        self.proto_btn_group.addButton(self.rb_proto_tc, 1)
        self.proto_btn_group.addButton(self.rb_proto_gl, 2)
        self.proto_btn_group.buttonClicked[int].connect(self._on_protocol_changed)

        proto_layout.addWidget(self.rb_proto_tc)
        proto_layout.addWidget(self.rb_proto_gl)
        layout.addWidget(proto_group)

        return container

    def _create_trims_and_trajectory_panel(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Digital Trim Adjustments (Airframe drift compensation)
        trim_group = QGroupBox("Digital Trims (Airframe Drift Calibration)")
        trim_layout = QGridLayout(trim_group)
        trim_layout.setSpacing(6)

        # Pitch Trim
        trim_layout.addWidget(QLabel("Pitch Trim:"), 0, 0)
        btn_pt_dn = QPushButton("▼ Back")
        btn_pt_dn.clicked.connect(lambda: self._adjust_trim("pitch", -TRIM_STEP))
        trim_layout.addWidget(btn_pt_dn, 0, 1)
        self.lbl_trim_pitch = QLabel("0")
        self.lbl_trim_pitch.setAlignment(Qt.AlignCenter)
        self.lbl_trim_pitch.setFixedWidth(40)
        self.lbl_trim_pitch.setStyleSheet("font-weight: bold; color: #ffb74d;")
        trim_layout.addWidget(self.lbl_trim_pitch, 0, 2)
        btn_pt_up = QPushButton("▲ Fwd")
        btn_pt_up.clicked.connect(lambda: self._adjust_trim("pitch", TRIM_STEP))
        trim_layout.addWidget(btn_pt_up, 0, 3)

        # Roll Trim
        trim_layout.addWidget(QLabel("Roll Trim:"), 1, 0)
        btn_rl_lt = QPushButton("◀ Left")
        btn_rl_lt.clicked.connect(lambda: self._adjust_trim("roll", -TRIM_STEP))
        trim_layout.addWidget(btn_rl_lt, 1, 1)
        self.lbl_trim_roll = QLabel("0")
        self.lbl_trim_roll.setAlignment(Qt.AlignCenter)
        self.lbl_trim_roll.setFixedWidth(40)
        self.lbl_trim_roll.setStyleSheet("font-weight: bold; color: #ffb74d;")
        trim_layout.addWidget(self.lbl_trim_roll, 1, 2)
        btn_rl_rt = QPushButton("Right ▶")
        btn_rl_rt.clicked.connect(lambda: self._adjust_trim("roll", TRIM_STEP))
        trim_layout.addWidget(btn_rl_rt, 1, 3)

        # Yaw Trim
        trim_layout.addWidget(QLabel("Yaw Trim:"), 2, 0)
        btn_yw_lt = QPushButton("⟲ CCW")
        btn_yw_lt.clicked.connect(lambda: self._adjust_trim("yaw", -TRIM_STEP))
        trim_layout.addWidget(btn_yw_lt, 2, 1)
        self.lbl_trim_yaw = QLabel("0")
        self.lbl_trim_yaw.setAlignment(Qt.AlignCenter)
        self.lbl_trim_yaw.setFixedWidth(40)
        self.lbl_trim_yaw.setStyleSheet("font-weight: bold; color: #ffb74d;")
        trim_layout.addWidget(self.lbl_trim_yaw, 2, 2)
        btn_yw_rt = QPushButton("CW ⟳")
        btn_yw_rt.clicked.connect(lambda: self._adjust_trim("yaw", TRIM_STEP))
        trim_layout.addWidget(btn_yw_rt, 2, 3)

        # Reset Trims Button
        btn_reset_trims = QPushButton("Reset All Trims to 0 (Z)")
        btn_reset_trims.setStyleSheet("padding: 4px; font-weight: bold;")
        btn_reset_trims.clicked.connect(self._reset_trims)
        trim_layout.addWidget(btn_reset_trims, 3, 0, 1, 4)

        layout.addWidget(trim_group)

        # Autonomous Trajectory / Track Flight Panel
        traj_group = QGroupBox("Autonomous Track / Trajectory Flight")
        traj_layout = QVBoxLayout(traj_group)
        traj_layout.setSpacing(6)

        traj_row1 = QHBoxLayout()
        traj_row1.addWidget(QLabel("Pattern:"))
        self.combo_pattern = QComboBox()
        self.combo_pattern.addItem("Square (Box Pattern)", TrajectoryPattern.SQUARE)
        self.combo_pattern.addItem("Circle (360° Orbit)", TrajectoryPattern.CIRCLE)
        self.combo_pattern.addItem("Figure-8 (Lissajous)", TrajectoryPattern.FIGURE_8)
        self.combo_pattern.addItem("Triangle", TrajectoryPattern.TRIANGLE)
        traj_row1.addWidget(self.combo_pattern, stretch=2)
        traj_layout.addLayout(traj_row1)

        traj_row2 = QHBoxLayout()
        self.btn_start_traj = QPushButton("▶ Run Trajectory")
        self.btn_start_traj.setStyleSheet("background-color: #7b1fa2; color: white; font-weight: bold; padding: 6px;")
        self.btn_start_traj.clicked.connect(self._on_start_trajectory)
        traj_row2.addWidget(self.btn_start_traj)

        self.btn_stop_traj = QPushButton("⏹ Abort")
        self.btn_stop_traj.setEnabled(False)
        self.btn_stop_traj.setStyleSheet("background-color: #424242; color: white; font-weight: bold; padding: 6px;")
        self.btn_stop_traj.clicked.connect(self._on_stop_trajectory)
        traj_row2.addWidget(self.btn_stop_traj)
        traj_layout.addLayout(traj_row2)

        # Progress bar
        self.bar_traj = QProgressBar()
        self.bar_traj.setRange(0, 100)
        self.bar_traj.setValue(0)
        self.bar_traj.setFixedHeight(12)
        self.bar_traj.setTextVisible(False)
        traj_layout.addWidget(self.bar_traj)

        layout.addWidget(traj_group)
        return container

    def _create_gauges_and_dpad_panel(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Virtual Stick Position Indicators
        gauges_group = QGroupBox("Virtual Sticks (1..255, Center 128)")
        gauges_layout = QGridLayout(gauges_group)
        gauges_layout.setSpacing(4)

        # Throttle
        gauges_layout.addWidget(QLabel("Throttle:"), 0, 0)
        self.bar_throttle = QProgressBar()
        self.bar_throttle.setRange(0, 255)
        self.bar_throttle.setValue(0)
        gauges_layout.addWidget(self.bar_throttle, 0, 1)
        self.lbl_thr_val = QLabel("0")
        self.lbl_thr_val.setFixedWidth(50)
        gauges_layout.addWidget(self.lbl_thr_val, 0, 2)

        # Yaw
        gauges_layout.addWidget(QLabel("Yaw:"), 1, 0)
        self.bar_yaw = QProgressBar()
        self.bar_yaw.setRange(1, 255)
        self.bar_yaw.setValue(128)
        gauges_layout.addWidget(self.bar_yaw, 1, 1)
        self.lbl_yaw_val = QLabel("128")
        self.lbl_yaw_val.setFixedWidth(50)
        gauges_layout.addWidget(self.lbl_yaw_val, 1, 2)

        # Pitch
        gauges_layout.addWidget(QLabel("Pitch:"), 2, 0)
        self.bar_pitch = QProgressBar()
        self.bar_pitch.setRange(1, 255)
        self.bar_pitch.setValue(128)
        gauges_layout.addWidget(self.bar_pitch, 2, 1)
        self.lbl_pitch_val = QLabel("128")
        self.lbl_pitch_val.setFixedWidth(50)
        gauges_layout.addWidget(self.lbl_pitch_val, 2, 2)

        # Roll
        gauges_layout.addWidget(QLabel("Roll:"), 3, 0)
        self.bar_roll = QProgressBar()
        self.bar_roll.setRange(1, 255)
        self.bar_roll.setValue(128)
        gauges_layout.addWidget(self.bar_roll, 3, 1)
        self.lbl_roll_val = QLabel("128")
        self.lbl_roll_val.setFixedWidth(50)
        gauges_layout.addWidget(self.lbl_roll_val, 3, 2)

        layout.addWidget(gauges_group)

        # Directional 360-Degree Flip Controls (D-Pad Cluster)
        flip_group = QGroupBox("Directional 360° Flips (600ms Pulse)")
        flip_layout = QGridLayout(flip_group)
        flip_layout.setSpacing(4)

        self.btn_flip_fwd = QPushButton("▲ FRONT")
        self.btn_flip_fwd.setStyleSheet("background-color: #5c6bc0; font-weight: bold; padding: 6px;")
        self.btn_flip_fwd.clicked.connect(lambda: self._on_flip_clicked("forward"))
        flip_layout.addWidget(self.btn_flip_fwd, 0, 1)

        self.btn_flip_left = QPushButton("◀ LEFT")
        self.btn_flip_left.setStyleSheet("background-color: #5c6bc0; font-weight: bold; padding: 6px;")
        self.btn_flip_left.clicked.connect(lambda: self._on_flip_clicked("left"))
        flip_layout.addWidget(self.btn_flip_left, 1, 0)

        self.btn_flip_right = QPushButton("RIGHT ▶")
        self.btn_flip_right.setStyleSheet("background-color: #5c6bc0; font-weight: bold; padding: 6px;")
        self.btn_flip_right.clicked.connect(lambda: self._on_flip_clicked("right"))
        flip_layout.addWidget(self.btn_flip_right, 1, 2)

        self.btn_flip_back = QPushButton("▼ BACK")
        self.btn_flip_back.setStyleSheet("background-color: #5c6bc0; font-weight: bold; padding: 6px;")
        self.btn_flip_back.clicked.connect(lambda: self._on_flip_clicked("backward"))
        flip_layout.addWidget(self.btn_flip_back, 2, 1)

        layout.addWidget(flip_group)
        return container

    def _create_inspector_and_log_panel(self) -> QWidget:
        container = QFrame()
        container.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        header_layout = QHBoxLayout()
        self.lbl_tx_stats = QLabel("LIVE UDP PACKET STREAM (Port 7099 @ 20 Hz) | TX: 0 | HB: 0 | RX: 0")
        self.lbl_tx_stats.setStyleSheet("font-weight: bold; color: #4fc3f7; font-size: 11px;")
        header_layout.addWidget(self.lbl_tx_stats)
        header_layout.addStretch()

        btn_clear = QPushButton("Clear Log")
        btn_clear.setMaximumWidth(80)
        btn_clear.clicked.connect(self._clear_logs)
        header_layout.addWidget(btn_clear)
        layout.addLayout(header_layout)

        self.txt_packet_stream = QLabel("Last Packet: None")
        self.txt_packet_stream.setFont(QFont("Monospace", 9))
        self.txt_packet_stream.setStyleSheet("""
            background-color: #12151c;
            color: #00e676;
            padding: 4px;
            border: 1px solid #232936;
            border-radius: 4px;
        """)
        layout.addWidget(self.txt_packet_stream)

        self.txt_log_console = QTextEdit()
        self.txt_log_console.setReadOnly(True)
        self.txt_log_console.setFont(QFont("Monospace", 9))
        self.txt_log_console.setStyleSheet("""
            background-color: #0d1017;
            color: #cfd8dc;
            border: 1px solid #1e2532;
            border-radius: 4px;
        """)
        layout.addWidget(self.txt_log_console)

        return container

    def _setup_dark_theme(self):
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(24, 28, 36))
        palette.setColor(QPalette.WindowText, QColor(230, 235, 245))
        palette.setColor(QPalette.Base, QColor(16, 20, 28))
        palette.setColor(QPalette.AlternateBase, QColor(24, 28, 36))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, QColor(230, 235, 245))
        palette.setColor(QPalette.Button, QColor(36, 42, 54))
        palette.setColor(QPalette.ButtonText, QColor(230, 235, 245))
        palette.setColor(QPalette.Highlight, QColor(33, 150, 243))
        palette.setColor(QPalette.HighlightedText, Qt.white)
        self.setPalette(palette)

    # ── Flight Actions ────────────────────────────────────────

    def _toggle_arm(self):
        if self.client.flight_state in (FlightState.FLYING, FlightState.TAKING_OFF):
            reply = QMessageBox.question(
                self, "Mid-Air Disarm Confirmation",
                "The drone is currently IN FLIGHT!\n\n"
                "Disarming will immediately shut off motor power and the drone will drop.\n"
                "Are you sure you want to disarm mid-air?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.No:
                return
            self.client.disarm()
            self._log_warning("MOTORS DISARMED MID-AIR.")
        elif self.client.is_armed:
            self.client.disarm()
            self._log_info("MOTORS DISARMED. Throttle cut to 0.")
        else:
            self.client.arm()
            self._log_info("MOTORS ARMED. Throttle raised to neutral hover (128).")

    def _on_takeoff_clicked(self):
        if self.client.flight_state in (FlightState.TAKING_OFF, FlightState.FLYING, FlightState.LANDING):
            self._log_warning(f"Takeoff ignored: drone in state {self.client.flight_state.value}")
            return
        if self.client.trigger_takeoff():
            self._log_info("Takeoff pulse sent (1000ms duration, Bit 0 = 1, Throttle=128).")

    def _on_land_clicked(self):
        if self.client.flight_state not in (FlightState.FLYING, FlightState.TAKING_OFF):
            self._log_warning("Landing ignored: drone is not in flight!")
            return
        if self.client.trigger_land():
            self._log_info("Landing pulse sent (1000ms duration). Drone initiating autonomous descent.")

    def _on_emergency_clicked(self):
        self.client.trigger_emergency_stop()
        self._log_warning("EMERGENCY STOP TRIGGERED! Motor PWM cut immediately, throttle forced to 0.")

    def _on_gyro_clicked(self):
        if self.client.is_armed:
            QMessageBox.warning(self, "Safety Precaution", "Drone must be DISARMED on a flat level surface before gyro calibration!")
            return
        self.client.trigger_gyro_calibration()
        self._log_info("Gyroscope Calibration pulse initiated (2000ms). Keep drone completely still.")

    def _on_flip_clicked(self, direction: str):
        if not self.client.is_armed:
            QMessageBox.warning(self, "Arm Required", "Drone must be armed before executing 360-degree flips!")
            return
        if self.propeller_removed_mode:
            self._log_info(f"[BENCH TEST] Simulating 360 {direction.upper()} flip: 600ms pulse with full deflection.")
        self.client.trigger_flip(direction)
        self._log_info(f"Triggered 360 {direction.upper()} flip (600ms duration).")

    def _on_headless_clicked(self):
        is_on = self.client.toggle_headless()
        state = "ON" if is_on else "OFF"
        self.btn_headless.setText(f"Headless: {state} (H)")
        self.btn_headless.setStyleSheet(f"padding: 7px; font-weight: bold; {'color: #00e676;' if is_on else 'color: white;'}")
        self._log_info(f"Headless mode set to {state}.")

    def _on_lights_clicked(self):
        is_on = self.client.toggle_lights()
        state = "ON" if is_on else "OFF"
        self.btn_lights.setText(f"LEDs: {state}")
        self.btn_lights.setStyleSheet(f"padding: 7px; font-weight: bold; {'color: #ffeb3b;' if is_on else 'color: #78909c;'}")
        self._log_info(f"Drone LED lights toggled to {state}.")

    def _on_alt_hold_clicked(self):
        self.client.is_altitude_hold = not self.client.is_altitude_hold
        state = "ON" if self.client.is_altitude_hold else "OFF"
        self.btn_alt_hold.setText(f"Alt Hold: {state}")
        self.btn_alt_hold.setStyleSheet(f"padding: 7px; font-weight: bold; {'color: #4fc3f7;' if self.client.is_altitude_hold else 'color: #78909c;'}")
        self._log_info(f"Altitude hold mode set to {state}.")

    def _on_return_home_clicked(self):
        self.client.trigger_return_home()
        self._log_info("Triggered One-Key Return to Home (3.0s pulse).")

    def _on_circle_turn_clicked(self):
        is_on = self.client.toggle_circle_turn()
        state = "ON" if is_on else "OFF"
        self.btn_circle_turn.setText(f"Auto-Spin: {state} (O)")
        self.btn_circle_turn.setStyleSheet(f"padding: 7px; font-weight: bold; {'color: #ab47bc;' if is_on else 'color: white;'}")
        self._log_info(f"Continuous 360 Auto-Spin toggled to {state}.")

    def _on_stick_mode_changed(self, mode: ControlStickMode):
        self.client.set_stick_mode(mode)
        self._log_info(f"Switched stick layout to {mode.value}.")

    def _on_speed_changed(self, mode_id: int):
        mode = SpeedMode(mode_id)
        self.client.set_speed_mode(mode)
        names = {SpeedMode.LOW: "30%", SpeedMode.MEDIUM: "60%", SpeedMode.HIGH: "100%"}
        self._log_info(f"Speed mode set to {names.get(mode)} deflection scaling.")

    def _on_protocol_changed(self, proto_id: int):
        proto = ProtocolType.TC_21BYTE if proto_id == 1 else ProtocolType.GL_9BYTE
        self.client.set_protocol_type(proto)
        self._log_info(f"Hardware protocol switched to {proto.value}.")

    # ── Trim Handling ─────────────────────────────────────────

    def _adjust_trim(self, axis: str, delta: int):
        self.client.adjust_trim(axis, delta)
        self.lbl_trim_pitch.setText(f"{self.client.trim_pitch:+d}")
        self.lbl_trim_roll.setText(f"{self.client.trim_roll:+d}")
        self.lbl_trim_yaw.setText(f"{self.client.trim_yaw:+d}")
        self._log_info(f"Trim adjusted: {axis.upper()} = {getattr(self.client, 'trim_' + axis):+d}")

    def _reset_trims(self):
        self.client.reset_trims()
        self.lbl_trim_pitch.setText("0")
        self.lbl_trim_roll.setText("0")
        self.lbl_trim_yaw.setText("0")
        self._log_info("All drift trims reset to neutral (0).")

    # ── Trajectory Handling ───────────────────────────────────

    def _on_start_trajectory(self):
        if self.client.flight_state != FlightState.FLYING:
            QMessageBox.warning(self, "Flight Required", "Drone must be airborne in stable hover before running trajectory flight!")
            return
        pattern = self.combo_pattern.currentData()
        self.client.start_trajectory(pattern)
        self.btn_start_traj.setEnabled(False)
        self.btn_stop_traj.setEnabled(True)
        self.bar_traj.setValue(0)
        self._log_info(f"Started autonomous flight trajectory: {pattern.value}")

    def _on_stop_trajectory(self):
        self.client.stop_trajectory()
        self.btn_start_traj.setEnabled(True)
        self.btn_stop_traj.setEnabled(False)
        self.bar_traj.setValue(0)
        self._log_info("Autonomous flight trajectory aborted.")

    def _on_trajectory_progress(self, progress: float, pattern_name: str):
        self.bar_traj.setValue(int(progress * 100))
        if progress >= 1.0 or not self.client.trajectory_active:
            self.btn_start_traj.setEnabled(True)
            self.btn_stop_traj.setEnabled(False)

    # ── Telemetry & Safety ────────────────────────────────────

    def _on_telemetry_tick(self, flight_secs: float, battery_pct: int):
        m = int(flight_secs) // 60
        s = int(flight_secs) % 60
        self.lbl_flight_timer.setText(f"⏱ FLIGHT: {m:02d}:{s:02d}")
        self.bar_battery.setValue(battery_pct)

        # Color-code battery bar
        if battery_pct > 50:
            color = "#00e676"  # Green
        elif battery_pct > 20:
            color = "#ffb74d"  # Amber
        else:
            color = "#ff1744"  # Red warning
        self.bar_battery.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #455a64;
                border-radius: 3px;
                text-align: center;
                color: white;
                font-size: 10px;
                font-weight: bold;
            }}
            QProgressBar::chunk {{
                background-color: {color};
            }}
        """)

        # Update Gamepad badge
        if self.client.gamepad_connected:
            self.lbl_gamepad.setText(f"🎮 {self.client.gamepad_name.upper()}")
            self.lbl_gamepad.setStyleSheet("color: #00e676; font-size: 11px; font-weight: bold;")
        else:
            self.lbl_gamepad.setText("🎮 NO GAMEPAD")
            self.lbl_gamepad.setStyleSheet("color: #78909c; font-size: 11px;")

    def _on_prop_mode_toggled(self, checked: bool):
        self.propeller_removed_mode = checked
        if checked:
            self._log_warning("BENCH TEST MODE ACTIVATED: Safe testing mode enabled. Propellers must be removed!")
        else:
            reply = QMessageBox.question(
                self, "Live Flight Warning",
                "You are disabling Bench Test Mode.\n\n"
                "Ensure propellers are secured, surroundings are clear, and drone is placed on a flat, safe surface before flight!\n\n"
                "Proceed?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.No:
                self.chk_prop_test.setChecked(True)
                return
            self._log_warning("LIVE FLIGHT MODE ACTIVATED: Fly with caution!")

    def _on_flight_state_changed(self, state: FlightState):
        styles = {
            FlightState.DISARMED: ("[ DISARMED ]", "background-color: #37474f; color: #eceff1; padding: 4px 8px; border-radius: 4px; font-weight: bold; font-size: 12px;"),
            FlightState.ARMED: ("[ ARMED - READY ]", "background-color: #e65100; color: white; padding: 4px 8px; border-radius: 4px; font-weight: bold; font-size: 12px;"),
            FlightState.TAKING_OFF: ("[ TAKING OFF... ]", "background-color: #0288d1; color: white; padding: 4px 8px; border-radius: 4px; font-weight: bold; font-size: 12px;"),
            FlightState.FLYING: ("[ HOVER / FLYING ]", "background-color: #2e7d32; color: white; padding: 4px 8px; border-radius: 4px; font-weight: bold; font-size: 12px;"),
            FlightState.LANDING: ("[ AUTOLANDING ]", "background-color: #f57f17; color: white; padding: 4px 8px; border-radius: 4px; font-weight: bold; font-size: 12px;"),
            FlightState.EMERGENCY: ("[ EMERGENCY SHUTDOWN ]", "background-color: #b71c1c; color: white; padding: 4px 8px; border-radius: 4px; font-weight: bold; font-size: 12px;"),
        }
        text, style = styles.get(state, ("[ UNKNOWN ]", ""))
        self.lbl_armed_status.setText(text)
        self.lbl_armed_status.setStyleSheet(style)

        if state == FlightState.DISARMED:
            self.btn_takeoff.setEnabled(True)
            self.btn_land.setEnabled(False)
            self.btn_gyro.setEnabled(True)
            self.btn_arm_toggle.setText("ARM (C)")
        elif state == FlightState.ARMED:
            self.btn_takeoff.setEnabled(True)
            self.btn_land.setEnabled(False)
            self.btn_gyro.setEnabled(False)
            self.btn_arm_toggle.setText("DISARM (C)")
        elif state == FlightState.TAKING_OFF:
            self.btn_takeoff.setEnabled(False)
            self.btn_land.setEnabled(False)
            self.btn_gyro.setEnabled(False)
        elif state == FlightState.FLYING:
            self.btn_takeoff.setEnabled(False)
            self.btn_land.setEnabled(True)
            self.btn_gyro.setEnabled(False)
            self.btn_arm_toggle.setText("DISARM (MID-AIR)")
        elif state == FlightState.LANDING:
            self.btn_takeoff.setEnabled(False)
            self.btn_land.setEnabled(False)
            self.btn_gyro.setEnabled(False)
        elif state == FlightState.EMERGENCY:
            self.btn_takeoff.setEnabled(True)
            self.btn_land.setEnabled(False)
            self.btn_gyro.setEnabled(True)
            self.btn_arm_toggle.setText("ARM (C)")

    # ── Keyboard Event Handling ───────────────────────────────

    def keyPressEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            key = event.key()
            modifiers = event.modifiers()
            self.pressed_keys.add(key)
            self._handle_instant_key(key, modifiers)
        event.accept()

    def keyReleaseEvent(self, event: QKeyEvent):
        if not event.isAutoRepeat():
            self.pressed_keys.discard(event.key())
        event.accept()

    def _handle_instant_key(self, key: int, modifiers):
        # Shift + Keys for Trims
        if modifiers & Qt.ShiftModifier:
            if key == Qt.Key_W:
                self._adjust_trim("pitch", TRIM_STEP)
            elif key == Qt.Key_S:
                self._adjust_trim("pitch", -TRIM_STEP)
            elif key == Qt.Key_A:
                self._adjust_trim("roll", -TRIM_STEP)
            elif key == Qt.Key_D:
                self._adjust_trim("roll", TRIM_STEP)
            elif key == Qt.Key_J:
                self._adjust_trim("yaw", -TRIM_STEP)
            elif key == Qt.Key_L:
                self._adjust_trim("yaw", TRIM_STEP)
            return

        if key == Qt.Key_C:
            self._toggle_arm()
        elif key == Qt.Key_T:
            self._on_takeoff_clicked()
        elif key == Qt.Key_L:
            self._on_land_clicked()
        elif key == Qt.Key_E:
            self._on_emergency_clicked()
        elif key == Qt.Key_G:
            self._on_gyro_clicked()
        elif key == Qt.Key_H:
            self._on_headless_clicked()
        elif key == Qt.Key_R:
            self._on_return_home_clicked()
        elif key == Qt.Key_O:
            self._on_circle_turn_clicked()
        elif key == Qt.Key_Z:
            self._reset_trims()
        elif key == Qt.Key_M:
            new_mode = ControlStickMode.MODE_1_EU if self.client.stick_mode == ControlStickMode.MODE_2_US else ControlStickMode.MODE_2_US
            if new_mode == ControlStickMode.MODE_2_US:
                self.rb_mode2.setChecked(True)
            else:
                self.rb_mode1.setChecked(True)
        elif key == Qt.Key_1:
            self.rb_spd_low.setChecked(True)
            self._on_speed_changed(SpeedMode.LOW)
        elif key == Qt.Key_2:
            self.rb_spd_med.setChecked(True)
            self._on_speed_changed(SpeedMode.MEDIUM)
        elif key == Qt.Key_3:
            self.rb_spd_high.setChecked(True)
            self._on_speed_changed(SpeedMode.HIGH)
        elif key == Qt.Key_Space:
            self.client.center_sticks()
            self._log_info("Space pressed: Center all sticks / Hover.")

    def _poll_keyboard_sticks(self):
        """
        Polls active pressed keys and computes stick deflections with spring-back.
        Respects Mode 2 vs Mode 1 stick layouts.
        """
        # Determine throttle/yaw vs pitch/roll mapping based on Mode 1 vs Mode 2
        is_mode2 = (self.client.stick_mode == ControlStickMode.MODE_2_US)

        # Pitch: W/S (or Up/Down in Mode 1)
        target_pitch = STICK_CENTER
        if is_mode2:
            if Qt.Key_W in self.pressed_keys:
                target_pitch = STICK_CENTER - self.stick_step * 3
            elif Qt.Key_S in self.pressed_keys:
                target_pitch = STICK_CENTER + self.stick_step * 3
        else:
            if Qt.Key_I in self.pressed_keys:
                target_pitch = STICK_CENTER - self.stick_step * 3
            elif Qt.Key_K in self.pressed_keys:
                target_pitch = STICK_CENTER + self.stick_step * 3

        # Roll: A/D
        target_roll = STICK_CENTER
        if Qt.Key_A in self.pressed_keys:
            target_roll = STICK_CENTER - self.stick_step * 3
        elif Qt.Key_D in self.pressed_keys:
            target_roll = STICK_CENTER + self.stick_step * 3

        # Throttle: I/K (Mode 2) vs W/S (Mode 1)
        target_throttle = self.client.throttle
        if self.client.is_armed:
            up_pressed = (Qt.Key_I in self.pressed_keys or Qt.Key_Up in self.pressed_keys) if is_mode2 else (Qt.Key_W in self.pressed_keys)
            dn_pressed = (Qt.Key_K in self.pressed_keys or Qt.Key_Down in self.pressed_keys) if is_mode2 else (Qt.Key_S in self.pressed_keys)

            if up_pressed:
                target_throttle = min(STICK_MAX, target_throttle + 4)
            elif dn_pressed:
                target_throttle = max(STICK_MIN, target_throttle - 4)
            else:
                # Spring-back towards center hover (128)
                if target_throttle > STICK_CENTER:
                    target_throttle = max(STICK_CENTER, target_throttle - 2)
                elif target_throttle < STICK_CENTER:
                    target_throttle = min(STICK_CENTER, target_throttle + 2)
        else:
            target_throttle = STICK_IDLE_THROTTLE

        # Yaw: J/L
        target_yaw = STICK_CENTER
        if Qt.Key_J in self.pressed_keys or Qt.Key_Left in self.pressed_keys:
            target_yaw = STICK_CENTER - self.stick_step * 3
        elif Qt.Key_L in self.pressed_keys or Qt.Key_Right in self.pressed_keys:
            target_yaw = STICK_CENTER + self.stick_step * 3

        self.client.set_sticks(target_roll, target_pitch, target_throttle, target_yaw)
        self._update_gauges(target_roll, target_pitch, target_throttle, target_yaw)

    def _update_gauges(self, r: int, p: int, t: int, y: int):
        self.bar_roll.setValue(r)
        self.lbl_roll_val.setText(f"{r}")
        self.bar_pitch.setValue(p)
        self.lbl_pitch_val.setText(f"{p}")
        self.bar_throttle.setValue(t)
        self.lbl_thr_val.setText(f"{t}")
        self.bar_yaw.setValue(y)
        self.lbl_yaw_val.setText(f"{y}")

    # ── Signal Handlers ───────────────────────────────────────

    def _on_packet_sent(self, pkt: bytes, hex_str: str, is_hb: bool):
        if not is_hb:
            self.txt_packet_stream.setText(f"TX [{len(pkt)}B]: {hex_str}")
            self.lbl_tx_stats.setText(
                f"LIVE UDP PACKET STREAM (Port 7099 @ 20 Hz) | "
                f"TX: {self.client.tx_count} | HB: {self.client.hb_count} | RX: {self.client.rx_count}"
            )

    def _on_status_changed(self, armed: bool, msg: str):
        self._log_info(f"Status update: {msg}")

    def _on_telemetry_received(self, data: bytes):
        hex_data = " ".join(f"{b:02X}" for b in data)
        self._log_info(f"RX Telemetry ({len(data)}B): {hex_data}")

    def _log_info(self, msg: str):
        timestamp = time.strftime("%H:%M:%S")
        self.txt_log_console.append(f"[{timestamp}] INFO: {msg}")

    def _log_warning(self, msg: str):
        timestamp = time.strftime("%H:%M:%S")
        self.txt_log_console.append(f'<span style="color: #ff9800;">[{timestamp}] WARN: {msg}</span>')

    def _clear_logs(self):
        self.txt_log_console.clear()

    def closeEvent(self, event):
        self._log_info("Closing desktop controller application...")
        self.key_timer.stop()
        self.client.stop()
        event.accept()
