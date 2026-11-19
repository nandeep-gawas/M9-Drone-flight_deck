# M9 Drone Flight Deck

> **Linux Desktop Controller for FALAFLY F1 Series / M9 Ultra-Light / FLOW-UFO Toy Drones**  
> Complete Python flight cockpit reverse-engineered from the official Android `KY UFO` application (`com.cooingdv.kyufo`).  
> **Zero camera, RTSP, or video streaming dependencies.**

---

## 1. Hardware & Drone Profile

| Parameter | Specification |
|---|---|
| **Drone Model** | FALAFLY F1 Series (M9 Ultra-light) |
| **All-Up Weight** | ~67 grams |
| **Wi-Fi SSID** | `FLOW-UFO-XXXXXX` (e.g. `FLOW-UFO-a48a33`) |
| **Drone IP / Gateway** | `192.168.1.1` |
| **UDP Control Port** | `7099` |
| **Control Frequency** | 20 Hz (50 ms cycle) |
| **Keepalive Frequency** | 1 Hz (1000 ms cycle, wire payload: `[0x01, 0x01]`) |
| **Hardware Platform** | Bouffalo Lab BL60X / TC Protocol (`deviceType == 10`) |
| **Stick Modes** | Mode 2 (Default / US) & Mode 1 (European / Right-Hand Throttle) |
| **Gamepad Support** | Native Linux `/dev/input/js*` USB & Bluetooth controller input |
| **Operating System** | Linux Mint / Ubuntu / Debian / Fedora / Arch |

---

## 2. Complete Features Reconstructed from Android App

1. **Flight Controls**:
   - One-Button Auto Takeoff (verified 1000ms pulse with altitude hold lock)
   - One-Button Auto Landing (verified 1000ms pulse with controlled descent)
   - **Emergency Stop** (instant motor power cutoff mid-air, throttle forced to 0)
   - Manual Throttle, Pitch, Roll, and Yaw controls with smooth spring-back
2. **Special Maneuvers & Flight Stunts**:
   - **Tornado 360** (`trigger_tornado_360`): Calibrated single complete 360-degree yaw rotation in hover (`U` for Clockwise, `Shift + U` for Counter-Clockwise)
   - Directional 360° Flips (Front Flip, Back Flip, Left Flip, Right Flip)
   - Continuous 360° Auto-Spin / Circle Turn (`isCircleTurn`)
   - One-Key Return to Home (`isFastReturn`)
   - Headless Mode (`setNoHeadMode`)
   - Optical Flow / Barometer Altitude Hold Lock (`isFixedHeightMode`)
   - Gyroscope Calibration (`setGyroCorrection`, 2.0s level ground pulse)
   - Drone LED Lights Toggle (`isOpenLight`)
3. **Autonomous Track / Trajectory Flight (`trackFly`)**:
   - **Square**: Autonomous 4-sided box flight pattern
   - **Circle**: 360° orbital trajectory
   - **Figure-8**: Lissajous looping pattern
   - **Triangle**: 3-point delta trajectory
   - Real-time progress bar, pattern selector, and instant abort control
4. **Digital Trim System (Physical Airframe Drift Compensation)**:
   - Pitch Trim (Forward / Backward fine tuning $\pm 24$)
   - Roll Trim (Left / Right fine tuning $\pm 24$)
   - Yaw Trim (CCW / CW fine tuning $\pm 24$)
   - One-click "Reset All Trims to 0"
5. **Stick Layout Switcher**:
   - **Mode 2 (US / World Standard)**: Left = Throttle & Yaw, Right = Pitch & Roll
   - **Mode 1 (European / Japanese)**: Left = Pitch & Yaw, Right = Throttle & Roll
6. **Live Telemetry & Dashboard**:
   - Flight Duration Timer (MM:SS clock active only while armed/in-flight)
   - Estimated Battery Gauge & Voltage Meter (percentage with color alerts)
   - Native Linux USB/Bluetooth Gamepad detection (`/dev/input/js*`)
   - Live 20 Hz UDP Hex Packet Inspector & Diagnostic Console
   - Propeller-Removed Bench Testing Mode for desk testing

---

## 3. Protocol Specification

### 21-Byte TC Frame (FLOW-UFO / BL60X Default)
```
[0x03, 0x66, 0x14, Roll, Pitch, Throttle, Yaw, Cmd, Aux, 0x00*10, Checksum, 0x99]
```

* **Byte 0**: Transport prefix `0x03`
* **Byte 1**: Frame header `0x66`
* **Byte 2**: Payload length `0x14` (20 bytes)
* **Byte 3**: **Roll** (`1..255`, neutral `128 / 0x80`, Left `< 128`, Right `> 128` + Trim)
* **Byte 4**: **Pitch** (`1..255`, neutral `128 / 0x80`, Forward `< 128`, Backward `> 128` + Trim)
* **Byte 5**: **Throttle** (`0..255`, neutral hover `128 / 0x80`, Down `< 128`, Up `> 128`)
* **Byte 6**: **Yaw** (`1..255`, neutral `128 / 0x80`, Turn Left `< 128`, Turn Right `> 128` + Trim)
* **Byte 7 (Cmd Bits)**:
  * `Bit 0` (`0x01`): `TAKEOFF_LAND` (1.0s active pulse)
  * `Bit 1` (`0x02`): `EMERGENCY` (1.0s pulse, motor cut)
  * `Bit 2` (`0x04`): `GYRO_CALIB` (2.0s pulse on flat surface)
  * `Bit 3` (`0x08`): `FLIP` (0.6s pulse + directional deflection)
  * `Bit 4` (`0x10`): `LIGHT_ON` (Drone LEDs active, default `1`)
* **Byte 8 (Aux Bits)**:
  * `Bit 0` (`0x01`): `HEADLESS` (Headless navigation mode)
  * `Bit 1` (`0x02`): `ALTITUDE_HOLD` (Optical flow / barometer lock, default `1`)
* **Bytes 9–18**: 10 Reserved padding bytes (`0x00`)
* **Byte 19**: **Checksum** — XOR sum of bytes 3 through 18:
  $$\text{Checksum} = \text{Roll} \oplus \text{Pitch} \oplus \text{Throttle} \oplus \text{Yaw} \oplus \text{Cmd} \oplus \text{Aux}$$
* **Byte 20**: Tail delimiter `0x99`

---

## 4. Project Structure

```
M9-Drone-flight_deck/
├── flight_deck/
│   ├── __init__.py           # Package exports
│   ├── controller.py         # Full PyQt5 desktop cockpit GUI
│   ├── network.py            # 20 Hz UDP engine, keepalive, trims, trajectory, gamepad
│   ├── protocol.py           # Dalvik-accurate packet encoder & verifier
│   └── state.py              # FlightState, ProtocolType, SpeedMode, StickMode enums
├── tests/
│   ├── __init__.py
│   └── test_protocol.py      # Automated unit test suite (23 tests)
├── tools/
│   ├── __init__.py
│   └── packet_inspector.py   # Pre-flight loopback UDP packet inspection tool
├── requirements.txt          # Python dependencies (PyQt5)
├── run_controller.py         # Application entry point
├── LICENSE                   # MIT License
└── README.md                 # Full documentation
```

---

## 5. Keyboard & Gamepad Controls

### Keyboard Reference

| Key | Function | Action |
|---|---|---|
| **`W` / `S`** | Pitch (Mode 2) / Throttle (Mode 1) | Forward / Backward (or Climb / Descend) |
| **`A` / `D`** | Roll | Bank Left / Bank Right |
| **`I` / `K`** or **`Up` / `Down`** | Throttle (Mode 2) / Pitch (Mode 1) | Climb / Descend (or Forward / Backward) |
| **`J` / `L`** or **`Left` / `Right`**| Yaw | Turn Left / Turn Right |
| **`Space`** | Center Sticks | Immediately centers all sticks to hover (128) |
| **`C`** | Arm / Disarm Motors | Ground arming toggle |
| **`T`** | One-Button Auto Takeoff | 1000ms pulse (`Bit 0 = 1`) |
| **`L`** | One-Button Auto Landing | 1000ms pulse (`Bit 0 = 1`) |
| **`E`** | **Emergency Motor Stop** | Immediately cuts motor power mid-air |
| **`G`** | Gyroscope Calibration | 2000ms pulse on flat surface |
| **`H`** | Headless Mode Toggle | Latching orientation toggle |
| **`R`** | One-Key Return Home | Commands return flight vector toward pilot |
| **`O`** | 360° Auto-Spin / Circle Turn | Smooth continuous yaw rotation in hover |
| **`U`** | **Tornado 360 (CW)** | Executes one complete 360° clockwise yaw spin |
| **`Shift + U`** | **Tornado 360 (CCW)** | Executes one complete 360° counter-clockwise yaw spin |
| **`M`** | Stick Mode Toggle | Switches between Mode 2 (US) and Mode 1 (EU) |
| **`Z`** | Reset All Trims | Re-zeros Pitch, Roll, and Yaw trims |
| **`Shift + W/S`** | Fine Pitch Trim | Forward / Backward trim adjustment ($\pm 2$) |
| **`Shift + A/D`** | Fine Roll Trim | Left / Right trim adjustment ($\pm 2$) |
| **`Shift + J/L`** | Fine Yaw Trim | CCW / CW trim adjustment ($\pm 2$) |
| **`1` / `2` / `3`** | Speed Selection | 30% Beginner / 60% Cruising / 100% Sport |

### Linux USB / Bluetooth Gamepad Mapping

* **Left Stick**: Yaw (X-axis) & Throttle (Y-axis)
* **Right Stick**: Roll (X-axis) & Pitch (Y-axis)
* **Button A (Cross)**: Takeoff
* **Button B (Circle)**: Land
* **Button X (Square)**: Front 360° Flip
* **Button Y (Triangle)**: **Emergency Motor Stop**
* **LB / RB**: Headless Toggle / Altitude Hold Toggle

---

## 6. How to Run

```bash
# 1. Run unit test suite
python3 -m unittest discover tests

# 2. Pre-flight bench testing (loopback mode)
python3 run_controller.py --loopback

# 3. Live drone flight (connected to FLOW-UFO Wi-Fi)
python3 run_controller.py
```

---

## 7. License

MIT License - Copyright (c) 2026 Nandeep Gawas
