# M9 Drone Flight Deck

> **Linux Desktop Controller for FALAFLY F1 Series / M9 Ultra-Light / FLOW-UFO Toy Drones**  
> Clean, lightweight, modular Python desktop application reverse-engineered from the official Android `KY UFO` application (`com.cooingdv.kyufo`).  
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
| **Operating System** | Linux Mint / Ubuntu / Debian / Fedora / Arch |

---

## 2. Root Cause Analysis of Previous Flight Bugs

### Bug A: Automatic Takeoff Kept Climbing (Runaway Ascent)
* **Root Cause**: In naive implementations, the takeoff command flag (`Bit 0` of Byte 7) was either left asserted continuously in the 20 Hz transmission loop, or the throttle stick was improperly held above neutral (e.g. 200–255), or Byte 8 altitude hold (`Bit 1`) was set to 0. In the BL60X optical-flow firmware:
  1. Takeoff is a **transient 1000 ms pulse** (`Bit 0 = 1`).
  2. Throttle must be **neutral hover** (`128 / 0x80`) during and after takeoff.
  3. After 1000 ms, `Bit 0` **must transition back to 0**. If `Bit 0` remains 1, the flight controller firmware interprets it as a continuous climbing command or repeated takeoff trigger.
  4. Byte 8 (`AuxByte`) must assert `Bit 1 = 1` (`0x02`) to engage optical flow / barometer altitude hold.

### Bug B: Landing Button Did Not Work
* **Root Cause**: The landing trigger in the Android APK Dalvik bytecode (`FlyController.setFastDrop`) activates the exact same command bit (`Bit 0 = 1`) as takeoff, with throttle at neutral hover (`128 / 0x80`). When a controller attempted to send a different undocumented command code or set throttle to 0, the drone ignored it. Furthermore, sending `Bit 0 = 1` indefinitely prevented the firmware from entering its autonomous descent sequence.
* **The Fix**: The controller issues a clean **1000 ms pulse** of `Bit 0 = 1` at neutral throttle (`128`), then returns `Bit 0` to `0` and transitions internal state to `LANDING`, allowing the drone's optical-flow descent algorithm to land smoothly.

### Bug C: Emergency Stop vs. Landing vs. Disarming
* **Emergency Stop (`Bit 1 = 1`, Throttle = `0`)**: Completely and immediately cuts PWM power to all 4 motors mid-air. The drone drops instantly. **Reserved strictly for emergencies.**
* **Controlled Landing (`Bit 0 = 1` for 1.0s, Throttle = `128`)**: Triggers firmware auto-descent with active gyro and optical stabilization until touchdown.
* **Disarm**: Cuts throttle to `0` on the ground. Mid-air disarm requires explicit user confirmation.

---

## 3. Protocol Specification

### 21-Byte TC Frame (FLOW-UFO / BL60X Default)
```
[0x03, 0x66, 0x14, Roll, Pitch, Throttle, Yaw, Cmd, Aux, 0x00*10, Checksum, 0x99]
```

* **Byte 0**: Transport prefix `0x03`
* **Byte 1**: Frame header `0x66`
* **Byte 2**: Payload length `0x14` (20 bytes)
* **Byte 3**: **Roll** (`1..255`, neutral `128 / 0x80`, Left `< 128`, Right `> 128`)
* **Byte 4**: **Pitch** (`1..255`, neutral `128 / 0x80`, Forward `< 128`, Backward `> 128`)
* **Byte 5**: **Throttle** (`0..255`, neutral hover `128 / 0x80`, Down `< 128`, Up `> 128`)
* **Byte 6**: **Yaw** (`1..255`, neutral `128 / 0x80`, Turn Left `< 128`, Turn Right `> 128`)
* **Byte 7 (Cmd Bits)**:
  * `Bit 0` (`0x01`): `TAKEOFF_LAND` (1.0s active pulse)
  * `Bit 1` (`0x02`): `EMERGENCY` (1.0s pulse, motor kill)
  * `Bit 2` (`0x04`): `GYRO_CALIB` (2.0s pulse on flat surface)
  * `Bit 3` (`0x08`): `FLIP` (0.6s pulse + stick deflection)
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
│   ├── controller.py         # PyQt5 desktop cockpit GUI
│   ├── network.py            # 20 Hz UDP engine, keepalive, and state machine
│   ├── protocol.py           # Dalvik-accurate packet encoder & verifier
│   └── state.py              # FlightState, ProtocolType, SpeedMode enums
├── tests/
│   ├── __init__.py
│   └── test_protocol.py      # Automated unit test suite (16 tests)
├── tools/
│   ├── __init__.py
│   └── packet_inspector.py   # Pre-flight loopback UDP packet inspection tool
├── requirements.txt          # Python dependencies (PyQt5)
├── run_controller.py         # Application entry point
├── LICENSE                   # MIT License
└── README.md                 # Full documentation
```

---

## 5. Installation & Setup

### Prerequisites
* Python 3.8+
* PyQt5

```bash
# Clone the repository
git clone git@github.com:nandeep-gawas/M9-Drone-flight_deck.git
cd M9-Drone-flight_deck

# Install requirements
pip3 install -r requirements.txt
```

---

## 6. Pre-Flight Verification & Bench Testing

> [!IMPORTANT]
> **ALWAYS bench test with PROPELLERS REMOVED before conducting actual flight!**

### Step 1: Run the Automated Unit Tests
```bash
python3 -m unittest discover tests
```
*Expected: 16 tests passing (`OK`).*

### Step 2: Run the Loopback Packet Inspector
In Terminal 1, run the inspector to monitor UDP port 7099:
```bash
python3 tools/packet_inspector.py --port 7099
```

In Terminal 2, launch the flight deck in loopback bench mode:
```bash
python3 run_controller.py --loopback
```

Observe in Terminal 1:
1. Normal neutral hover shows:
   `[VALID] R=128 P=128 T=128 Y=128 | Cmd=0x10 Aux=0x02 Chk=0x12 | ALT_HOLD`
2. Pressing `T` (Takeoff) shows a 1.0-second pulse:
   `[VALID] R=128 P=128 T=128 Y=128 | Cmd=0x11 Aux=0x02 Chk=0x13 | TAKEOFF/LAND_PULSE | ALT_HOLD`
   followed immediately by returning to `Cmd=0x10` (no continuous climb).
3. Pressing `E` (Emergency) immediately drops Throttle to 0:
   `[VALID] R=128 P=128 T=  0 Y=128 | Cmd=0x12 Aux=0x02 Chk=0x90 | EMERGENCY_STOP | ALT_HOLD`

---

## 7. Live Flight Operating Guide

1. Power on the drone on a flat, level surface.
2. Connect laptop Wi-Fi to `FLOW-UFO-XXXXXX`.
3. Launch controller:
   ```bash
   python3 run_controller.py
   ```
4. Perform **Gyro Calibration (`G`)** while drone is motionless on the ground.
5. Uncheck **PROPELLER-REMOVED BENCH TEST MODE** once verified.
6. Press **`C`** to ARM motors (propellers will spin at idle).
7. Press **`T`** to TAKEOFF. Drone rises to ~1.0m and enters stable optical-flow hover.
8. Control flight using keyboard inputs.
9. Press **`L`** to LAND smoothly.
10. In any hazard, press **`E`** (EMERGENCY STOP) to immediately cut power.

### Keyboard Controls Reference

| Key | Function | Wire Value / Action |
|---|---|---|
| **`W` / `S`** | Pitch (Forward / Backward) | `W: < 128`, `S: > 128` |
| **`A` / `D`** | Roll (Left / Right) | `A: < 128`, `D: > 128` |
| **`I` / `K`** or **`Up` / `Down`** | Throttle (Climb / Descend) | `I: > 128`, `K: < 128` |
| **`J` / `L`** or **`Left` / `Right`** | Yaw (Turn Left / Right) | `J: < 128`, `L: > 128` |
| **`Space`** | Center Sticks (Neutral Hover) | `R=128, P=128, T=128, Y=128` |
| **`C`** | Arm / Disarm Motors | Toggles Ground Arming State |
| **`T`** | One-Button Auto Takeoff | 1000ms pulse (`Cmd Bit 0 = 1`) |
| **`L`** | One-Button Auto Landing | 1000ms pulse (`Cmd Bit 0 = 1`) |
| **`E`** | **Emergency Motor Stop** | Cuts PWM power immediately (`Cmd Bit 1 = 1`) |
| **`G`** | Gyroscope Calibration | 2000ms pulse (`Cmd Bit 2 = 1`) |
| **`H`** | Headless Mode Toggle | Toggles `Aux Bit 0` |
| **`1` / `2` / `3`** | Speed Modes | 30% Low / 60% Medium / 100% High |

---

## 8. Feature Status

- [x] 21-Byte TC Protocol framing and XOR checksum
- [x] Optical flow altitude hold (`Aux Bit 1 = 1`)
- [x] Timed 1000ms Takeoff pulse with auto-hover transition
- [x] Timed 1000ms Landing pulse with descent state
- [x] Instant Emergency Stop with throttle cut
- [x] Gyroscope calibration (2000ms level surface pulse)
- [x] Directional 360° flips (Front, Back, Left, Right)
- [x] Headless mode toggle
- [x] Client-side speed deflection limiting (30%, 60%, 100%)
- [x] Smooth keyboard spring-back virtual joysticks (50 Hz)
- [x] 1 Hz Keepalive heartbeat (`[0x01, 0x01]`) & Teardown (`[0x08, 0x01]`)
- [x] Propeller-removed bench testing safety guard
- [x] Live hex packet inspector and telemetry logging console
- [x] Complete automated unit test suite

---

## 9. License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
