# How to Use M9 Drone Flight Deck

This guide provides step-by-step instructions for operating the **M9 Drone Flight Deck** on Linux Mint / Ubuntu / Debian.

---

## 1. Quick Launch

You can start the software using the launcher script or Python:

```bash
cd M9-Drone-flight_deck

# Run in safe bench testing mode (no drone required)
./launch.sh --loopback

# Run live controller (connected to FLOW-UFO Wi-Fi)
./launch.sh
```

---

## 2. Live Flight Procedure (Step-by-Step)

```
[1. Power On] ➔ [2. Connect Wi-Fi] ➔ [3. Launch App] ➔ [4. Calibrate Gyro (G)]
       ➔ [5. Uncheck Bench Mode] ➔ [6. Arm Motors (C)] ➔ [7. Takeoff (T)] ➔ [8. Land (L)]
```

### Step 1: Power On the Drone
* Place your M9 drone on a flat, level surface indoors or in a clear area.
* Power the drone on. The navigation LEDs will begin flashing.

### Step 2: Connect Laptop Wi-Fi
* Open your Linux network manager.
* Select and connect to the drone's Wi-Fi network: **`FLOW-UFO-XXXXXX`** (e.g. `FLOW-UFO-a48a33`).
* *Note: The network has no password. Linux may notify you that there is "No Internet", which is expected.*

### Step 3: Launch Controller
```bash
./launch.sh
```
* The cockpit GUI will open. The top bar will show `● NETWORK READY` and `Drone IP: 192.168.1.1:7099`.

### Step 4: Calibrate Gyroscope (`G`)
* Ensure the drone is resting completely still on the level floor.
* Press the **`G`** key on your keyboard or click **`Calibrate Gyro (G)`**.
* The IMU sensors will zero out over 2 seconds, and the LEDs will become solid.

### Step 5: Disable Bench Test Mode
* Uncheck **`BENCH TEST MODE`** in the top bar.
* Confirm that propellers are installed and the area is clear.

### Step 6: Arm the Motors (`C`)
* Press **`C`** on your keyboard or click **`ARM (C)`**.
* The status badge will change to **`[ ARMED - READY ]`** and motors will spin at low idle speed.

### Step 7: Auto Takeoff (`T`)
* Press **`T`** on your keyboard or click **`TAKEOFF (T)`**.
* The drone will ascend smoothly to ~1 meter, engage optical-flow altitude hold, and stabilize in hover.

### Step 8: Manually Fly & Trim
* **Fly Forward / Backward:** `W` / `S`
* **Bank Left / Right:** `A` / `D`
* **Climb Up / Descend Down:** `I` / `K` (or `Up` / `Down` Arrow)
* **Turn Left / Right:** `J` / `L` (or `Left` / `Right` Arrow)
* **Instant Hover:** Release keys or tap `Spacebar`.
* **Correct Drift:** Use `Shift + W/S/A/D` to adjust trims, or `Z` to reset.

### Step 9: Safe Landing (`L`)
* Fly the drone above your target landing zone.
* Press **`L`** on your keyboard or click **`LAND (L)`**.
* The drone will execute an automated descent and shut off its motors upon touchdown.

---

## 3. Emergency Motor Cutoff (`E`)

> [!WARNING]
> If the drone drifts toward an obstacle, a wall, or someone walks into its path, press **`E`** immediately.  
> This cuts all motor power instantly mid-air to prevent collisions or damage.

---

## 4. Keyboard Controls Reference

| Shortcut | Function | Description |
|:---:|:---|:---|
| **`C`** | **Arm / Disarm** | Toggles motor idle spin on the ground |
| **`T`** | **Auto Takeoff** | 1000ms pulse; ascends to 1.0m hover |
| **`L`** | **Auto Land** | Controlled descent until touchdown |
| **`E`** | **Emergency Stop** | Instant mid-air motor power cutoff |
| **`G`** | **Gyro Calibration** | 2.0s level ground sensor alignment |
| **`Space`** | **Hover Center** | Centers all sticks to neutral hover (`128`) |
| **`W / S`** | **Pitch** | Forward / Backward flight |
| **`A / D`** | **Roll** | Left / Right bank |
| **`I / K`** | **Throttle** | Climb / Descend |
| **`J / L`** | **Yaw** | Rotate Left / Rotate Right |
| **`H`** | **Headless Mode** | Toggles relative pilot orientation |
| **`R`** | **Return Home** | Auto-flies toward pilot orientation |
| **`O`** | **Auto-Spin** | Continuous 360° circle yaw rotation |
| **`M`** | **Stick Mode** | Toggles Mode 2 (US) vs Mode 1 (EU) |
| **`Z`** | **Reset Trims** | Re-zeros Pitch, Roll, and Yaw trims |
| **`Shift + WASD`**| **Fine Trims** | Drift micro-adjustments ($\pm 2$) |
| **`1 / 2 / 3`** | **Speed Modes** | 30% Low / 60% Medium / 100% Sport |

---

## 5. Gamepad / Joystick Support

Plug in any USB or Bluetooth gamepad (Xbox, PlayStation, Logitech, etc.):
* **Left Stick:** Throttle (vertical) & Yaw (horizontal)
* **Right Stick:** Pitch (vertical) & Roll (horizontal)
* **Button A (Cross):** Takeoff
* **Button B (Circle):** Land
* **Button X (Square):** Front 360° Flip
* **Button Y (Triangle):** **Emergency Stop**
* **LB / RB:** Headless Mode / Altitude Hold
