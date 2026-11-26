# 🌡️ DHT11 Boiler Monitor

**Flipper Zero `.fap` that polls a DHT11 sensor every 2 seconds and automatically fires Sub-GHz `.sub` triggers on temperature threshold crossings.**

Built in pure C for **Momentum firmware**, with live `°C` / `°F` readings, **Above** or **Below** thresholds, **Once** or **Interval** transmission modes, and persistent config storage.

> [!WARNING]
> **Safety & Legal Notice:** The DHT11 is a slow, low-cost **ambient** sensor—do not use it on boiler water pipes, flues, or for safety-critical heating control. Automated RF transmissions can trigger paired equipment unexpectedly; only transmit `.sub` signals you are authorized and legally permitted to use.

---

<details open>
<summary><strong>⚡ Features</strong></summary>

<br>

- **2-Second Telemetry:** Reads temperature & humidity every 2 seconds over Flipper GPIO (`PC0` / pin 16).
- **Threshold Sub-GHz TX:** Fires your selected `.sub` file when temperature crosses **Above** or **Below** your set limit (`0–30 °C` / `32–86 °F`).
- **Trigger Modes:**
  - **Once:** Fires on entry and rearms only after the temperature leaves the threshold condition.
  - **Interval:** Fires on entry and repeats at your chosen interval (`10 s` up to `1 hr`).
- **Switchable Units:** Toggle display units between Celsius (`°C`) and Fahrenheit (`°F`).
- **Persistent Settings:** Automatically loads and saves settings in `/ext/apps_data/boiler_monitor/config.bin`.

</details>

<details open>
<summary><strong>🔌 Wiring (3.3 V Only)</strong></summary>

<br>

| DHT11 Module | Flipper Zero | Notes |
| :--- | :--- | :--- |
| `VCC` / `+` | `3V3` (Pin 9) | **3.3 V only**—never 5 V |
| `DATA` / `S` | `PC0` (Pin 16) | Usually needs a `4.7k–10kΩ` pull-up resistor to `3V3` unless on the module |
| `GND` / `-` | `GND` (Pin 18) | Check silkscreen labels on your module before powering on |

</details>

<details open>
<summary><strong>🎮 Controls</strong></summary>

<br>

**Main Screen**
- **Up / Down:** Adjust threshold for the selected direction (`0–30 °C` / `32–86 °F`).
- **OK:** Arm or pause monitoring.
- **Hold Right:** Open **TX Settings** (Right no longer opens the file picker directly).

**TX Settings**
- **Up / Down:** Move between options.
- **OK:** Change a setting or open the `.sub` file picker.
- **Left:** Clear the selected `.sub` signal file.
- **Back:** Save settings and return to the main screen.

</details>

<details open>
<summary><strong>🛠️ Build & Install</strong></summary>

<br>

1. Install `uFBT` once:
   ```powershell
   py -m pip install --upgrade ufbt
