# DHT11 Boiler Monitor

This Flipper Zero external app reads temperature and humidity from a DHT11
sensor and displays the readings. It can transmit selected Sub-GHz `.sub`
files when the temperature crosses configured above/below thresholds.

The DHT11 is a slow, low-cost ambient sensor. It is not suitable for measuring
boiler water or flue temperatures, or for boiler safety/control. Do not rely on
this app for a safety decision or to control heating equipment. RF transmission
may cause paired equipment to operate unexpectedly. Only transmit signals you
are authorized and legally permitted to use.

## Wiring

The app uses the Flipper GPIO header's **PC0 / pin 16** for the sensor data
line:

| DHT11 module | Flipper Zero |
| --- | --- |
| VCC / `+` | 3V3, header pin 9 |
| DATA / `S` | PC0, header pin 16 |
| GND / `-` | GND, header pin 18 |

Check the labels on your particular module before connecting it. Use 3.3 V,
not 5 V, for the sensor supply or data pull-up. A bare DHT11 usually needs a
4.7k-10k pull-up resistor from DATA to 3V3; a 3-pin breakout module may already
include one.

## Build

Install uFBT once:

```powershell
py -m pip install --upgrade ufbt
```

From the `boiler_monitor` app directory, run:

```powershell
ufbt
```

The `.fap` is generated under `dist`. Copy it to the Flipper SD card under
`ext\apps`, then launch it from the Apps menu. The app reads the DHT11 every
two seconds, deferring a sensor read if a Sub-GHz transmission is still active
to protect the DHT11's timing-sensitive signal.

Hold Right on the main screen to open **TX Settings**. Use Up/Down to move,
OK to change a setting or open the `.sub` picker, and Left to clear the
selected signal file. Back saves and returns to the main screen. Select one
trigger direction (Above or Below), choose its signal file, then choose Once
or Interval and the repeat interval. The Units option switches the display
between Celsius and Fahrenheit. The temperature threshold is not changed in
this menu: adjust it directly on the main screen, between 0 and 30°C (32 and
86°F). Interval mode sends on entering the threshold condition and repeats at
the selected interval (10 seconds to 1 hour); Once mode sends only on entry
and rearms after the next valid reading leaves the condition. Above mode stops
sending as soon as a valid reading falls below its threshold; Below mode stops
when a valid reading rises above its threshold. Readings are sampled every two
seconds, so threshold changes are recognized on the next successful sample.

On the main screen, Up/Down adjusts the selected direction's threshold and OK
arms/pauses monitoring. Right no longer opens a file picker. Settings are
loaded from and saved to
`/ext/apps_data/boiler_monitor/config.bin`.

The app uses the installed uFBT SDK's `InputTypeLong` input event and Storage
API (`storage_file_alloc`, `storage_common_mkdir`) for the hold gesture and
versioned settings file. Build against an SDK compatible with your installed
Momentum firmware. If the FAP does not build or load with the default uFBT SDK,
use the matching Momentum firmware source/build tools to build the app for
that firmware.
