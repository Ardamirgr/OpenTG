# OpenTG Wideband Controller Firmware

Firmware for an AVR-based wideband oxygen sensor controller using the **Bosch CJ125** interface IC.  
The controller:

- Reads CJ125 analog outputs (**UA**, **UR**) via ADC
- Regulates the sensor heater using **PI control**
- Outputs an analog **0-1V (or 0-5V) emulation** using an **MCP4725 DAC**
- Publishes live data over **CAN bus**
- Supports **wideband** and **narrowband** analog emulation modes
- Includes startup sequencing: power/sensor validation → CJ125 calibration → heater warmup → closed-loop control

---

## Features

- **CJ125 SPI interface**
  - Reads CJ125 diagnostic register periodically
  - Switches CJ125 between calibration and normal measurement mode
- **Heater warmup state machine**
  - Condensation phase (low heater power)
  - Ramp-up phase (increasing heater power)
  - Stabilization phase (hold until optimal UR)
  - Handover to PI closed-loop regulation
- **PI heater controller**
  - Uses UR feedback against stored optimal UR from calibration
- **Analog output**
  - **Wideband emulation**: 0V = AFR 20, 1V = AFR 10 (linear between)
  - **Narrowband emulation**: thresholded mapping around stoich
- **CAN telemetry**
  - Transmits status flags, lambda, AFR, sensor temperature, battery reading
  - Receives control flags over CAN to switch emulation/debug behavior
- **Watchdog safety**
  - Watchdog enabled (2s) and fed throughout the control loops

---

## Hardware

See the EAGLE PCB files in the relevant folder.


## Repository Layout

This sketch includes several project headers:

- `OpenTG.h`  
  Status flag bit definitions, constants (e.g., `UBAT_MIN`, `UHEATER_*`), pin macros, CAN retry settings.
- `lookup_tables.h`  
  Lookup tables for:
  - UA ADC → lambda (`Lambda_Conversion`)
  - UR-derived value → temperature (`Temp_Conversion`)
- Any CJ125 register constants/macros used:
  - `CJ125_DIAG_REG_REQUEST`, `CJ125_DIAG_REG_STATUS_OK`, etc.
  - Mode set commands: `CJ125_INIT_REG1_MODE_CALIBRATE`, `CJ125_INIT_REG1_MODE_NORMAL_V8`

---

## Startup Sequence Overview

The firmware uses a structured startup orchestrator `start`:

1. **waitForPowerAndSensor**
   - Loops until:
     - battery voltage ≥ `UBAT_MIN`
     - CJ125 diagnostic register reports **OK**
   - Updates `status_flags` bits:
     - low battery
     - sensor present
     - short circuit / wiring fault
   - Includes a simple “engine on” detection window comparing current battery voltage against the initial key-on value.

2. **calibrateCJ125**
   - Puts CJ125 into calibration mode
   - Samples and stores:
     - `sens_signal_optimal` (UA at λ=1)
     - `sens_resistance_optimal` (UR at optimal temp)
   - Computes a small UA calibration offset to align UA to `306` ADC counts
   - Switches CJ125 back to normal mode (V=8 gain)

3. **heaterWarmup**
   - **Condensation phase**: low heater power for a fixed duration
   - **Ramp-up phase**: step heater voltage upward until UR approaches optimal
   - **Stabilization**: hold power until UR reaches target
   - **Handover**: heater control transitions to PI closed-loop regulation

After startup, the firmware enters `loop` continuously.

---

## Main Loop Behavior

Each loop iteration:

1. Read CJ125 diagnostics via SPI (two reads)
2. Read ADCs:
   - UA (`sens_signal`) + calibration offset
   - UR (`sens_resistance`)
   - battery (`v_batt`)
3. If battery low or CJ125 not OK:
   - heater off
   - force re-initialization by calling `start`
4. Otherwise:
   - compute PI heater PWM from UR (`heater_control`)
   - compute lambda from UA using lookup table (`lookup_lambda`)
   - compute AFR from lambda (stored as integer + decimal packed into `afr_value`)
   - compute sensor temp from UR (`lookup_temp`)
   - update analog output via MCP4725 (`update_analog`)
   - send CAN when the 100ms communication flag is set

---

## CAN Protocol

### TX frame (controller → bus)
- **CAN ID:** `0xFE`
- **DLC:** 8

| Byte | Meaning |
|------|---------|
| 0 | `status_flags` |
| 1 | lambda high byte |
| 2 | lambda low byte |
| 3 | AFR packed high byte |
| 4 | AFR packed low byte |
| 5 | sensor temp high byte |
| 6 | sensor temp low byte |
| 7 | battery ADC byte |

**AFR packing**
- AFR is computed as `afr = lambda * 14.7`
- Packed as: `(afr_int << 8) | afr_decimal`
  - `afr_int` = integer part
  - `afr_decimal` = 0–99

### RX frame (bus → controller)
- **CAN ID:** `0xFF`
- **DLC:** 2

| Byte | Meaning |
|------|---------|
| 0 | `flagEMU` (0 = wideband, 1 = narrowband) |
| 1 | `flagDebug` (1 forces debug lambda source) |

When `flagDebug == 1`, `STATUS_DEBUG` is set and lambda is derived from `sens_signal_optimal` rather than live UA.

---

## Analog Output Modes

Analog output is produced by the MCP4725 DAC, scaled for a 0–1V or 0-5V range:

### Wideband emulation (default)
- AFR **10** → **1.0V**/**5.0V**
- AFR **20** → **0.0V**
- Linear mapping between

### Narrowband emulation
- Uses thresholds around stoich with a steep transition band to emulate narrowband behaviour.
- Keeps ECU happy.
- It could be used to replace narrowband sensor that gives fuel feedback. Try at your own risk.

---

## Status Flags

`status_flags` is an 8-bit bitfield defined in `OpenTG.h`.  
Examples used in the code:

- `STATUS_LOW_BAT`
- `STATUS_SENSOR`
- `STATUS_SHORT_CIRC`
- `STATUS_ENGINE_ON`
- `STATUS_HEAT`
- `STATUS_NORMAL`
- `STATUS_DEBUG`
- `STATUS_RECEIVE`

---

## Watchdog

- Watchdog is enabled in `setup` with a 2-second timeout.
- The firmware feeds the watchdog throughout:
  - power/sensor wait loops
  - warmup phases
  - main loop

---

## Building / Flashing

This project is intended for an AVR Arduino-compatible environment.

1. Install required libraries:
   - Arduino **CAN** library
   - **SPI** library (core)
   - **Adafruit MCP4725** library
   - **Make sure to use only the libraries that are provided in this repo.**

2. Ensure your `OpenTG.h` and `lookup_tables.h` are present and correct for your board and sensor wiring.

3. Compile and flash using Arduino IDE / PlatformIO. **You will need an Arduino or similar to act as an ICSP**.  

---

## Notes / Customization

- Heater control tuning:
  - `p_gain`, `i_gain`, `iMax` in the PI controller
- Warmup behavior:
  - `UHEATER_START`, `UHEATER_MAX`, `UHEATER_RAMP_STEP` in headers
- Battery thresholds:
  - `UBAT_MIN` in headers
- CAN timing:
  - Timer1 interrupt triggers communication window every **100ms**

---

## Safety Disclaimer

Wideband heater control involves significant power and temperature.  
Use appropriate sensor wiring, fusing, and validated heater profiles. Misconfiguration can damage the sensor, controller hardware, or vehicle wiring.

---

## Attribution

This firmware is loosely based on Bylund Automotive’s Lambda Shield project (https://github.com/Bylund/Lambda-Shield-Example/tree/master).  
Some of the original heater PID code and CJ125 implementation has been copied over, but most of the code and logic in this repository have been heavily modified or are original.


## License

See the repository root for license information.
