# Boost Controller

An open-source electronic boost controller for driving a MAC valve solenoid using closed-loop MAP feedback.

The controller supports boost-by-gear, boost-by-RPM, selectable boost profiles, CAN-based engine data, analog sensor inputs, EGT monitoring, and a companion gauge/display unit.

> [!WARNING]
> This project controls turbocharger boost pressure. Incorrect wiring, calibration, plumbing, or control behavior can cause overboost, lean operation, engine damage, turbocharger damage, drivetrain damage, or loss of traction. Bench-test the hardware first, start with low boost targets, and keep proper ECU-side overboost/fuel-cut protection enabled.

---

## Features

- Closed-loop boost control using MAP feedback
- MAC valve PWM control through MOSFET output
- Boost target by gear and RPM
- Up to 4 selectable boost profiles
- CAN-based RPM and vehicle speed input
- Optional analog RPM/VSS input support
- MAP sensor input
- Integrated IAT reading through MAP sensor NTC
- Type-K EGT reading using MCP9600
- CAN output for external gauge/display
- Status and fault reporting
- Debug mode with serial output
- Companion gauge support for boost, EGT, IAT, status, active profile, and LED bar display

> [!NOTE]
> The firmware is structured in an ECU-like way: lookup tables, interpolation, scheduled tasks, fixed-point integer math, and status/fault flags instead of blocking control logic.

---

## System Overview

The Boost Controller reads engine and vehicle data, calculates a target boost level from calibration tables, and controls a MAC valve solenoid with a PWM-driven MOSFET.

```text
RPM / VSS / Gear / Profile
          |
          v
Boost target lookup table
          |
          v
Atmospheric pressure compensation
          |
          v
Closed-loop PI boost control
          |
          v
MAC valve PWM output
```

The controller is intended for custom turbocharged engine setups where flexible gear-dependent boost control is required.

---

## Hardware

The controller is designed around:

| Hardware | Purpose |
|---|---|
| ATmega328PB or similar AVR MCU | Main controller |
| MCP25625 | CAN controller/transceiver |
| MAC valve boost solenoid | Wastegate pressure control |
| Logic-level MOSFET | MAC valve low-side drive |
| MAP sensor with integrated IAT NTC | Boost and intake temperature measurement |
| MCP9600 | Type-K thermocouple interface |
| 5V logic supply | MCU and sensor logic |
| 12V automotive supply input | Vehicle power and solenoid supply |
| Companion gauge/display | User interface and live feedback |

### Main Inputs

| Input | Description |
|---|---|
| MAP | Manifold absolute pressure sensor |
| IAT | Integrated NTC inside the MAP sensor |
| EGT | Type-K thermocouple through MCP9600 |
| RPM | CAN or hardware timer input |
| VSS | CAN or hardware timer input |
| Profile Select | Received from dashboard/gauge over CAN |

### Main Outputs

| Output | Description |
|---|---|
| MAC PWM | 15–30 Hz PWM output for MAC valve |
| CAN TX | Sensor/status data sent to gauge |
| Status LEDs | Power and CAN status indication |

> [!IMPORTANT]
> Use proper automotive protection: fuse the supply, protect the MOSFET gate, use flyback suppression for the solenoid, and make sure the MAC valve plumbing fails to a safe mechanical wastegate pressure.

---

## Boost Control Strategy

Boost targets are stored as **gauge pressure in mbar**.

The controller captures atmospheric pressure at key-on and adds the selected boost target on top of it:

```c
target_map_abs = ambient_pressure + boost_target_gauge;
```

The PI controller compares this absolute target against the live MAP reading:

```c
error = target_map_abs - measured_map_abs;
```

The PI output then drives the MAC valve PWM duty cycle.

This keeps the boost tables intuitive while allowing the controller to operate using absolute pressure internally.

> [!TIP]
> Keep boost calibration tables in gauge mbar for easy tuning, but compare absolute pressure internally because the MAP sensor measures absolute manifold pressure.

---

## Boost Profiles

The firmware supports up to 4 boost profiles:

| Profile | Intended Use |
|---:|---|
| 0 | Mild / Economy |
| 1 | Street |
| 2 | Sport |
| 3 | Race / Maximum |

Each profile contains a gear/RPM boost target map:

```c
boost_profile[gear][rpm_point]
```

The RPM axis is shared across all profiles and boost targets are interpolated between breakpoints.

> [!NOTE]
> Boost map values are gauge pressure values. The controller adds the stored ambient pressure to create the final absolute MAP target.

---

## Scheduler

The firmware uses a Timer2-based cooperative scheduler with a 5 ms base tick.

Typical task timing:

| Task | Interval |
|---|---:|
| MAP read | 10 ms |
| RPM/VSS read | 10 ms |
| Boost control | 40 ms |
| CAN transmit | 100 ms |
| IAT read | 200 ms |
| EGT / ambient read | 500 ms |

The MAC valve PWM itself is generated separately by Timer1.

> [!NOTE]
> The boost PI routine is intentionally slower than the ADC sampling. Sensor values are refreshed first, then the 40 ms boost-control task uses the latest cached values.

---

## MAC Valve PWM

The MAC valve is driven using Timer1 PWM.

Default configuration:

| Parameter | Value |
|---|---:|
| PWM frequency | 25 Hz |
| Output | Timer1 OC1A |
| Duty command range | 0–255 |

The firmware can compensate for the usable MAC valve duty range. For example, if the valve is non-responsive below `20/255` and fully open above `215/255`, the logical 0–255 control output can be mapped into the effective valve range.

> [!CAUTION]
> Forced PWM test patterns should only be run on the bench or with boost-control plumbing safely disconnected. Do not sweep the MAC valve duty on a live boost system unless you know exactly how the plumbing will respond.

---

## CAN Communication

The controller receives RPM and vehicle speed data from the ECU when CAN sensor mode is enabled.

It also transmits live data to the companion gauge.

### Controller-to-Gauge CAN Frame

| Byte | Data |
|---:|---|
| 0 | Status flags |
| 1–2 | MAP absolute pressure, mbar |
| 3–4 | EGT, °C |
| 5–6 | IAT, °C × 10 |
| 7 | Profile max boost, encoded as `max_boost * 100 mbar` |

Example:

```c
byte7 = 30; // 3000 mbar max boost
```

The gauge uses the profile max boost value to scale the LED bar and determine display warning colors.

> [!IMPORTANT]
> CAN-derived RPM/VSS data is timeout-protected. If ECU CAN messages stop arriving, the controller invalidates RPM and VSS instead of continuing to use stale values.

---

## Gauge Display

The companion gauge can display:

- Boost in bar
- Boost in PSI
- EGT in °C
- IAT
- Status flags
- Active boost profile
- LED boost bar

The LED bar uses the profile max boost value for scaling.

Recommended behavior:

| Condition | LED / Display Behavior |
|---|---|
| Below max boost | Green display, normal LED scaling |
| At max boost | Yellow display / yellow LED |
| Over max boost | Red display / red LED |

> [!NOTE]
> The last two LEDs are intended as warning indicators: yellow for reaching the profile maximum and red for exceeding it.

---

## Status Flags

The firmware uses an 8-bit status register for reporting controller state and faults.

Example status layout:

```c
#define STATUS_DEBUG          (1 << 7)
#define STATUS_ENGINE_ON      (1 << 6)
#define STATUS_CAN_ACTIVE     (1 << 5)
#define STATUS_BOOST_ACTIVE   (1 << 4)
#define STATUS_EGT_FAIL       (1 << 3)
#define STATUS_MAP_FAULT      (1 << 2)
#define STATUS_VSS_FAULT      (1 << 1)
#define STATUS_LOW_BAT        (1 << 0)
```

These flags are transmitted over CAN and can be displayed by the gauge.

> [!TIP]
> Keep status bits specific. Avoid vague flags like `NORMAL` when possible; it is usually better to report the exact fault or active state.

---

## CAN Timeout Handling

When CAN-based RPM/VSS is used, the firmware tracks message freshness.

If ECU CAN messages stop arriving, RPM and VSS are invalidated after a timeout. This prevents the controller from continuing to use stale data.

Typical timeout:

```c
#define CAN_TIMEOUT_TICKS 20
```

With a 5 ms scheduler tick, this gives a timeout of approximately 100 ms.

When the timeout expires:

- RPM returns 0
- VSS returns 0
- Gear detection returns neutral/unknown
- Boost control is disabled by RPM threshold
- Fault/status bits can be set

> [!WARNING]
> Stale RPM or VSS data can cause unsafe boost control decisions. Timeout handling should remain enabled when CAN sensor mode is used.

---

## Sensors

### MAP Sensor

The MAP sensor is read through the ADC and converted to pressure using a lookup table.

The MAP reading is treated as absolute pressure in mbar.

### IAT Sensor

The IAT is an NTC integrated into the MAP sensor.

The firmware uses an ECU-style 1D lookup table:

```c
ADC axis -> temperature
```

Temperature is represented as `°C × 10`.

Example:

```c
253  =  25.3 °C
-100 = -10.0 °C
```

### EGT Sensor

EGT is measured using a Type-K thermocouple and the MCP9600.

The MCP9600 is used to read:

- Thermocouple temperature
- Board/ambient temperature

EGT is mainly used for display and warning/alert functionality.

> [!CAUTION]
> Sensor calibration is critical. Validate raw ADC values and converted values before enabling closed-loop boost control.

---

## Gear Detection

When gear is not directly received from CAN, the controller estimates gear from RPM and vehicle speed.

The basic relationship is:

```c
ratio = rpm / vss;
```

The calculated ratio is compared against calibrated expected values for each gear.

If no valid gear is detected, the controller returns gear `0`, meaning neutral, clutch-in, shifting, or invalid state.

> [!NOTE]
> Unknown gear returns a zero boost target. This prevents boost control during shifts, clutch-in conditions, or implausible RPM/VSS combinations.

---

## Safety Behavior

The firmware includes several safety-oriented behaviors:

- Boost control disabled below minimum RPM
- MAC valve output forced to zero when RPM is too low
- CAN timeout protection
- Unknown gear returns zero boost target
- Low battery flag support
- MAP/IAT plausibility checks can be added
- EGT failure and alert support
- Optional MAC valve response monitoring

> [!WARNING]
> Software boost control should never be the only safety layer. Use a safe mechanical wastegate spring, correct boost plumbing, proper overboost protection, and conservative initial calibration.

---

## Debug Mode

The firmware includes a debug mode that can output live data over serial.

Typical debug output includes:

- MAP
- IAT
- RPM
- VSS
- Gear
- Boost target
- MAC PWM duty
- EGT
- Ambient temperature
- Battery voltage
- CAN age
- Status flags

Debug mode can also step the MAC valve PWM output through a test pattern:

```text
0%, 20%, 40%, 60%, 80%, 0%
```

This is useful for bench-testing the MOSFET driver and valve output.

> [!CAUTION]
> Do not run forced PWM debug patterns on a live boost system unless the boost-control plumbing is safely disconnected.

---

## Development Notes

The project follows an ECU-style firmware structure:

- Calibration data stored in lookup tables
- Interpolation for sensor conversion and target lookup
- Scheduled periodic tasks
- Status flags for diagnostics
- CAN communication for external display
- Fixed-point integer math where practical

This keeps the firmware lightweight and suitable for AVR hardware.

---

## Installation / Bring-Up Checklist

> [!IMPORTANT]
> Validate the system in stages. Do not install the controller and immediately run high boost.

Recommended bring-up order:

1. Confirm 5V and 12V supplies.
2. Confirm CAN communication.
3. Confirm raw MAP and IAT ADC readings.
4. Confirm MAP and IAT conversion tables.
5. Confirm MCP9600 EGT and ambient readings.
6. Confirm Timer1 PWM frequency on a scope.
7. Confirm MOSFET gate and MAC valve output behavior.
8. Confirm RPM/VSS acquisition and gear detection.
9. Run with wastegate spring pressure only.
10. Enable low boost targets and gradually increase after logging.

---

## License

See the repository root for license information.

---

## Project Status

Work in progress.

Current development focus:

- Boost-by-gear and RPM control
- MAP/IAT calibration
- CAN RPM/VSS integration
- MCP9600 EGT support
- Gauge display integration
- MAC valve behavior testing
- Safety/fault handling
