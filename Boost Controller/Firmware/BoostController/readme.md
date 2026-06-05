> **Warning:** This project controls turbocharger boost pressure. Incorrect wiring, calibration, tuning, or software behavior can damage an engine. Use conservative targets and verify all safety behavior before road use.

## Features

- Closed-loop PI boost control
- 16-bit Timer1 PWM MAC valve output
- 4 selectable boost profiles
- Gear- and RPM-based boost target maps
- CAN RPM/VSS input support
- Automatic gear detection from RPM/VSS ratio
- Gauge boost calculation from absolute MAP minus ambient pressure
- MAP and ambient pressure fault handling
- CAN timeout fault handling
- Overboost cut
- EGT, IAT, battery voltage, and CAN status reporting

Main pin assignments:

| Pin | Function |
|---|---|
| D9 / PB1 / OC1A | MAC valve PWM |
| A0 | MAP sensor |
| A1 | IAT sensor |
| A2 | Battery voltage |
| D6 / PD6 | CAN standby |
| D10 / PB2 | CAN chip select |

## Boost Control

The controller is designed for a bleed-style MAC valve setup:
- `0% duty` = wastegate actuator sees full boost pressure = spring pressure / safe state
- Higher duty = more pressure bled from the actuator = more boost

For this reason, all fault states command `0%` MAC duty.

Boost tables are in **gauge boost pressure, mbar**, not absolute MAP. Actual boost is calculated as:

```cpp
boost_mbar = map_mbar - ambient_pressure;
```

The PI controller compares calculated gauge boost against the table target and outputs direct Timer1 PWM counts from `0` to `ICR1`.

Timer1 is configured for 25 Hz PWM:

```cpp
ICR1 = 9999;
```

## Boost Maps

Boost maps are stored in `lookup_tables.h`.

Current map layout:

- 4 boost profiles
- 6 gear rows per profile
- 15 RPM breakpoints

Profiles:

| Profile | Name |
|---|---|
| 0 | Mild / Economy |
| 1 | Street |
| 2 | Sport |
| 3 | Race / Maximum |

The maps support 6 gears, but the current vehicle gear count can be limited separately:

```cpp
#define BOOST_NUM_GEARS    6
#define VEHICLE_NUM_GEARS  5
```

## Safety Features

The controller includes:

- MAP ADC range checking
- Ambient pressure sanity checking
- CAN RPM/VSS timeout detection
- Overboost cut
- Integrator reset on fault
- MAC output forced to `0%` on sensor or control faults

Important status flags:

```cpp
STATUS_OVERBOOST
STATUS_AMB_FAULT
STATUS_MAP_FAULT
STATUS_VSS_FAULT
STATUS_RPM_FAULT
```

## CAN

The controller can receive RPM and vehicle speed from an ECU CAN frame.

Default ECU frame:

```cpp
#define DME1_CAN_ID 0x316
```

The controller transmits an 8-byte status frame on CAN ID `0xFE`.

Current output frame:

| Byte | Data |
|---|---|
| 0 | Status flags |
| 1-2 | MAP, mbar |
| 3-4 | EGT |
| 5-6 | IAT |
| 7 | Max boost/profile byte |

## Scheduler

Timer2 provides a 5 ms base tick.

| Task | Interval |
|---|---:|
| MAP + RPM/VSS | 10 ms |
| Boost control | 40 ms |
| CAN transmit | 100 ms |
| IAT | 200 ms |
| EGT + battery voltage | 500 ms |

## Configuration

Main configuration is in `BoostController.h`.

```cpp
#define USE_CAN_SENSORS 1
#define DEBUG 0

#define BOOST_NUM_PROFILES 4
#define BOOST_NUM_GEARS 6
#define BOOST_NUM_RPM_POINTS 15
```

PI tuning values are in the main controller file:

```cpp
const int16_t boost_kp_x100 = 100;
const int16_t boost_ki_x100 = 2;
```

## Testing Checklist

Before road testing:

- Verify MAP reads near atmospheric pressure at key-on
- Verify calculated boost is near `0 mbar` with engine off
- Verify RPM, VSS, and gear detection
- Verify `0%` MAC duty gives wastegate spring pressure
- Verify MAP fault forces `0%` duty
- Verify CAN timeout forces `0%` duty
- Verify overboost cut forces `0%` duty
- Start with conservative boost targets
- Log target boost, actual boost, RPM, gear, MAC duty, and status flags

## Project Files

```text
BoostController/
├── BoostController.ino
├── BoostController.h
├── lookup_tables.h
└── README.md
```

## License

Same as whole repository
