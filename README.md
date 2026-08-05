<div align="center">

# OpenTG

**Open-source automotive electronics for monitoring, control, ECU communication, and calibration.**

Originally developed for the **Hyundai Coupe / Tiburon**, with many projects designed to be adaptable to other vehicles, ECUs, and custom engine-management setups.

[![Status](https://img.shields.io/badge/status-active-2ea44f)](#projects)
[![Hardware](https://img.shields.io/badge/hardware-open--source-blue)](#repository-contents)
[![Firmware](https://img.shields.io/badge/firmware-embedded-orange)](#repository-contents)

[Support OpenTG](https://www.paypal.com/donate/?hosted_button_id=CYKH5WKZKP4M2)

</div>

---

## Projects

### OpenTG AFR Meter

A complete wideband AFR/lambda system based on the **Bosch LSU 4.9** sensor and **Bosch CJ125** interface IC.

**Wideband controller**

- LSU 4.9 sensor and heater control
- CAN bus output
- Configurable analog AFR/lambda output
- Narrowband and wideband signal emulation

**Digital gauge**

- TFT display with AFR, lambda, oxygen concentration, and sensor-temperature views
- Configurable LED bar
- On-device menu and EEPROM-stored settings
- CAN communication with the controller

---

### OpenTG Boost Controller

A configurable electronic boost-control system for turbocharged vehicles using a MAC-style boost-control solenoid.

- Closed-loop boost control
- Boost targets by RPM and gear
- Multiple selectable boost profiles
- MAP and intake-air-temperature measurement
- CAN-based RPM and vehicle-speed input
- Optional analog inputs
- Overboost protection
- Exhaust-gas-temperature support
- Dedicated TFT gauge and user interface

---

### LTM43 Live Tuning Module

A live-calibration module designed specifically for the **Siemens SIMK43 ECU**.

The LTM43 replaces the original ECU flash memory with a removable module containing:

- **1 Mbit flash memory**, twice the capacity of the original 512 Kbit device
- **512 Kbit SRAM** connected in parallel with the flash memory
- A castellated pin-header adapter PCB that solders directly to the original flash footprint

> [!NOTE]
> The design is based on the live-calibration approach used by Siemens during ECU and engine development.

---

### SLCAN Adapter - Experimental

A compact CAN-to-USB serial interface for ECU flashing, diagnostics, logging, and calibration.

Typical uses include:

- SIMK43 communication
- ECU data logging
- CAN traffic monitoring
- LTM43 live tuning
- Use with compatible OpenTG and OpenGK tools

The adapter implements the **SLCAN protocol**, providing a low-cost alternative to commercial CAN interfaces for supported software.

---

### S4/RS4 MAF Adapter

An adapter board for integrating the Audi S4/RS4 B6 mass-airflow sensors with the stock SIMK43 ECU setups.

It provides a clean interface between the original harness connector and the MAF, without requiring any modifications.

Typical applications include:

- Turbo conversions
- Setups that exceed the measurement range of the original MAF sensor
- Replacement option if the stock MAF sensor can't be found from a reputable manufacturer (VDO, Continental).

---

### R8 COP Adapter

An ignition adapter board for converting the G4GC engines to Audi R8-style coil-on-plug ignition coils. This can theoretically work on any wasted-spark setup.

The board provides an organized interface between the factory engine harness and the replacement coils, simplifying wiring and installation.

> [!IMPORTANT]
> Coil compatibility and ignition strategy must be verified before installation. Depending on the ECU, the ignition dwell times WILL need to be changed, or the R8 Coils will be catastrophically damaged.
> For the G4GC dwell times of <2ms should be used. For most turbocharged setups 1.4-1.6ms is enough.

---

### Oil Temperature / Oil Pressure Gauge

A planned standalone gauge for monitoring engine-oil temperature and pressure.

**Status:** Under development. Have a prototype made from a modified Gauge unit, but no proper hardware design.

---

## Repository Contents

Depending on the project, each directory may include:

| Category | Contents |
|---|---|
| **Hardware** | Schematics, PCB layouts, Gerbers, BOMs, and assembly notes |
| **Firmware** | Embedded source code, binaries, and flashing instructions |
| **Wiring** | Pinouts, connection diagrams, and installation guidance |
| **Calibration** | Configuration files, lookup data, and setup notes |
| **Mechanical** | Enclosure, mounting, or other fabrication files |

Refer to the README inside each project directory for project-specific instructions.

---

## Intended Use

OpenTG projects are intended for enthusiasts, developers, motorsport applications, research, and education.

They may require fine-pitch soldering, ECU disassembly, firmware flashing, automotive electrical knowledge, and engine-management calibration experience.

> [!WARNING]
> Incorrect installation, wiring, firmware, or calibration can damage the ECU, engine, sensors, wiring, or connected equipment. Verify pin assignments, supply voltages, grounding, firmware versions, and board revisions before powering any hardware.

---

## Contributing

Contributions, testing, bug reports, documentation improvements, and hardware revisions are welcome.

For useful bug reports, include the relevant hardware revision, firmware version or commit, vehicle and ECU type, wiring configuration, CAN bitrate, logs or measurements, and a clear description of the problem.

---

## License

See the LICENSE file present in the repository root for license information.
