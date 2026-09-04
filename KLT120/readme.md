# KLT120

**KLT120** is an open-source discrete automotive **K-Line / L-Line transceiver** designed for interfacing microcontrollers with ISO 9141, ISO 14230 / KWP2000, and manufacturer-specific diagnostic buses.

The design uses **LM393B comparators** and discrete N-channel MOSFET line drivers instead of a dedicated K-Line transceiver IC. Depending on the fitted LDO, the logic side of the board can operate at either **3.3V or 5V**, allowing KLT120 to interface with a wide range of microcontrollers.

The design target is reliable operation at **120 kbaud** and beyond, while remaining fully suitable for traditional 10.4 kbaud K-Line communication and slow initialization methods.

---

## Features

- Automotive K-Line interface
- Automotive L-Line interface
- Independent K-Line and L-Line control
- Supports either **3.3V or 5V MCU logic**
- Logic voltage selected by LDO choice
- LM393B comparator-based transmit and receive stages
- Discrete N-channel MOSFET bus drivers
- Battery-referenced receive thresholds
- Approximately 510Ω tester-side K/L pull-ups
- Protected automotive VBAT supply
- Designed for operation at **120 kbaud**
- Suitable for ISO 9141
- Suitable for ISO 14230 / KWP2000
- Suitable for manufacturer-specific K-Line protocols
- Supports 5-baud initialization
- Supports fast initialization

---

## Logic Voltage

KLT120 can be configured for either **3.3V or 5V logic** depending on the fitted LDO.

Depending on the selected regulator:

```text
VLOGIC = 3.3V for TLV7603
```

or:

```text
VLOGIC = 5.0V for TLV7605
```

The selected logic voltage is used for:

- RX output pull-ups
- TX input logic levels
- Comparator TX reference generation

This allows the same PCB design to be used with both modern 3.3V and traditional 5V microcontrollers.

---

## Architecture

Each diagnostic line consists of two main sections:

1. **Transmit driver**
2. **Receive comparator**

K-Line and L-Line are controlled independently.
A typical implementation uses two LM393B dual comparators:

```text
LM393B #1
├── K-Line TX
└── K-Line RX

LM393B #2
├── L-Line TX
└── L-Line RX
```

L-Line reception is optional for applications where only L-Line transmission is required.

---

## Transmit Path

The transmit stage uses one comparator as an inverter and logic-level detector.
A reference voltage equal to approximately half of the MCU logic voltage is generated using a resistor divider:

Therefore:

```text
TX_REF ≈ VLOGIC / 2
```

Examples:

```text
3.3V logic → TX_REF ≈ 1.65V
5.0V logic → TX_REF ≈ 2.50V
```

The MCU TX signal is connected to the inverting comparator input:

```text
MCU TX -------- LM393B (-)
TX_REF -------- LM393B (+)
```

The LM393B has an open-collector output, allowing the MOSFET gate pull-up voltage to be chosen independently from the comparator supply voltage.
The comparator output controls an N-channel MOSFET used as a low-side diagnostic-line driver.

The resulting logic is:

| MCU TX | MOSFET | K/L Line |
|---:|:---:|:---:|
| HIGH | OFF | HIGH / released |
| LOW | ON | LOW / dominant |

This preserves normal UART polarity.

UART idle is HIGH, so the MOSFET remains OFF and the diagnostic line is released when no data is being transmitted.

---

## K-Line / L-Line Output Stage

Each diagnostic line is pulled toward vehicle battery voltage through approximately **510 Ω** on the tester side.
A value around 510–520 Ω is suitable, implemented by three 1206 resistors in series

At a vehicle charging voltage of 14.5V and a 520 Ω pull-up:

```text
I = 14.5 / 520
I ≈ 27.9mA
P ≈ 0.41W
```

The MOSFET therefore only needs to sink approximately 30 mA during a dominant LOW state.
A SOT-23 N-channel MOSFET such as the **2N7002K** or **BSS123** is suitable for this application.

Important MOSFET parameters include:

- Sufficient VDS rating (>20V)
- Sufficient VGS rating (at least 20V)
- Low gate charge
- Ability to sink at least approximately 30mA
- Suitable switching behavior at the selected gate voltage

Low RDS(on) is not required because the bus current is relatively small.

---

## Receiver

The receiver uses the second comparator in each LM393B.
Instead of comparing the diagnostic line against a fixed voltage, the receive threshold is derived from vehicle battery voltage.

This keeps the switching point proportional to the actual K/L-Line HIGH level.
The diagnostic line is divided before entering the comparator:

This gives:

```text
VLINE_SENSE = VLINE / 4
```

The reference voltage is generated from VBAT giving:

```text
VREF = VBAT / 8
```

The comparator switches when:

```text
VLINE / 4 = VBAT / 8
```

therefore:

```text
VLINE = VBAT / 2
```

The resulting receiver threshold is approximately:

```text
VTH ≈ 0.5 × VBAT
```

This battery-referenced approach is similar to that used by dedicated automotive K-Line transceiver ICs.

---

## RX Logic Output

Because the LM393B output is open collector, the receive output can be pulled directly to the selected logic rail.
This means the same hardware can produce either:

```text
0 / 3.3V for TLV7603
```

or:

```text
0 / 5V for TLV7605
```

depending on the fitted LDO.

No additional RX level shifter is required.

---

## K-Line and L-Line

K-Line is normally used for bidirectional communication between the diagnostic tester and ECU.
L-Line is primarily associated with initialization and wake-up on older ISO 9141 / KWP2000 systems.
KLT120 provides independent control of both lines.

Keeping K and L separate allows firmware to implement:

- K-Line-only communication
- K + L initialization
- 5-baud initialization
- Fast initialization
- Manufacturer-specific wake-up sequences
- Independent line monitoring

For applications that do not require L-Line reception, the L_RX comparator may be left unused.

---

## Baud Rate

KLT120 is designed around a target communication speed of approximately:

```text
120 kbaud
```

At 120000 baud:

```text
Tbit = 1 / 120000
Tbit ≈ 8.33 µs
```

The LM393B propagation delay under large input overdrive is on the order of hundreds of nanoseconds, while the MOSFET switching time is significantly shorter.
The main limitation at higher baud rates is normally the LOW-to-HIGH transition of the diagnostic bus.

When the MOSFET releases the line, the pull-up resistor must charge the total bus capacitance.
The approximate RC time constant is:

```text
τ = R × C
```

With a 510 Ω pull-up:

```text
Cbus = 1 nF  → τ ≈ 0.52 µs
Cbus = 5 nF  → τ ≈ 2.60 µs
Cbus = 10 nF → τ ≈ 5.20 µs
```

The actual maximum usable baud rate therefore depends on:

- Vehicle wiring
- Diagnostic cable length
- ECU input capacitance
- Protection components
- Connector capacitance
- Number of devices connected to the bus

Traditional 10.4 kbaud K-Line operation has a very large timing margin.

**120 kbaud is the intended design target rather than a guarantee for every possible vehicle harness.**

---

## Why a Discrete Transceiver?

Dedicated K-Line transceiver ICs are convenient, but some devices can be difficult to source, relatively expensive, or not ideal for configurable MCU logic levels.
KLT120 implements the basic automotive diagnostic PHY using common components:

```text
Comparator
    +
MOSFET
    +
Resistors
    +
Protection
```

This makes the design:

- Easy to understand
- Easy to modify
- Easy to repair
- Flexible with component selection
- Compatible with 3.3V and 5V systems
- Suitable for custom diagnostic hardware

It also allows the receive threshold, output drive, pull-up strength, and protection circuitry to be adapted for specific applications.

---

## Applications

KLT120 can be used in projects including:

- Automotive diagnostic interfaces
- ECU flashing tools
- ECU reverse engineering
- KWP2000 communication
- ISO 9141 communication
- ECU bench harnesses
- ESP32-based diagnostic tools
- AVR-based diagnostic tools
- STM32-based diagnostic tools
- Logging equipment
- Manufacturer-specific K-Line protocols
- 5-baud initialization
- Fast initialization
- Development and research hardware

---

## Project Status

KLT120 is currently under development.
The hardware should be considered experimental until it has been validated against multiple ECUs, vehicles, wiring configurations, baud rates, and automotive transient conditions.
Testing is intended to cover both standard KWP2000 communication speeds and higher-speed K-Line implementations.

---

## Disclaimer

This project is intended for development, research, diagnostic, and educational use.
Automotive electrical systems can generate high-energy voltage transients, and incorrect connections may damage the interface, ECU, vehicle electronics, or connected development hardware.

Use at your own risk.
