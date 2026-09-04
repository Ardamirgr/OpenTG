# KLT120

> **Open-source discrete K-Line / L-Line transceiver for 3.3V and 5V microcontrollers**

**KLT120** is a discrete automotive **K-Line / L-Line transceiver** designed for interfacing microcontrollers with **ISO 9141**, **ISO 14230 / KWP2000**, and manufacturer-specific diagnostic buses.

Instead of relying on a dedicated K-Line transceiver IC, KLT120 uses **LM393B comparators** together with discrete **N-channel MOSFET line drivers**.

Depending on the fitted LDO, the logic side of the board can operate at either **3.3V or 5V**, allowing the same PCB to interface with a wide range of microcontrollers.

KLT120 is designed around a **120 kbaud target**, with additional headroom possible depending on bus capacitance, wiring, and the connected ECU. It remains fully suitable for traditional **10.4 kbaud K-Line communication**, **5-baud initialization**, and **fast initialization**.

---

## Features

| Feature | Description |
|---|---|
| **K-Line** | Full transmit and receive support |
| **L-Line** | Independent transmit and receive support |
| **Logic voltage** | 3.3V or 5V, selected by LDO choice |
| **PHY architecture** | LM393B comparators + discrete N-channel MOSFETs |
| **RX threshold** | Battery-referenced, approximately 0.5 × VBAT |
| **Bus pull-up** | Approximately 510–520 Ω tester-side pull-up |
| **Target speed** | 120 kbaud |
| **Protocols** | ISO 9141, ISO 14230 / KWP2000, manufacturer-specific K-Line |
| **Initialization** | 5-baud init and fast init |
| **Supply** | Automotive VBAT with protection and regulated logic rail |

---

## Logic Voltage

KLT120 can be configured for either **3.3V** or **5V** logic by fitting the appropriate LDO.

| LDO | Logic Rail |
|---|---:|
| `TLV7603` | `3.3V` |
| `TLV7605` | `5.0V` |

Throughout the design, this regulated logic rail is referred to as:

```text
VLOGIC
```

The selected logic voltage is used for:

- RX output pull-ups
- TX input logic levels
- Comparator TX reference generation

This allows the same PCB to be used with both modern **3.3V MCUs** and traditional **5V microcontrollers**.

---

## Architecture

Each diagnostic line consists of two main sections:

1. **Transmit driver**
2. **Receive comparator**

K-Line and L-Line are controlled independently.

```text
LM393B #1
├── K-Line TX
└── K-Line RX

LM393B #2
├── L-Line TX
└── L-Line RX
```

L-Line reception is optional in applications where only L-Line transmission is required.

---

## Transmit Path

The transmit stage uses one LM393B comparator as both an **inverter** and **logic-level detector**.

A reference voltage equal to approximately half of the selected logic voltage is generated from `VLOGIC`:

```text
TX_REF ≈ VLOGIC / 2
```

For example:

```text
3.3V logic → TX_REF ≈ 1.65V
5.0V logic → TX_REF ≈ 2.50V
```

The MCU transmit signal is connected to the inverting comparator input:

```text
MCU TX  -------- LM393B (-)
TX_REF  -------- LM393B (+)
```

Because the LM393B output is **open collector**, the MOSFET gate pull-up voltage can be chosen independently from the comparator supply voltage.

The comparator output drives an N-channel MOSFET used as a low-side K/L-Line switch.

### TX Logic

| MCU TX | MOSFET | K/L Line |
|---:|:---:|:---:|
| `HIGH` | OFF | HIGH / released |
| `LOW` | ON | LOW / dominant |

This preserves normal UART polarity.

When UART TX is idle (`HIGH`), the MOSFET remains OFF and the diagnostic line is released.

---

## K-Line / L-Line Output Stage

Each diagnostic line is pulled toward vehicle battery voltage through approximately **510–520 Ω** on the tester side.

The pull-up is implemented using **three 1206 resistors in series** to spread the power dissipation.

```text
VBAT
 |
~510–520 Ω
 |
 +------------- K/L LINE
 |
Drain
 NMOS
Source
 |
GND
```

At a vehicle charging voltage of **14.5V** and a **520 Ω** pull-up:

```text
I = 14.5 / 520
I ≈ 27.9 mA

P = V² / R
P ≈ 0.40 W
```

The line-driver MOSFET therefore only needs to sink roughly **30 mA** during a dominant LOW state.

Suitable SOT-23 N-channel MOSFETs include:

- `2N7002K`
- `BSS123`

Important MOSFET characteristics are:

- Sufficient `VDS` rating
- Sufficient `VGS` rating
- Low gate charge
- Ability to sink at least ~30 mA
- Suitable switching behavior at the selected gate voltage

Very low `RDS(on)` is not required because the bus current is relatively small.

---

## Receiver

The receive stage uses the second comparator in each LM393B.

Instead of comparing the diagnostic line against a fixed voltage, the receiver threshold is derived from **VBAT**. This keeps the switching point proportional to the actual K/L-Line HIGH level.

The diagnostic-line input is divided before reaching the comparator:

```text
VLINE_SENSE = VLINE / 4
```

The reference input is derived from VBAT:

```text
VREF = VBAT / 8
```

The comparator changes state when:

```text
VLINE / 4 = VBAT / 8
```

Therefore:

```text
VLINE = VBAT / 2
```

So the resulting receive threshold is approximately:

```text
VTH ≈ 0.5 × VBAT
```

This battery-referenced threshold is similar in principle to the input architecture used by dedicated automotive K-Line transceiver ICs.

---

## RX Logic Output

Because the LM393B output is **open collector**, the RX output can be pulled directly to the selected logic rail.

Depending on the fitted LDO, the MCU therefore receives either:

```text
0 / 3.3V
```

or:

```text
0 / 5.0V
```

No additional RX level shifter is required.

---

## K-Line and L-Line

K-Line is normally used for bidirectional communication between the diagnostic tester and ECU.

L-Line is primarily associated with initialization and wake-up on older ISO 9141 / KWP2000 systems.

KLT120 keeps both lines independent, allowing firmware to implement:

- K-Line-only communication
- K + L initialization
- 5-baud initialization
- Fast initialization
- Manufacturer-specific wake-up sequences
- Independent K-Line / L-Line monitoring

For applications that do not require L-Line reception, the `L_RX` comparator may be left unused.

---

## Baud Rate

KLT120 is designed around a target communication speed of:

```text
120 kbaud
```

At 120000 baud:

```text
Tbit = 1 / 120000
Tbit ≈ 8.33 µs
```

The LM393B propagation delay under large input overdrive is on the order of hundreds of nanoseconds, while the MOSFET switching time is significantly shorter.

The main limitation at higher baud rates is normally the **LOW-to-HIGH transition** of the diagnostic bus.

When the MOSFET releases the line, the pull-up resistor must charge the total bus capacitance.

The approximate RC time constant is:

```text
τ = R × C
```

With a 510–520 Ω pull-up:

| Bus Capacitance | Approx. RC Time Constant |
|---:|---:|
| `1 nF` | `~0.52 µs` |
| `5 nF` | `~2.60 µs` |
| `10 nF` | `~5.20 µs` |

The actual maximum usable baud rate depends on:

- Vehicle wiring
- Diagnostic cable length
- ECU input capacitance
- Protection components
- Connector capacitance
- Number of devices connected to the bus

Traditional **10.4 kbaud** K-Line operation has a very large timing margin.

> **120 kbaud is the intended design target, not a guarantee for every possible vehicle harness.**

---

## Why a Discrete Transceiver?

Dedicated K-Line transceiver ICs are convenient, but some parts can be difficult to source, relatively expensive, or inconvenient when working with configurable MCU logic levels.

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
- Compatible with both 3.3V and 5V systems
- Well suited to custom diagnostic hardware

It also allows the receive threshold, output drive, pull-up strength, and protection circuitry to be adapted for specific applications.

---

## Applications

KLT120 can be used for:

- Automotive diagnostic interfaces
- ECU flashing tools
- ECU reverse engineering
- KWP2000 communication
- ISO 9141 communication
- ECU bench harnesses
- ESP32-based diagnostic tools
- AVR-based diagnostic tools
- STM32-based diagnostic tools
- Data logging equipment
- Manufacturer-specific K-Line protocols
- 5-baud initialization
- Fast initialization
- Development and research hardware

---

## Project Status

> **KLT120 is currently under development.**

The hardware should be considered experimental until it has been validated against multiple ECUs, vehicles, wiring configurations, baud rates, and automotive transient conditions.

Testing is intended to cover both standard KWP2000 communication speeds and higher-speed K-Line implementations.

---

## Disclaimer

This project is intended for **development, research, diagnostic, and educational use**.

Automotive electrical systems can generate high-energy voltage transients. Incorrect connections may damage the interface, ECU, vehicle electronics, or connected development hardware.

**Use at your own risk.**
