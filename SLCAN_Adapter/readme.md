# GKFlasher SLCAN Interface Firmware

This firmware turns the hardware used in the **OpenAFR AFR Gauge** or **Boost Controller Gauge** into a temporary USB-to-CAN interface compatible with **GKFlasher**.

The gauge hardware already contains the required components:

- ATmega328PB microcontroller
- FT232RL USB-to-UART interface
- MCP25625 CAN controller and transceiver
- CAN connector and supporting circuitry

By flashing this firmware, a regular AFR or Boost Controller gauge can be repurposed as a quick CAN-to-serial interface without building a separate adapter.

The firmware implements a basic **Lawicel/SLCAN-compatible serial protocol**, allowing the board to be used through `python-can`, Scapy, and the modified CAN backend used by GKFlasher.

> This firmware temporarily replaces the normal gauge firmware. Reflash the original AFR or Boost Controller firmware when the gauge is needed for normal use again.

---

## Features

- USB virtual COM port through the FT232RL
- SLCAN/Lawicel-compatible serial command interface
- 11-bit and 29-bit CAN frame support
- Classical CAN data frames and RTR frames
- Configurable CAN bitrate
- Optional SLCAN timestamps
- CAN receive buffering
- Compatible with `python-can` using the `slcan` backend
- Intended for quick GKFlasher setup and diagnostic use

Supported SLCAN commands include:

| Command | Description |
|---|---|
| `C` | Close the CAN channel |
| `O` | Open the CAN channel |
| `L` | Open in listen mode request |
| `S0`…`S8` | Select a standard CAN bitrate |
| `t` | Transmit an 11-bit CAN data frame |
| `T` | Transmit a 29-bit CAN data frame |
| `r` | Transmit an 11-bit RTR frame |
| `R` | Transmit a 29-bit RTR frame |
| `V` | Read firmware version |
| `N` | Read adapter serial number |
| `Z0` | Disable timestamps |
| `Z1` | Enable timestamps |
| `F` | Read status flags |

---

## Hardware

The intended hardware is a standard gauge PCB from either:

- the OpenAFR AFR Controller and Gauge project, or
- the Boost Controller Gauge project

The relevant signal path is:

```text
PC / GKFlasher
      |
      | USB
      v
   FT232RL
      |
      | UART
      v
 ATmega328PB
      |
      | SPI
      v
  MCP25625
      |
      | CAN-H / CAN-L
      v
 Vehicle CAN bus
```

The firmware assumes:

- Schematic does not deviate from original OpenTG one
- 500 kbit/s vehicle CAN operation for GKFlasher

---

## Serial Configuration

The default UART rate is:

```text
1,000,000 baud
8 data bits
No parity
1 stop bit
```

At a 16 MHz ATmega clock, 1 Mbaud can be generated exactly and provides enough bandwidth for typical diagnostic CAN traffic.

The serial baud rate and CAN bitrate are independent:

```text
1,000,000 baud = PC to ATmega serial speed
500,000 bit/s  = vehicle CAN bus speed
```

---

## Building and Flashing

Install the Arduino CAN library by Sandeep Mistry:

```text
CAN by Sandeep Mistry
```

The firmware uses the following library interface:

```cpp
#include <CAN.h>
```

Select the correct ATmega328PB board definition and programmer in the Arduino IDE or compatible build environment.

Flash the firmware to the gauge using the normal ISP or bootloader programming method used for the AFR or Boost Controller project.

After flashing, the FT232RL should appear in Windows Device Manager as a COM port, for example:

```text
USB Serial Port (COM4)
```

---

## Basic Serial Test

Open the adapter COM port at 1,000,000 baud using a serial terminal.

Commands must be terminated with a carriage return.

Send:

```text
V
```

Expected response:

```text
V0101
```

Send:

```text
N
```

Expected response:

```text
N0001
```

Initialize the CAN channel:

```text
C
S6
O
```

`S6` selects 500 kbit/s.

A successful command returns a carriage return. Depending on the serial terminal, this may appear as an empty line.

To transmit a test frame:

```text
t12321122
```

This represents:

```text
CAN ID: 0x123
DLC:    2
Data:   11 22
```

---

## Using with python-can

Install the required Python packages:

```powershell
python -m pip install python-can pyserial
```

Example receive test:

```python
import can
import time

PORT = "COM4"
SERIAL_BAUD = 1_000_000
CAN_BITRATE = 500_000

bus = can.Bus(
    interface="slcan",
    channel=f"{PORT}@{SERIAL_BAUD}",
    bitrate=CAN_BITRATE,
)

print(f"Opened: {bus.channel_info}")
print("Listening for CAN frames...")

try:
    end_time = time.monotonic() + 10

    while time.monotonic() < end_time:
        message = bus.recv(timeout=1.0)

        if message is not None:
            print(
                f"ID=0x{message.arbitration_id:X} "
                f"DLC={message.dlc} "
                f"DATA={message.data.hex(' ')}"
            )
finally:
    bus.shutdown()
```

Replace `COM4` with the actual COM port assigned to the FT232RL.

---

## Using with GKFlasher

GKFlasher's CAN hardware backend must use the `slcan` backend instead of the default PCAN backend.

The CAN socket should be opened similarly to:

```python
self.socket = CANSocket(
    bustype="slcan",
    channel=f"{self.port}@1000000",
    bitrate=self.bitrate,
)
```

Then start GKFlasher with the gauge's COM port:

```powershell
python gkflasher.py --protocol canbus --interface COMyournumber
```

For a 500 kbit/s target bus, GKFlasher or `python-can` will normally initialize the adapter with:

```text
C
S6
O
```

The firmware then translates SLCAN serial commands into MCP25625 CAN frames and returns received CAN frames through the FT232RL.

---

## Wiring and Connection Notes

Before connecting to a vehicle, verify:

- CAN-H and CAN-L are not reversed
- the gauge and vehicle share a common ground
- the MCP25625 is not held in standby
- the CAN bitrate matches the target network
- the MCP25625 oscillator setting is correct
- the gauge does not add an unwanted 120 ohm termination resistor to an already terminated vehicle bus

For an OBD-II connector, high-speed CAN is commonly present on:

```text
Pin 6  - CAN-H
Pin 14 - CAN-L
Pin 4/5 - Ground
```

Confirm the pinout for the specific vehicle before connecting.

---

## Limitations

This firmware is intended as a convenient temporary interface, not as a full replacement for a dedicated professional CAN adapter.

Current limitations include:

- Classical CAN only
- Maximum payload of 8 bytes
- No CAN FD support
- No advanced hardware acceptance-filter configuration through SLCAN
- Listen-only command is treated as a normal CAN open request
- Custom `BTR0/BTR1` timing configuration is not implemented
- Receive performance is limited by the MCP25625 receive buffers, ATmega processing speed, and ASCII serial framing
- A heavily loaded CAN network may overflow the receive queue

For GKFlasher and normal diagnostic request/response traffic, these limitations are generally acceptable.

---

## Safety Warning

ECU flashing carries a risk of rendering the ECU inoperable if communication or power is interrupted.

Before flashing:

- use a stable vehicle battery supply
- verify CAN communication before starting
- confirm the correct ECU and protocol
- avoid loose USB, CAN, or power connections
- do not disconnect the interface during erase or programming
- keep a recovery method available when possible

This firmware and hardware are provided without warranty. Use them at your own risk.

---

## Restoring the Gauge

To return the board to normal operation, reflash the original firmware for the applicable project:

- OpenAFR AFR Gauge firmware, or
- Boost Controller Gauge firmware

No hardware modification is required when the same PCB already contains the FT232RL, ATmega328PB, and MCP25625.

---

## License

See the repository root for license information.
