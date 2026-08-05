# LTM43 — Live Tuning Module for the Siemens SIMK43 ECU

LTM43 is a hardware module that enables live calibration changes on Siemens SIMK43 engine control units.

The module replaces the original flash memory fitted to the ECU. A small pin-header adapter PCB is soldered directly onto the original flash IC footprint, and the LTM43 plugs into this adapter through female headers.

> [!WARNING]
> This project requires fine-pitch soldering directly onto an automotive ECU PCB. Incorrect assembly, wiring, flashing, or use may permanently damage the ECU or prevent the engine from starting. Use the module only if you are familiar with ECU repair, boot-mode flashing, and automotive electrical systems.

## How It Works

The LTM43 contains:

- A **1 Mbit flash memory**, providing twice the capacity of the original 512 Kbit device.
- A **512 Kbit SRAM**, connected in parallel with the flash memory.

When the ECU starts in the required debug mode, the calibration data is copied from flash into SRAM. The ECU then operates from the SRAM copy, allowing calibration data to be modified while the ECU is running.

During shutdown, the modified SRAM contents are copied back into flash. The updated calibration is therefore retained and restored at the next startup.

This follows the live-tuning method used by Siemens during ECU and engine development.

## Hardware Overview

The system consists of two PCBs:

1. **Flash adapter board**  
   Soldered directly to the original flash IC pads on the SIMK43 ECU.

2. **LTM43 module**  
   Plugs into the adapter board through female pin headers and contains the replacement flash and SRAM devices.

## SRAM Options

The LTM43 PCB includes alternative SRAM footprints to support different packages. During assembly, populate **only one SRAM option**.

Supported devices are:

- Infineon `CY62128ELL-45ZXI`
- Infineon `CY62128ELL-45SXA`
- Renesas `R1LP0108ESN-5SI#B1`

## Board Assembly

The pin-header adapter board must sit completely flush against the ECU PCB so that its castellated edges can be soldered correctly to the original flash IC pads.

After installing and trimming the pin headers:

1. Cut the protruding header pins as close to the adapter PCB as practical.
2. Carefully sand the remaining pin ends until they are level with the underside of the PCB.
3. Confirm that no pin end prevents the adapter board from sitting flat.

For the LTM43 module, just solder the IC's and 100nF capacitors at their respective footprints. 
Remeber that the female headers should be placed on the bottom PCB side! 

> [!CAUTION]
> Do not force the adapter board flat while soldering. Any remaining pin protrusion can place mechanical stress on the ECU pads and may cause pad lifting or unreliable connections.

## Installation

1. Remove the original flash IC from the SIMK43 ECU.
2. Clean and inspect the exposed flash footprint.
3. Assemble the pin-header adapter as described above.
4. Align the castellated pads with the original flash footprint.
5. Check orientation and pin numbering before soldering (Pin#1 should be at the side of the C167 MCU).
6. Solder the adapter board to the ECU flash pads.
7. Solder a jumper wire from C167 Pin 88 (P4.3/A19 ) to the Adapter Board pin #3. This is best done from a via present on the bottom side of the ECU, that connects to Pin 88. 
8. Inspect every connection for bridges, incomplete joints, lifted pads, or misalignment.
9. Insert the LTM43 to the pin headers of the adapter board, observing the correct orientation (LTM43 oriented towards C167 MCU).
10. Perform an initial BSL flash to program the replacement flash memory.

## Initial BSL Flash

A blank replacement flash device must first be programmed through the SIMK43 boot-strapped loader, or **BSL**, interface.

The required firmware is:

```text
ca663057
```

It is available from the OpenGK SIMK repository:

- [OpenGK SIMK firmware repository](https://github.com/OpenGK-org/opengk-simk)

GKFlasher can be used to BSL flash and it is available here:

- [GKFlasher releases](https://github.com/Dante383/GKFlasher/releases)

### BSL Connection Method 1

1. Connect ECU pins **3, 14, 21, and 22** to **+12 V**.
2. Connect the diagnostic **K-Line** to ECU pin **77**, according to the ECU pinout.
3. Fit a **470 Ω resistor** to the ECU **W-Line**.
4. Splice the K-Line into the K-Line connection on the ECU header. When using a Y-split OBD2 cable, this is pin **9**.
5. Pull the **BOOT** pin to ground through a **10 kΩ resistor**.
6. Connect ECU power and ground.
7. Open the **BSL** tab in GKFlasher and begin the required operation.

### Alternative BSL Connection Method

Use this method when the first connection method does not work:

1. Connect ECU pins **3, 14, 21, and 22** to **+12 V**.
2. Connect OBD pin **7**, the K-Line, to ECU header pin **47**, the W-Line.
3. Pull flash-memory pin **28**, the BOOT pin, to ground through a **10 kΩ resistor**.
4. Connect ECU power and ground.
5. Open the **BSL** tab in GKFlasher and begin the required operation.

> [!IMPORTANT]
> Verify all ECU pin numbers against the correct SIMK43 connector and PCB documentation before applying power. Use a current-limited, fused 12 V supply for bench flashing.

## CAN Bus Communication

After the initial BSL flash, subsequent communication and calibration changes can be performed through CAN bus.

Supported interfaces include:

- A PCAN-compatible interface
- The OpenTG SLCAN Adapter

SLCAN Adapter project:

- [OpenTG SLCAN Adapter](https://github.com/Ardamirgr/OpenTG/tree/main/SLCAN_Adapter)

### CAN Wiring

Connect:

| Signal | SIMK43 ECU pin |
|---|---:|
| CAN High | 7 |
| CAN Low | 6 |
| Ground | ECU/interface shared ground |

A shared ground between the ECU and CAN interface is required.

### SLCAN Adapter Power

The SLCAN Adapter requires a regulated **5V supply**. It can be powered by either:

- The USB header, when the corresponding power-selection jumper is shorted.
- An external regulated 5V source, such as a suitable ECU-side supply.

Do not apply 12V directly to a 5V input.

## Recommended First Power-Up Procedure

Before connecting the ECU to a vehicle:

1. Inspect the adapter and LTM43 for solder bridges and incorrect component orientation.
2. Check resistance between all supply rails and ground for obvious short circuits.
3. Verify continuity from every ECU flash pad to the corresponding LTM43 signal.
5. Power the ECU from a fused, current-limited bench supply.
6. Perform the BSL flash using the `ca663057` firmware.
7. Power-cycle the ECU and verify CAN communication.
8. Confirm that calibration reads and writes operate correctly before vehicle installation.

## Removal and Serviceability

The LTM43 is removable after the flash-footprint adapter has been installed. This allows the module to be replaced or serviced without repeatedly soldering directly to the ECU flash pads.

Always disconnect ECU power before inserting or removing the LTM43.

## Disclaimer

This project is intended for development, research, motorsport, and off-road applications by experienced users.

The project contributors accept no responsibility for:

- ECU, engine, wiring, or vehicle damage
- Data corruption or failed flashing
- Unsafe engine calibration
- Emissions or road-legality violations
- Injury or property damage resulting from assembly or use

You are responsible for verifying the hardware, firmware, wiring, calibration, and legal suitability of the finished system.

## Related Projects

- [OpenGK SIMK](https://github.com/OpenGK-org/opengk-simk)
- [GKFlasher](https://github.com/Dante383/GKFlasher/releases)
- [OpenTG SLCAN Adapter](https://github.com/Ardamirgr/OpenTG/tree/main/SLCAN_Adapter)

