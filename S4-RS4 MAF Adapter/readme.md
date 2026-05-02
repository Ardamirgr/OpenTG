# Audi S4/RS4 MAF Sensor Adapter Board

PCB files for an adapter board compatible with the following Bosch MAF sensors:

- Bosch `0280218038`
- Bosch `028021807D`
- Bosch `0280218067`
- Bosch `0986280219`
- Bosch `028021808P`

## Notes

### TVS and Reverse-Polarity Protection

The TVS diode and reverse-polarity diode are optional. They were included for extra protection, but they are not required for normal operation.

If you choose not to install the reverse-polarity diode, short the `D1` pads.

### Chassis Ground Jumper

Leave the jumper between **Chassis Ground** and the **ground pour** open.

This jumper is included only for debugging. Closing it shorts chassis ground to sensor ground and may negatively affect signal integrity for all ECU sensors.
