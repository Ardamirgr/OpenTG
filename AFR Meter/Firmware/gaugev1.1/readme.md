# OpenTG Wideband Gauge Display Firmware

Firmware for an AVR-based **TFT gauge display** that pairs with a wideband controller over **CAN bus**.  
The display provides a large numeric readout (AFR or Lambda), sensor temperature, LED indicator bar, optional UART logging, and an on-device configuration menu stored in EEPROM.

This code is designed to work with a wideband controller that transmits data on **CAN ID 0xFE** and receives configuration on **CAN ID 0xFF**.

---

## Features

- **172x320 ST7789 TFT UI** (DFRobot GDL library)
  - Main screen: sensor temperature + mode marker + large numeric value
  - Boot splash (OpenTG logo)
  - Error screens with warning icon and user prompts
- **CAN communication**
  - Receives live values (status flags, lambda, AFR, temperature, battery)
  - Sends configuration bytes (emulation mode + debug flag) to controller
- **Menu system (2 buttons)**
  - Display mode: AFR / Lambda / Optimal
  - Logging enable/disable + emulation selection
  - LED behavior selection
  - Backlight levels including auto mode (photo sensor)
  - Persistent storage via EEPROM
- **LED indicator output**
  - 8 LED outputs driven active-low
  - Modes: single LED / bar graph / off
- **Backlight PWM**
  - Levels: High / Medium / Low / Auto
  - Auto reads photo sensor and updates brightness
- **Optional UART logging**
  - Streams lambda + temperature (or error string) over UART TX
- **Watchdog safety**
  - Watchdog enabled (4s) and fed in main loop and blocking screens

---

## Hardware

### Display
- **DFRobot ST7789 172x320 TFT** via **hardware SPI**
- Pins are expected to be defined in `gauge.h`:
  - `TFT_DC`, `TFT_CS`, `TFT_RST`

### CAN
- Uses Arduino CAN library
- `CAN.setPins(10, 2)` in `setup`

### Buttons
- Uses direct port reads:
  - Button 1: `PIND3` (LOW when pressed)
  - Button 2: `PIND5` (LOW when pressed)
- Button 1 also functions as a fast way to enter the main menu from `loop`

### LEDs
- 8 output pins listed in `ledPins`:
  - `{14, 15, 16, 17, 18, 19, 25, 7}`
- Implementation writes directly to PORTC / PORTE / PORTD (active-low)

### Backlight
- PWM on Timer0 OC0A (`OCR0A`)
- A photo sensor ADC channel is used for Auto brightness:
  - `PHOTO_SENSOR` (defined in `gauge.h`)

---

## Repository Layout Expectations

This firmware references project headers/assets:

- `gauge.h`
  - pin definitions, constants like `BAUDRATE`, `MAX_RETRIES`, `AFR_X`, `AFR_Y`, `CHAR_W`, ADC channel defines, and status bits
- `icons.h`
  - XBM bitmaps and dimensions (logo, thermometer, etc.)

---

## Boot Sequence

On boot, `setup` performs:

1. **GPIO, ADC, timers, UART init**
2. **EEPROM initialization** (first boot flag at address 5)
3. Load user settings from EEPROM:
   - `disp_out_mode`, `log_flag`, `led_mode`, `flagEMU`, `bl_level`
4. Enable watchdog (4s)
5. Apply backlight level
6. Initialize TFT and clear screen
7. Initialize CAN (1 Mbps)  
   - If CAN init fails → `errorCAN` (blocking error screen)
8. Register CAN receive callback `packetReceive`
9. Send configuration to controller: `sendCAN(flagEMU)`
10. Show OpenTG splash `drawOpenTG`
11. Wait until engine is ON:
    - `while !(status_flags & STATUS_ENGINE_ON)`
12. LED sweep animation
13. Draw main screen `drawMainScreen`

---

## Runtime Behavior

### Main loop

Each `loop` iteration:

- If `STATUS_RECEIVE` is set → clears `com_flag` (prevents repeated config TX)
- If `com_flag == 1` → sends config to controller (`sendCAN(flagEMU)`)
- If `bl_tick == 1` and `bl_level == 3` → update auto-backlight (`setBacklightLevel`)
- If Button 1 pressed → enter menu (`main_Menu`)
- Check status flags for faults:
  - missing sensor → `errorCJ125(ERR_SENS)`
  - short circuit → `errorCJ125(ERR_SHRT)`
  - low battery → `errorCJ125(ERR_BAT)`
- If in normal operation:
  - if `sens_temp > 1030` → `errorTemp`
  - update LEDs via `UpdateLEDS`
  - update UI via `UpdateScreen`
  - if logging enabled → UART `logData` for lambda and temperature
- If in heat mode (`STATUS_HEAT`) → shows “Heat”
- Feed watchdog

---

## CAN Protocol

### RX frame (controller → display)
- **CAN ID:** `0xFE`
- **DLC:** 8

| Byte | Meaning |
|------|---------|
| 0 | `status_flags` |
| 1 | lambda high byte |
| 2 | lambda low byte |
| 3 | AFR integer |
| 4 | AFR decimal (0–99) |
| 5 | sensor temp high byte |
| 6 | sensor temp low byte |
| 7 | battery ADC byte |

On receipt, the display parses and stores:
- `status_flags`, `lambda_value`, `afr_int`, `afr_decimal`, `sens_temp`, `vbat`

### TX frame (display → controller)
- **CAN ID:** `0xFF`
- **DLC:** 2

| Byte | Meaning |
|------|---------|
| 0 | `flagEMU` (0 = wideband emu, 1 = narrowband emu) |
| 1 | `flagDebug` (0/1) |

The display sends this periodically until it detects a receive frame (`STATUS_RECEIVE` logic) to avoid spamming.

---

## Display Modes

`disp_out_mode`:

- `0` = AFR
- `1` = Lambda
- `2` = Optimal (this mode also forces `flagDebug = 1` so the controller can switch to an alternate debug behavior)

`UpdateScreen` uses a fixed-width, “only redraw changed characters” approach for efficient TFT updates.

---

## Backlight Modes

`bl_level`:

- `0` = High (`OCR0A = 255`)
- `1` = Medium (`OCR0A = 170`)
- `2` = Low (`OCR0A = 85`)
- `3` = Auto  
  - reads photo sensor ADC and maps to PWM
  - Timer1 ISR sets `bl_tick = 1` to trigger periodic refresh

---

## LED Modes

`led_mode`:

- `0` = single LED at AFR index
- `1` = bar up to AFR index
- other = all off

AFR is mapped roughly from **11..18** to an LED index **0..7** (clamped).

---

## Menu System

Menu navigation uses two buttons:

- Button 2: next option
- Button 1: select option

Menus:

- Main Menu
  - Display
  - Logging & Emu.
  - LED Menu
  - Exit
- Display Menu
  - AFR
  - Lambda
  - Optimal
  - Backlight settings
- Logging & Emu.
  - Logging enable/disable
  - Emulation mode (narrowband / wideband)
- LED Menu
  - Single / Multi / Disabled
- Backlight Menu
  - High / Medium / Low / Auto

Settings are stored to EEPROM using `EEPROM.update`.

---

## UART Logging

If `log_flag == 1`, the firmware prints values over UART TX:

- `logData(lambda_value)`
- `logData(sens_temp)`

If an error flag is present (sensor missing/short/low battery), it prints a fixed error string instead.

---

## Building / Flashing

1. Install libraries:
   - `DFRobot_GDL`
   - Arduino `CAN`
   - Arduino `EEPROM`
   - **Make sure to use only the libraries that are provided in this repo.**

2. Ensure `gauge.h` matches your pinout and defines:
   - TFT pins, ADC channels, CAN retry count, baudrate constants, screen layout constants

3. Compile and flash using Arduino IDE / PlatformIO. You will need an Arduino or similar to act as an ICSP.

---

## License

See the repository root for license information.
