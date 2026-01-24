//Current consumption w/330R = 386mA (121mA LEDs)
//Current consumption w/680R = 323mA (58mA LEDs)
//Current consumption w/CUST = 318mA (53mA LEDs)

#include "DFRobot_GDL.h"
#include "CAN.h"
#include "EEPROM.h"
#include "gauge.h"
#include "icons.h"
#include "avr/wdt.h"
#include "avr/io.h"
#include "avr/interrupt.h"

uint16_t timer = 11;
uint8_t redraw_afr = 0;

const uint8_t ind_Ypos[4][3] = {
                              {/*y0=*/53,/*y1=*/67,/*y2=*/60},
                              {/*y0=*/83,/*y1=*/97,/*y2=*/90},
                              {/*y0=*/113,/*y1=*/127,/*y2=*/120},
                              {/*y0=*/143,/*y1=*/157,/*y2=*/150}
                              };

const uint8_t ledPins[8] = {14,15,16,17,18,19,25,7};                                                

//uint8_t menu_state = 0;
uint8_t buttonState1 = 0;
uint8_t buttonState2 = 0;
uint8_t selected_option = 0;
uint8_t com_flag = 1;

uint8_t disp_out_mode;
uint8_t log_flag;
uint8_t led_mode;
uint8_t flagEMU;
uint8_t flagDebug = 0;

uint8_t bl_level, bl_tick;
uint8_t status_flags = 0;
uint16_t die_counter = 0;

uint8_t  vbat = 200;
uint16_t sens_temp, lambda_value;
uint16_t afr_int, afr_decimal;

enum MenuState : uint8_t {
    MAIN_MENU = 0,
    DISP_MENU,
    LOG_MENU,
    LED_MENU,
    BCKLIGHT_MENU
};

enum : uint8_t {
    DISP_OPT = 4,
    LOG_OPT = 4,
    LED_OPT = 3,
    BCKLIGHT_OPT = 4,
    MAIN_OPT = 4
};


MenuState menu_state = MAIN_MENU;


/**
 * @brief Constructor of hardware SPI communication
 * @param dc Command/data line pin for SPI communication
 * @param cs Chip select pin for SPI communication
 * @param rst reset pin of the screen
 */

DFRobot_ST7789_172x320_HW_SPI screen(/*dc=*/TFT_DC,/*cs=*/TFT_CS,/*rst=*/TFT_RST);


/*
 *User-selectable macro definition color
 *COLOR_RGB565_BLACK   COLOR_RGB565_NAVY    COLOR_RGB565_DGREEN   COLOR_RGB565_DCYAN 
 *COLOR_RGB565_MAROON  COLOR_RGB565_PURPLE  COLOR_RGB565_OLIVE    COLOR_RGB565_LGRAY     
 *COLOR_RGB565_DGRAY   COLOR_RGB565_BLUE    COLOR_RGB565_GREEN    COLOR_RGB565_CYAN  
 *COLOR_RGB565_RED     COLOR_RGB565_MAGENTA COLOR_RGB565_YELLOW   COLOR_RGB565_ORANGE           
 *COLOR_RGB565_WHITE   
 */
 
void setup() {
  
  CAN.setPins(10, 2);

  /******************
  *    GPIO SETUP   *
  ******************/
  DDRC  |= 0b00111111;                        // GRN1 to RED1
  DDRE  |= (1 << DDE2);                       // RED2
  DDRD  |= 0b11010000;                        // RED3, BACKLIGHT, CAN_STANDBY, as OUTPUT    
  
  
  // set pins high (LEDs off)
  PORTD |= (1 << PORTD2);               // Set internal pullup for Interrupt
  PORTC |= 0b00111111;                  // PC1..PC5 HIGH
  PORTE |= (1 << PE2);                  // PE2 HIGH
  PORTD |= (1 << PD7);                  // PD6, PD7 HIGH  

  /******************
  *    ADC SETUP    *
  ******************/
  ADMUX  = (1 << REFS0); // REFS1=0, REFS0=1 -> AVCC 
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler = 128 (16 MHz / 128 = 125 kHz ADC clock)

  /******************
  * TIMER PWM SETUP *
  ******************/
  TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); // fast PWM, non-inverting on OC0A
  TCCR0B = (1 << CS00);  //  WGM02 = 0, prescaler = 10

  /******************
  * TIMER BACKLIGHT SETUP *
  ******************/
  TCCR1A = 0;                          // normal port operation
  TCCR1B = 0;

  OCR1A  = 2499;                       // 10ms @ 16MHz with /64 prescaler
  TCCR1B |= (1 << WGM12);              // CTC mode (clear timer on compare match)
  TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64
  TIMSK1 |= (1 << OCIE1A);             // enable compare A interrupt

  /******************
  *   UART SETUP    *
  ******************/ 
  UBRR0H = (uint8_t)(BAUDRATE >> 8); // set baudrate
  UBRR0L = (uint8_t) BAUDRATE;  
  UCSR0B = (1 << TXEN0); // Enable TX 
  // Frame format: 8 data bits, 1 stop bit, no parity
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // UCSZ = 3 -> 8-bit
  
  /******************
  * BOOT SEQUENCE   *
  ******************/
  if (EEPROM.read(5) != 1) {
      EEPROM.write(0,0);
      EEPROM.write(1,0);
      EEPROM.write(2,0);
      EEPROM.write(3,0);
      EEPROM.write(4,0);
      EEPROM.write(5,1);
  }
  // Read EEPROM to get user setup
  disp_out_mode = EEPROM.read(0);
  log_flag      = EEPROM.read(1);
  led_mode      = EEPROM.read(2);
  flagEMU       = EEPROM.read(3);
  bl_level      = EEPROM.read(4);
  bl_tick       = 0;

  wdt_enable(WDTO_4S);   
  
  menu_state = MAIN_MENU;  
  if (bl_level > 3) bl_level = 0;
  setBacklightLevel(bl_level); 
  
  screen.begin(8000000);
  screen.setRotation(1);
   // Set text wrapping mode
   // true = Text word wrap, false = No word wrap
  screen.setTextWrap(true);
  //Fill color, RGB color with 565 structure
  screen.fillScreen(COLOR_RGB565_BLACK);

  if (!CAN.begin(1000000)) {
    errorCAN();
  }  

  CAN.onReceive(packetReceive);
   
  sendCAN(flagEMU);
  drawOpenGK(93,3);

  while (!(status_flags & STATUS_ENGINE_ON)) {
    delay_ms(100);
   wdt_reset();
 }

  for (int i = 0; i <= 7; i++) {
    digitalWrite(ledPins[i], LOW);
    delay_ms(375);
    wdt_reset();
  } 
    
  drawMainScreen();
  sens_temp = 0; 
}

void loop(){
    
  if (status_flags & STATUS_RECEIVE) com_flag = 0;
  if (com_flag == 1) sendCAN(flagEMU);

  if (bl_tick == 1 && bl_level == 3) {
  setBacklightLevel(bl_level);
  }
  
  uint8_t btn1 = (PIND & (1 << PIND3)) ? 1 : 0;
  if (btn1 == 0) {    
    btn1 = 1;
    menu_state = MAIN_MENU;
    main_Menu();
  }
  
  if (!(status_flags & STATUS_SENSOR))  errorCJ125(ERR_SENS);
  if (status_flags & STATUS_SHORT_CIRC) errorCJ125(ERR_SHRT);
  if (status_flags & STATUS_LOW_BAT)    errorCJ125(ERR_BAT);
  
  if (status_flags & STATUS_NORMAL) {  
    if (sens_temp > 1030) errorTemp();   

    /*afr = ((uint32_t)lambda_value * 147);
    afr = afr/100;
  
    uint16_t afr_int = afr / 100;      // Get the part before the decimal
    uint16_t afr_decimal = afr % 100;    // Get the part after the decimal*/
  
    UpdateLEDS(afr_int, led_mode);
    UpdateScreen();
    
    if (log_flag == 1) {
      logData(lambda_value);
      logData(sens_temp);
    }
  }
  
  if (status_flags & STATUS_HEAT) {

    screen.setTextColor(COLOR_RGB565_ORANGE, COLOR_RGB565_BLACK);
    screen.setTextSize(10);
    screen.setCursor(75, 100);
    screen.print("Heat");
  }
  

  cli();
  wdt_reset();
  sei();
}

/**************************************************************************************
 * Updates the display with the current oil pressure value, properly formatted        *
 * and colored based on thresholds. Also handles error conditions.                    *
 **************************************************************************************/
void UpdateScreen() {
  static uint8_t prev_mode = 0xFF;

  // per-mode previous strings (so switching modes doesn't confuse diffs)
  static char afr_prev[6]    = "     ";
  static char lambda_prev[6] = "     ";

  static uint16_t color_prev = 0;

  // --- Temperature (keep your existing logic) ---
  // NOTE: your timer is "calls to UpdateScreen", not time. If you later throttle UI, this becomes sane.
  if (timer > 50) {
    screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK);
    screen.setTextSize(3);
    screen.setCursor(110, 45);

    // crude fixed-width-ish overwrite (keeps your behavior)
    if (sens_temp < 1000) screen.print(" ");
    screen.print(sens_temp);

    timer = 0;
  }

  // --- Determine AFR-based color ---
  uint16_t color;
  if (afr_int > 15)      color = COLOR_RGB565_RED;
  else if (afr_int > 13) color = COLOR_RGB565_YELLOW;
  else                   color = COLOR_RGB565_GREEN;

  // force redraw if color changed
  if (color != color_prev) {
    redraw_afr = 1;
    color_prev = color;
  }

  // force redraw if display mode changed (also reset prev text so diffs don't "skip")
  if (disp_out_mode != prev_mode) {
    prev_mode = disp_out_mode;
    redraw_afr = 1;
    afr_prev[0]=afr_prev[1]=afr_prev[2]=afr_prev[3]=afr_prev[4]=' ';
    lambda_prev[0]=lambda_prev[1]=lambda_prev[2]=lambda_prev[3]=lambda_prev[4]=' ';
  }

  // --- Main value area ---
  switch (disp_out_mode) {

    default: { // Mode 0 = AFR
      char cur[6];
      // afr_decimal is already 0..99 in your CAN protocol
      makeFixed5(cur, (uint8_t)afr_int, (uint8_t)afr_decimal);

      drawFixed5Cells(AFR_X, AFR_Y, cur, afr_prev,
                      color, COLOR_RGB565_BLACK,
                      redraw_afr);

      break;
    }

    case 1: { // Mode 1 = Lambda
      // Your existing scaling:
      // lambda_value -> lambda_disp = lambda_value/10
      // lambda_disp_int = lambda_disp/100
      // lambda_disp_dec = lambda_disp%100
      uint16_t lambda_disp = lambda_value / 10;
      uint8_t  lambda_disp_int = (uint8_t)(lambda_disp / 100);
      uint8_t  lambda_disp_dec = (uint8_t)(lambda_disp % 100);

      char cur[6];
      makeFixed5(cur, lambda_disp_int, lambda_disp_dec);

      drawFixed5Cells(AFR_X, AFR_Y, cur, lambda_prev,
                      color, COLOR_RGB565_BLACK,
                      redraw_afr);

      break;
    }
  }

  timer++;
  redraw_afr = 0;
}


/*void UpdateScreen() {
  static int8_t afr_int_prev = -1;
  static int8_t afr_dec_prev = -1;
  
  static int8_t lambda_int_prev = -1;
  static int8_t lambda_dec_prev = -1;
  
  static int8_t o2_int_prev = -1;
  static int8_t o2_dec_prev = -1;

  static int color_prev = 0;
  
  int color;

  // Only print temperature once every 50 timer ticks
  if (timer > 50) {
    screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK); 
     screen.setTextSize(3);
     screen.setCursor(110, 45);
    if (sens_temp < 1000) screen.print(" ");    
      screen.print(sens_temp);
      timer = 0;
    
  }
  
  // Determine AFR color
  if (afr_int > 15) color = COLOR_RGB565_RED;
  else if (afr_int > 13) color = COLOR_RGB565_YELLOW;
  else color = COLOR_RGB565_GREEN;

  if (color_prev != color) redraw_afr = 1;
  color_prev = color;

  screen.setTextColor(color, COLOR_RGB565_BLACK);
  screen.setTextSize(10);  

  switch (disp_out_mode) {
    default: // Mode = AFR
      if ((afr_int != afr_int_prev) || redraw_afr == 1) {

        screen.setCursor(75, 100);
        screen.print(afr_int);
        // Print decimal point only when whole number changes
        screen.print(".");
        
        if (afr_int < 10) {
          screen.setCursor(190, 100);
          if (afr_decimal < 10) screen.print("0");
          screen.print(afr_decimal);
        } 
        else {
          screen.setCursor(250, 100);
          screen.print(afr_decimal / 10);
        }
      }
      
      // Update decimal part if changed
      else if (afr_decimal != afr_dec_prev) {
        screen.setTextColor(color, COLOR_RGB565_BLACK);
        screen.setTextSize(10);
        if (afr_int < 10) {
          screen.setCursor(190, 100);
          if (afr_decimal < 10) screen.print("0");
          screen.print(afr_decimal);
        } else {
          screen.setCursor(250, 100);
          screen.print(afr_decimal / 10);
        }
      
      }
      afr_int_prev = afr_int;
      afr_dec_prev = afr_decimal; 

      break; 
          
    case 1: // Mode = Lambda 
      uint16_t lambda_disp = lambda_value / 10;
      uint8_t  lambda_disp_int = lambda_disp / 100;
      uint8_t  lambda_disp_dec = lambda_disp % 100;

      if ((lambda_disp_int != lambda_int_prev) || redraw_afr == 1) {      
        screen.setCursor(75, 100);      
        screen.print(lambda_disp_int);
        // Print decimal point only when whole number changes
        screen.print(".");
        
        if (lambda_disp_dec < 10) screen.print("0");
        screen.print(lambda_disp_dec);
      }
      else if (lambda_disp_dec != lambda_dec_prev) {
          screen.setCursor(190, 100);
          if (lambda_disp_dec < 10) screen.print("0");
          screen.print(lambda_disp_dec);
      }

      lambda_int_prev = lambda_disp_int;
      lambda_dec_prev = lambda_disp_dec; 

      break;  
  }  
  timer++;
  redraw_afr = 0;
}
*/

void delay_ms(uint32_t ms) {
  ms = 1420 * ms;             // I have 0 idea why this number isn't even close to 16000 when running with a 16MHz clock
  for (ms; ms > 1; ms--) {
     asm("nop");
  }
}   

/**************************************************************************************
 * Handles menu navigation and selection using two buttons.                           *
 * - nextOptType: determines the behavior of selectNext() (e.g., option group/type)   *
 * - onSelect: callback function to run when an option is selected                    *
 **************************************************************************************/
 
void handleMenu(int nextOptType, void (*onSelect)()) {
  while (true) {
    // Read the state of the two buttons
    uint8_t btn1 = (PIND & (1 << PIND3)) ? 1 : 0;  // Typically the "select" button
    uint8_t btn2 = (PIND & (1 << PIND5)) ? 1 : 0;  // Typically the "next" button

    // If BUTTON2 is pressed (LOW), go to the next option
    if (btn2 == 0) {
      selectNext(nextOptType); // Advance to the next menu item
      delay_ms(100);              // Debounce delay to avoid bouncing/multiple triggers
    }

    // If BUTTON1 is pressed (LOW), select the current option
    if (btn1 == 0) {
      onSelect();              // Call the selection callback
      selected_option = 0;     // Reset selected option to the beginning
      drawMenu();              // Redraw the menu (likely returns to top level)
      delay_ms(100);              // Longer debounce + pause for UI feedback
      break;                   // Exit the loop after selection
    }

    delay_ms(100); // Polling interval (and debounce) when idle
    cli();
    wdt_reset();
    sei();
  }
}


/**************************************************************************************
 * Handles input for navigating the main menu.                                        *
 * Responds to two buttons: one for moving through options, and one for selecting.    *
 * Exits the loop when menu_state changes from MAIN_MENU.                             *
 **************************************************************************************/
 
void handleMainMenu() { 
  while (menu_state == MAIN_MENU) {
    // Read button states
    uint8_t btn1 = (PIND & (1 << PIND3)) ? 1 : 0;  // Typically the "select" button
    uint8_t btn2 = (PIND & (1 << PIND5)) ? 1 : 0;  // Typically the "next" button

    // If BUTTON2 is pressed (LOW), move to the next main menu option
    if (btn2 == 0) {
      selectNext(MAIN_OPT);  // Cycle to next option in main menu
      delay_ms(100);            // Debounce delay
    }

    // If BUTTON1 is pressed (LOW), select the current option
    if (btn1 == 0) {
      menu_state = selected_option + 1;  // Set new menu state based on selected option index
      selected_option = 0;              // Reset option index for next menu

      if (menu_state == 4) { // Option 4 is "Exit"
        menu_state = MAIN_MENU;  // Stay in main menu
        return;                  // Exit this function (no redraw needed)
      } else {
        drawMenu();   // Draw the next menu screen based on new state
        delay_ms(100);   // Debounce and give user visual confirmation
      }
    }

    delay_ms(100); // Polling delay when idle (avoids excessive CPU usage)
    cli();
    wdt_reset();
    sei();
  }
}

/**************************************************************************************
 * Main menu controller function                                                      *
 * Draws the main menu, waits for user interaction, and handles submenu logic         *
 **************************************************************************************/
 
void main_Menu() {
  // Set text color (white text on black background)
  screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK);
  drawMenu();       // Draw initial main menu
  delay_ms(100);       // Allow time for screen refresh before input

  bool menu_exit = false;  // Flag to exit the menu loop

  while (!menu_exit) {
    handleMainMenu();  // Wait for user input to change menu_state

    switch (menu_state) {

      // --- Display Settings Menu ---
      case DISP_MENU:
        handleMenu(DISP_OPT, []() {
          if (selected_option < 3) {  // Options: AFR, Lambda, O2%
            disp_out_mode = selected_option;
            if (disp_out_mode == 2) {
              flagDebug = 1; 
              com_flag = 1;
            }
            else flagDebug = 0;
            EEPROM.update(0, disp_out_mode);  // Save to EEPROM            
            menu_state = MAIN_MENU;           // Return to main menu
          } else {
            // Option 3: Backlight Settings
            menu_state = BCKLIGHT_MENU;
          }
        });
        break;

      // --- Logging Options Menu ---
      case LOG_MENU:
        handleMenu(LOG_OPT, []() {
          if (selected_option < 2) {  // Options: Enable/Disable logging modes
            log_flag = selected_option;
            EEPROM.update(1, log_flag);  // Save to EEPROM
            menu_state = MAIN_MENU;
          } else {
            // Option 2+: EMU flag settings
            if (selected_option == 2) flagEMU = 1;
            else flagEMU = 0;
            EEPROM.update(3, flagEMU);
            com_flag = 1;               // Raise flag for CAN com.
            menu_state = MAIN_MENU;
          }
        });
        break;

      // --- LED Behavior Settings Menu ---
      case LED_MENU:
        handleMenu(LED_OPT, []() {
          led_mode = selected_option;
          EEPROM.update(2, led_mode);  // Save LED mode
          menu_state = MAIN_MENU;
        });
        break;

      // --- Backlight Settings Menu ---
      case BCKLIGHT_MENU:
        handleMenu(BCKLIGHT_OPT, []() {
          bl_level = selected_option;
          EEPROM.update(4, bl_level);       // Save backlight level
          setBacklightLevel(bl_level);      // Apply brightness
          menu_state = MAIN_MENU;
        });
        break;

      // --- Default / Exit Condition ---
      default:
        menu_exit = true;  // Exit menu loop
        break;
    }
  }

  drawMainScreen();  // Redraw the main display after exiting menu
  redraw_afr = 1;    // Trigger AFR value redraw on main screen
  return;
}


/**************************************************************************************
 * Draws the main display screen, showing the oil can graphic and the pressure        *
 * measurement unit.                                                                  *
 **************************************************************************************/
// Draws the main display screen, showing sensor temperature, logging status, and output mode (AFR, Lambda, or O2%)
void drawMainScreen() {
  // Clear the entire screen to black
  screen.fillScreen(COLOR_RGB565_BLACK);

  // Set text cursor for header title
  screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK);  
  screen.setTextSize(3);
  screen.setCursor(85, 6);
  screen.print("Sens. Temp");  // Header label: Sensor Temperature

  // Draw thermometer icon at the top left
  drawThermometerIcon(57, 0);

  // If logging is enabled, draw a folder/log icon
  if (log_flag == 1)
    drawFolderIcon(10, 138);

  screen.setCursor(290, 6);
  screen.setTextColor(COLOR_RGB565_GREEN, COLOR_RGB565_BLACK);  
  if (flagEMU == 1) screen.print("N");  // Header label: Sensor Temperature
  else screen.print("W");    
  
  // Display placeholder for temperature (e.g., "--- C")
  screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK); 
  screen.setCursor(120, 45);
  screen.print("--- C");

  // Draw a small rounded rectangle next to temperature, possibly as a degree symbol or icon
  screen.fillRoundRect(
    /*x0=*/185,    // X position
    /*y0=*/45,     // Y position
    /*w=*/5,       // Width
    /*h=*/5,       // Height
    /*radius=*/2,  // Corner radius
    /*color=*/COLOR_RGB565_WHITE  // Fill color
  );

  // Display the label for the selected display mode: AFR, Lambda, or O2%
  screen.setCursor(0, 108);
  screen.setTextSize(3);
  
   //  Display unit label
  switch (disp_out_mode) {
    case 0: screen.print("AFR:"); break;
    case 1: screen.print("LMD:"); break;
    case 2: screen.print("OPT:"); break;

  }
}

/**************************************************************************************
 * Draws the menu header, along with the relevant menu icon                           *
 **************************************************************************************/

void drawMenuHeader(const char* title, void (*iconFunc)()) {
  screen.fillScreen(COLOR_RGB565_BLACK);
  iconFunc();                               // Draw menu icon

  screen.setTextSize(3);
  screen.setCursor(100, 5); 
  screen.print(title);              // Print menu 
  
}

/**************************************************************************************
 * Draws the menu items, along with the cursor/selector (red triangle)                *
 **************************************************************************************/

void drawMenuItems(const char* items[], uint8_t itemCount) {
  screen.setCursor(35, 50);
  for (uint8_t i = 0; i < itemCount; i++) {
    if (i == 0) screen.print("");             // No indent for first item
    else screen.print("  ");
    screen.println(items[i]);
  }

  // Draw selector triangle
  screen.fillTriangle(10, 53, 10, 67, 25, 60, COLOR_RGB565_RED);
}

/**************************************************************************************
 * Draws the appropriate menu screen based on the current menu_state.                *
 **************************************************************************************/
 
void drawMenu() {
  switch (menu_state) {

    // --- Main Menu ---
    case MAIN_MENU: {
      const char* items[] = { "Display", "Logging & Emu.", "LED Menu", "Exit" };
      drawMenuHeader("Main Menu", drawMenuIcon); // Set title and icon
      drawMenuItems(items, 4);                   // Draw menu items
      break;
    }

    // --- Display Menu ---
    case DISP_MENU: {
      const char* items[] = { "A/F Ratio", "Lambda", "Optimal", "Bcklight Sett." };
      drawMenuHeader("Disp. Menu", drawDispIcon); // Set title and icon
      drawMenuItems(items, 4);                    // Draw menu items
      break;
    }

    // --- Logging & Emulation Menu ---
    case LOG_MENU: {
      const char* items[] = { "Log. Disabled", "Log. Enabled", "Emu. Nrbnd", "Emu. Wdbnd 0-1V" };
      drawMenuHeader("Log. & Emu.", [](){ drawFolderIcon(45, 1); }); // Draws a folder icon
      drawMenuItems(items, 4);
      break;
    }

    // --- LED Menu ---
    case LED_MENU: {
      const char* items[] = { "Single LED", "Multi LED", "Disabled" };
      drawMenuHeader(" LED Menu", drawLedIcon);  // Set title and icon
      drawMenuItems(items, 3);
      break;
    }

    // --- Backlight Level Menu ---
    case BCKLIGHT_MENU: {
      const char* items[] = {"High", "Medium", "Low", "Auto" };
      drawMenuHeader("Backlight", [](){ drawDispIcon(); }); // Temporary use of Disp icon
      drawMenuItems(items, 4);
      break;
    }
  }
}

/**************************************************************************************
 * Moves the selector to the next menu option and wraps around when at the end.       *
 **************************************************************************************/

void selectNext(uint8_t menu_options) {
  // Remove current selector (cursor) by drawing over it with background color
  drawSelector(selected_option, COLOR_RGB565_BLACK);

  // Increment selected option index, wrapping around to 0 if it exceeds the option count
  selected_option = (selected_option + 1) % menu_options;

  // Draw new selector (cursor) in red
  drawSelector(selected_option, COLOR_RGB565_RED);
}

/**************************************************************************************
 * Draws a triangle-shaped cursor beside the selected option                          *
 * "option" is the index of the menu item; "color" is the cursor color                *
 **************************************************************************************/

void drawSelector(uint8_t option, uint16_t color) {
  screen.fillTriangle(
    10, ind_Ypos[option][0],  // Top point (x, y)
    10, ind_Ypos[option][1],  // Bottom left point (x, y)
    25, ind_Ypos[option][2],  // Bottom right point (x, y)
    color                     // Fill color
  );
}

/***************************************************************************************
 * Took me far longer than it should have (~5 hours), but saving flash is king :')     *
 * Draws the OpenGK logo that appears for ONLY 3 seconds at boot.                      *
 * NEVER AND I MEAN EVER, TOUCH THIS. These offsets have been worked out with blood.   *
 **************************************************************************************/

void drawOpenGK(int offsetX, int offsetY) {
  
  screen.fillScreen(COLOR_RGB565_BLUE);

  // Made a bitmap and stored it in flash. Didn't go well. Flash hog, extremely slow, but a lot better looking. Leaving here for maybe other projects.
  // screen.drawXBitmap(/*x=*/offsetX,/*y=*/offsetY,/*bitmap gImage_Bitmap=*/gklogo,/*w=*/gklogo_width,/*h=*/gklogo_height,COLOR_RGB565_WHITE);

  // Draw layered round rect border
  for (int i = 0; i < 5; i++) {
    screen.drawRoundRect(offsetX + i, offsetY + i + 15, 135 - 2 * i, 135 - 2 * i, 20 - i, COLOR_RGB565_WHITE);
  }

  // Top and bottom pins
  for (int x = 15; x < 130; x += 15) {
    for (int dx = 0; dx <= 1; dx++) {
      screen.drawFastVLine(offsetX + x + dx, offsetY, 10, COLOR_RGB565_WHITE);        // Top
      screen.drawFastVLine(offsetX + x + dx, offsetY + 155, 10, COLOR_RGB565_WHITE);  // Bottom
    }
  }

  // Left and right pins
  for (int y = 30; y < 145; y += 15) {
    for (int dy = 0; dy <= 1; dy++) {
      screen.drawFastHLine(offsetX - 15, offsetY + y + dy, 10, COLOR_RGB565_WHITE);       // Left
      screen.drawFastHLine(offsetX + 140, offsetY + y + dy, 10, COLOR_RGB565_WHITE);      // Right
    }
  }

  // Inner "T" shape cutout — top bar
  for (int x = 25, i = 0; x < 35; x++, i += 2)
    screen.drawFastVLine(offsetX + x, offsetY + 30, 45 + i, COLOR_RGB565_WHITE);

  for (int x = 35, i = 0; x < 45; x++, i++)
    screen.drawFastVLine(offsetX + x, offsetY + 30, 25 - i, COLOR_RGB565_WHITE);

  for (int x = 45; x < 59; x++)
    screen.drawFastVLine(offsetX + x, offsetY + 30, 15, COLOR_RGB565_WHITE);

  for (int x = 59; x < 77; x++)
    screen.drawFastVLine(offsetX + x, offsetY + 30, 95, COLOR_RGB565_WHITE);

  for (int x = 77; x < 90; x++)
    screen.drawFastVLine(offsetX + x, offsetY + 30, 15, COLOR_RGB565_WHITE);

  for (int x = 90, i = 0; x < 100; x++, i++)
    screen.drawFastVLine(offsetX + x, offsetY + 30, 15 + i, COLOR_RGB565_WHITE);

  for (int x = 100, i = 0; x < 110; x++, i += 2)
    screen.drawFastVLine(offsetX + x, offsetY + 30, 63 - i, COLOR_RGB565_WHITE);

  // Vertical stem of the "T"
  for (int x = 63, i = 0; x > 40; x--, i += 2)
    screen.drawFastHLine(offsetX + x, offsetY + 136 - i / 2, 10 + i, COLOR_RGB565_WHITE);

  for (int x = 52, i = 0; x < 60; x++, i += 2)
    screen.drawFastHLine(offsetX + x, offsetY + 113 - i / 2, 32 - i, COLOR_RGB565_WHITE);

  for (int x = 40, i = 0; x < 46; x++, i++)
    screen.drawFastVLine(offsetX + x, offsetY + 106 + i, 8 - i, COLOR_RGB565_WHITE);

  for (int x = 90, i = 0; x < 96; x++, i++)
    screen.drawFastVLine(offsetX + x, offsetY + 111 - i, 3 + i, COLOR_RGB565_WHITE);

  // Blue eraser line to "cut" middle part
  for (int x = 66; x < 70; x++) {
    screen.drawFastVLine(offsetX + x, offsetY + 30, 115, COLOR_RGB565_BLUE);
  }
}



void drawThermometerIcon(int offsetX, int offsetY) {


  screen.drawXBitmap(/*x=*/offsetX,/*y=*/offsetY,/*bitmap gImage_Bitmap=*/thermo1,/*w=*/thermo1_width ,/*h=*/thermo1_height ,COLOR_RGB565_WHITE);  // 95 45
  screen.drawXBitmap(/*x=*/offsetX+2,/*y=*/offsetY+9,/*bitmap gImage_Bitmap=*/thermo2,/*w=*/thermo2_width ,/*h=*/thermo2_height ,COLOR_RGB565_RED);  // 95 45
}
/*  // Draw white thermometer body
  for (int y = 0; y < 30; y++) {
    for (int x = 0; x < 30; x++) {
      // White body shape
      if (
        (x >= 12 && x <= 17 && y >= 0 && y <= 20) ||        // Extended stem
        (x >= 8 && x <= 21 && y >= 21 && y <= 29)           // Bulb lower
      ) {
        screen.drawPixel(offsetX + x, offsetY + y, COLOR_RGB565_WHITE);
      }
    }
  }

  // Draw red mercury (now longer)
  for (int y = 4; y <= 20; y++) {
    for (int x = 14; x <= 16; x++) {
      screen.drawPixel(offsetX + x, offsetY + y, COLOR_RGB565_RED);
    }
  }

  // Draw red bulb
  for (int y = 22; y <= 28; y++) {
    for (int x = 10; x <= 19; x++) {
      screen.drawPixel(offsetX + x, offsetY + y, COLOR_RGB565_RED);
    }
  }

  // Right-side tick marks (slightly spaced out for 30px height)
  for (int i = 4; i <= 20; i += 4) {
    for (int j = 0; j < 3; j++) {
      screen.drawPixel(offsetX + 18 + j, offsetY + i, COLOR_RGB565_WHITE);
    }
  }
}
*/

/**************************************************************************************
 * Draws the Menu icon, used at Main Menu (the paper with the lines/options)          *
 **************************************************************************************/

void drawMenuIcon() {
  screen.fillRoundRect(/*x0=*/57, /*y0=*/1, /*w=*/25, /*h=*/30, /*radius=*/2, /*color=*/COLOR_RGB565_WHITE);    
    for (uint8_t i = 0; i < 28; i+= 7) {
      for (uint8_t y = 0; y < 3; y++) {
        screen.drawFastHLine(60, 5 + y + i, 20, COLOR_RGB565_DGRAY );
      } 
    }
    
    for (uint8_t i = 0; i < 3; i++) {
      screen.drawFastVLine(65+i, 3, 28, COLOR_RGB565_WHITE );
    }
}

/**************************************************************************************
 * Draws the Display icon, used at Display Menu (the screen with OIL at the center)   *
 **************************************************************************************/

void drawDispIcon() {

  screen.fillRoundRect(/*x0=*/45, /*y0=*/1, /*w=*/40, /*h=*/30, /*radius=*/2, /*color=*/COLOR_RGB565_DGRAY);
  screen.fillRoundRect(/*x0=*/49, /*y0=*/5, /*w=*/32, /*h=*/22, /*radius=*/2, /*color=*/COLOR_RGB565_CYAN);
  screen.setTextSize(1);
  screen.setCursor(57, 13);
  screen.setTextColor(COLOR_RGB565_RED, COLOR_RGB565_CYAN);  
  screen.print("AFR");
  screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK);
   
}

/**************************************************************************************
 * Draws the Folder icon, used at Log Menu and while Logging is enabled               *
 **************************************************************************************/

void drawFolderIcon(uint16_t x, uint16_t y) {

  screen.fillRoundRect(/*x0=*/x, /*y0=*/y, /*w=*/40, /*h=*/30, /*radius=*/1, /*color=*/COLOR_RGB565_ORANGE);
  screen.fillRoundRect(/*x0=*/x, /*y0=*/y+7, /*w=*/34, /*h=*/23, /*radius=*/1,/*color=*/COLOR_RGB565_YELLOW);
 
  screen.drawFastHLine(x+12, y, 30, COLOR_RGB565_BLACK);
  screen.drawFastHLine(x+12, y+1, 28, COLOR_RGB565_BLACK);
  screen.drawFastHLine(x+14, y+2, 26, COLOR_RGB565_BLACK);

}

/**************************************************************************************
 * Draws the LED icon, used at LED Menu (the three colorful bars)                     *
 **************************************************************************************/

void drawLedIcon() {

  screen.fillRoundRect(/*x0=*/45, /*y0=*/1, /*w=*/10, /*h=*/30, /*radius=*/1, /*color=*/COLOR_RGB565_GREEN);
  screen.fillRoundRect(/*x0=*/60, /*y0=*/1, /*w=*/10, /*h=*/30, /*radius=*/1,/*color=*/COLOR_RGB565_YELLOW);
  screen.fillRoundRect(/*x0=*/75, /*y0=*/1, /*w=*/10, /*h=*/30, /*radius=*/1,/*color=*/COLOR_RGB565_RED);
}

/**************************************************************************************
 * Draws the warning icon, used when errors occur (Trinagle with !)                   *
 **************************************************************************************/

void drawWarningIcon(uint8_t severity) {

  //--- Placeholder code for different error severity ---
  int16_t error_color;
  if (severity < 1) error_color = COLOR_RGB565_ORANGE;        // Low severity = orange triangle
  else error_color = COLOR_RGB565_RED;                        // High severity = red triangle
  
  screen.fillTriangle(160, 5, 130, 45, 190, 45, error_color);
  screen.setTextColor(COLOR_RGB565_WHITE, error_color); 
  screen.setTextSize(3);
  screen.setCursor(153, 18);
  screen.print("!");
}

/**************************************************************************************
 * Handles a sensor error and advises user on maintainance action                     *
 **************************************************************************************/


void errorTemp() {
  uint8_t user_action = 0;
  screen.fillScreen(COLOR_RGB565_BLACK);
  drawThermometerIcon(100,2);
  drawWarningIcon(0);

  screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK); 
  screen.setTextSize(3);
  screen.setCursor(5, 68);
  screen.println(" Sens. Overheat!");
  screen.setTextSize(3);
  screen.println(" Stop vehicle for");
  screen.println(" sensor cooldown!");
  while(sens_temp > 1030) {    
    if (user_action == 0) user_action = flashLEDS();
    cli();
    wdt_reset(); 
    sei(); 
  }
  drawMainScreen();
  timer = 51;
}

void errorCAN() {
  uint8_t user_action = 0;
  screen.fillScreen(COLOR_RGB565_BLACK);
  drawWarningIcon(1);

  screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK); 
  screen.setTextSize(3);
  screen.setCursor(0, 60);
  screen.println(" CAN init failed!");
  screen.setTextSize(3);
  screen.println(" Check board");
  while(1) {    
    if (user_action == 0) user_action = flashLEDS();
    cli();
    wdt_reset(); 
    sei();  
  }
  
}

/**************************************************************************************
 * Handles a sensor error and advises user on maintainance action                     *
 **************************************************************************************/


void errorCJ125(uint8_t error_type) {
  uint8_t user_action = 0;
  screen.fillScreen(COLOR_RGB565_BLACK);
  drawWarningIcon(1);

  screen.setTextColor(COLOR_RGB565_WHITE, COLOR_RGB565_BLACK); 
  screen.setTextSize(3);
  screen.setCursor(0, 60);
  screen.println("  Wideband Error");
  screen.setTextSize(3);
  switch (error_type) {
    case 0: 
      screen.println(" No sens detected");
      break;
    case 1: 
      screen.println(" Sens. Shrt Circ");
      break;
    case 2: 
      screen.println(" VBat too low.");
      break;
    
  }
  
  while(1) {    
    if (user_action == 0) user_action = flashLEDS();
    cli();
    wdt_reset(); 
    sei();  
  }
  
}

// Sends a single byte over the CAN bus using message ID 0xFF
void sendCAN(uint8_t data_sent) {
    uint8_t attempt = 0;

    PORTD &= ~(1 << PD2);  // set CAN standby low

    while (attempt < MAX_RETRIES) {
        if (CAN.beginPacket(0xFF, 2)) {
            CAN.write(data_sent);   // Write a single byte of data
            CAN.write(flagDebug);
            CAN.endPacket();        // Finalize and send
            break;                  // success → exit loop
        }

        attempt++;
        delay_ms(2);  // small delay before retry
    }

    PORTD |= (1 << PD2); // release standby
}



// Called when a CAN packet is received
// Expects a packet of 8 bytes in a specific format:
// [0] error flag
// [1-2] lambda value (high byte, low byte)
// [3-4] O2 value (high byte, low byte)
// [5-6] sensor temp (high byte, low byte)
// [7] battery voltage
void packetReceive(uint8_t packetSize) {
  
  // Check if it's a valid full 7-byte packet and not a remote transmission request (RTR)
  PORTD &= ~(1 << PD2);  // set CAN standby low
  if (CAN.packetId() == 0xFE && packetSize == 8) {
    uint8_t buffCAN[8];
    uint8_t i = 0;

   // Read exactly 8 bytes (defensive against partial availability)
    while (i < 8 && CAN.available()) {
        buffCAN[i++] = (uint8_t)CAN.read();
    }
    
    if (i != 8) return;  // short read; drop frame
    // Parse
    status_flags  = buffCAN[0];
    lambda_value = (uint16_t)((uint16_t)buffCAN[1] << 8) | buffCAN[2];
    afr_int      = buffCAN[3]; 
    afr_decimal  = buffCAN[4];
    sens_temp    = (uint16_t)((uint16_t)buffCAN[5] << 8) | buffCAN[6];
    vbat         = buffCAN[7];
  }
   PORTD |= (1 << PD2); // release standby
}


/**************************************************************************************
 * Sets the screen backlight intensity by changing the PWM duty Cycle                 *
 **************************************************************************************/

void setBacklightLevel(uint8_t level) {
  switch (level) {
    case 0:
      OCR0A = 255;   // High
      break;
    case 1:
      OCR0A = 170;   // Med
      break;
    case 2:
      OCR0A = 85;    // Low
      break;
    case 3:
      uint16_t photo_sens = adc_read(PHOTO_SENSOR, 10);   //Auto
      photo_sens = (1023-photo_sens) / 4;
      if (photo_sens < 64) photo_sens = 64;
      OCR0A = photo_sens;
      bl_tick = 0;
      break;      
  }
}
/**************************************************************************************
 * Send up to a uint16_t value through UART, for logging, formatted as a string.      *
 **************************************************************************************/

void logData(uint16_t value) {

     if (!(status_flags & STATUS_SENSOR) && !(status_flags & STATUS_SHORT_CIRC) && !(status_flags & STATUS_LOW_BAT)) {
        // Handle value printing
        if (value == 0) {
            while (!(UCSR0A & (1 << UDRE0))); UDR0 = '0';
        } else {
            char buf[5];   // up to 4 digits for 0–9999
            uint8_t i = 0;

            while (value > 0 && i < sizeof(buf)) {
                buf[i++] = (value % 10) + '0';
                value /= 10;
            }

            while (i > 0) {
                while (!(UCSR0A & (1 << UDRE0)));
                UDR0 = buf[--i];
            }
        }
    } else {
        // Print "CJ125_Error_0x" + status in hex
        const char *msg = "Wideband_Error";
        while (*msg) {
            while (!(UCSR0A & (1 << UDRE0)));
            UDR0 = *msg++;
        }
    }

    // Common CR/LF
    while (!(UCSR0A & (1 << UDRE0))); UDR0 = '\r';
    while (!(UCSR0A & (1 << UDRE0))); UDR0 = '\n';
}

/*void logData(int lambda_val, int afr_value) {

   if (status_flags == CJ125_DIAG_REG_STATUS_OK) {
      
      //Assembled data.
      txString = "Oxygen:";
      txString += String(afr_value, 2);
      txString += ",Lambda:";
      txString += String(lambda_val, 2);

      //Output string
      Serial.println(txString);
   }
      //Error handling.
   else {
      Serial.print("CJ125_Error_0x");
      Serial.print(status_flags, HEX);
      Serial.print("\n\r");
   }
        
    
}

/**************************************************************************************
 * Updates the indicator LEDs depending on chosen LED mode and the AFR value *
 **************************************************************************************/

// led_mode: 0 = single LED at index, 1 = bar up to index, default = all off
static inline void UpdateLEDS(uint8_t afr_int, uint8_t led_mode)
{
  // Map AFR (11..18) → index (0..7) with clamp, branchless-ish
  uint8_t index;
  if (afr_int <= 11)      index = 0;
  else if (afr_int >= 18) index = 7;
  else                      index = (uint8_t)(afr_int - 11);

  // Build ON-mask for 8 LEDs in bits 0..7 (1 = LED ON)
  uint8_t onmask;
  if (led_mode == 0) {
    onmask = (uint8_t)(1u << index);
  } else if (led_mode == 1) {
    // LEDs 0..index ON; handle index=7 without 1<<8 overflow
    onmask = (index == 7) ? 0xFFu : (uint8_t)((1u << (index + 1)) - 1u);
  } else {
    onmask = 0; // all OFF
  }

  // ----- Write PORTC (PC0..PC5) -----
  // Active-low: ON -> 0, OFF -> 1; preserve other PORTC bits
  uint8_t pc_final = (uint8_t)(~onmask) & 0x3Fu;      // desired bits for PC0..PC5
  PORTC = (PORTC & (uint8_t)~0x3Fu) | pc_final;

  // ----- Write PORTE (PE2 = LED index 6) -----
  const uint8_t E_MASK = (1 << PE2);
  uint8_t e_bit = (onmask & (1u << 6)) ? 0 : E_MASK;  // ON->0, OFF->1
  PORTE = (PORTE & (uint8_t)~E_MASK) | e_bit;

  // ----- Write PORTD (PD7 = LED index 7) -----
  const uint8_t D_MASK = (1 << PD7);
  uint8_t d_bit = (onmask & (1u << 7)) ? 0 : D_MASK;  // ON->0, OFF->1
  PORTD = (PORTD & (uint8_t)~D_MASK) | d_bit;
}

uint8_t flashLEDS() {
  
    uint8_t btn1 = (PIND & (1 << PIND3)) ? 1 : 0;  // Typically the "select" button
    uint8_t btn2 = (PIND & (1 << PIND5)) ? 1 : 0;  // Typically the "next" button
   
    if (btn2 == 0 || btn1 == 0) { 
      btn1 = 1;
      btn2 = 1;
      return 1;
    }

    PORTC = 0b00000000;                    // PC1..PC5 LOW
    PORTE &= ~(1 << PE2);                  // PE2 LOW
    PORTD &= ~(1 << PD7);                  // PD7 LOW     

    delay_ms(500);

    if (btn2 == 0 || btn1 == 0) { 
      btn1 = 1;
      btn2 = 1;
      return 1;
    }

    PORTC = 0b00111111;                  // PC1..PC5 HIGH
    PORTE |= (1 << PE2);                  // PE2 HIGH
    PORTD |= (1 << PD7);                  // PD6, PD7 HIGH 
     
    delay_ms(500);
    return 0;
}

uint16_t adc_read(uint8_t channel, uint8_t times) {
    uint32_t sum = 0;

    //  select channel 0–7
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)); 

    for (uint8_t i = 0; i < times; i++) {      // Start conversion and sample X times
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC));        
        sum += ADC;
    }

    return (uint16_t)(sum / times); // Get the average reading.
}

static inline void makeFixed5(char out[6], uint8_t whole, uint8_t dec2)
{
  // Always "xx.xx" (leading space if < 10)
  out[0] = (whole < 10) ? ' ' : (char)('0' + (whole / 10));
  out[1] = (char)('0' + (whole % 10));
  out[2] = '.';
  out[3] = (char)('0' + (dec2 / 10));
  out[4] = (char)('0' + (dec2 % 10));
  out[5] = 0;
}

// Draw only changed character *cells* (fixed-width), force redraw on color/mode change
static inline void drawFixed5Cells(uint16_t x, uint16_t y,
                                   const char cur[6], char prev[6],
                                   uint16_t fg, uint16_t bg,
                                   bool force_redraw)
{
  screen.setTextSize(10);
  screen.setTextColor(fg, bg);

  for (uint8_t i = 0; i < 5; i++) {
    if (force_redraw || cur[i] != prev[i]) {
      screen.setCursor(x + (uint16_t)i * CHAR_W, y);

      char t[2] = { cur[i], 0 };   // 1-char string
      screen.print(t);

      prev[i] = cur[i];
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  bl_tick = 1;
}
