
#include "SPI.h"
#include "CAN.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "lookup_tables.h"
#include "BoostController.h"
#include "avr/wdt.h"
#include "EEPROM.h"
#include "Adafruit_MCP9600.h"

Adafruit_MCP9600 egt_sens;

//Global variables.
uint16_t v_batt = 0;                                                /* ADC value read from the voltage divider calculating Ubat */
uint8_t  status_flags = 0;                                          /* 8-bit status flag variable, see BoostController.h */
uint16_t egt = 0;
uint16_t board_amb = 0;
uint16_t ambient_pressure = 0;                                     /* MAP reading at key-on, used as atmospheric reference */
uint8_t  active_profile = 0;                                        /* Currently selected boost profile (0-3) */

// Timer2 scheduler state
volatile uint8_t tick_flags = 0;                                     /* Bitmask of pending task flags, set by ISR */
volatile uint8_t tick_count = 0;                                     /* 5ms tick counter (0-199, wraps every 1s) */

// Runtime sensor state (updated by loop tasks)
uint16_t map_mbar = 0;                                               /* Latest MAP reading (mbar) */
uint16_t rpm = 0;                                                    /* Latest RPM reading */
uint16_t vss = 0;                                                    /* Latest vehicle speed (km/h) */
uint8_t  gear = 0;                                                   /* Detected gear (0=neutral, 1-6) */
int16_t  iat = 0;                                                    /* Latest intake air temp (°C × 10) */
uint16_t boost_setpoint = 0;                                         /* Current boost target from map lookup (mbar) */
uint8_t  mac_output = 0;                                             /* PI controller output driving MAC valve (0-255) */

#if USE_CAN_SENSORS == 1
// CAN-sourced sensor data (populated by packetReceive from DME1 0x316)
volatile uint16_t can_rpm = 0;
volatile uint8_t  can_vss = 0;
volatile uint8_t  can_age_ticks = 255;   // 5ms ticks since last valid DME1 frame
#else
// RPM reading variables (Timer4 ICP4)
volatile uint16_t rpm_signal_period = 0;  
volatile uint16_t last_rpm_signal_val = 0;
// VSS reading variables (Timer3 ICP3)
volatile uint16_t vss_period = 0;
volatile uint16_t last_vss_val = 0;
#endif

//PID regulation variables.                                         /* Last position input. */
int32_t       integ_acc;                                            /* Integrator state. */
const int16_t iMax = 2550;                                          /* Maximum allowable integrator state. */
const uint8_t p_gain = 50;                                          /* Proportional gain. Default = 5*/
const uint8_t i_gain = 1;                                           /* Integral gain. Default = 0.2*/

void setup() {
  CAN.setPins(10, 2); // PB2 as CS
  wdt_enable(WDTO_2S);

  //Set up digital output pins.
  // Power LED (PB0)
  DDRB |= (1 << DDB0);  
  PORTB |= (1 << PORTB0); // Off initially (Active LOW)
  
  // MAC Valve PWM (PB1)
  DDRB |= (1 << DDB1);
  
  // CAN Status LED (PD7)
  DDRD |= (1 << DDD7);
  PORTD |= (1 << PORTD7); // Off initially (Active LOW)
  
  // CAN Standby (PD6)
  DDRD |= (1 << DDD6);
  PORTD |= (1 << PORTD6); // STDBY = 1 initially (sleep)

  /******************
  * MAC TIMER1 PWM  *
  ******************/
  // Timer1 Mode 14 (Fast PWM, TOP=ICR1). Prescaler 64. 
  // 16MHz / 64 = 250,000 tick/sec.
  // We want 25 Hz for MAC Valve -> TOP = 250000 / 25 - 1 = 9999
  TCCR1A = (1 << COM1A1) | (1 << WGM11); 
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
  ICR1 = 9999;
  OCR1A = 0; // 0% duty initially

#if USE_CAN_SENSORS == 0
  /******************
  *   RPM TIMER4    *
  ******************/
  // Set PE0 (ICP4) as input
  DDRE &= ~(1 << DDE0);
  TCCR4A = 0;
  TCCR4B = (1 << ICNC4) | (1 << ICES4) | (1 << CS42);
  TIMSK4 = (1 << ICIE4) | (1 << TOIE4);

  /******************
  *   VSS TIMER3    *
  ******************/
  // Set PE2 (ICP3) as input for VSS
  DDRE &= ~(1 << DDE2);
  TCCR3A = 0;
  TCCR3B = (1 << ICNC3) | (1 << ICES3) | (1 << CS32) | (1 << CS30); 
  TIMSK3 = (1 << ICIE3) | (1 << TOIE3);
#endif

  /******************
  *    ADC SETUP    *
  ******************/
  ADMUX  = (1 << REFS0); // REFS1=0, REFS0=1 -> AVCC 
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler = 128 (16 MHz / 128 = 125 kHz ADC clock)

  /******************
  * TIMER2 SCHEDULER*
  ******************/
  // Timer2 CTC mode, prescaler 1024. 16MHz / 1024 = 15625 Hz.
  // OCR2A = 77 -> (77+1) / 15625 = 4.992ms ≈ 5ms tick
  TCCR2A = (1 << WGM21);                        // CTC mode (WGM22:0 = 010)
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
  OCR2A  = 77;
  TIMSK2 = (1 << OCIE2A);                       // Enable compare match A interrupt

  setBoostLevel(0); /* PWM is initially off. */ 

  if (!egt_sens.begin(MCP_ADDRESS)) {
    status_flags |= STATUS_EGT_FAIL;
  }
  egt_sens.setAmbientResolution(RES_ZERO_POINT_25);
  egt_sens.setADCresolution(MCP9600_ADCRESOLUTION_14);
  egt_sens.setThermocoupleType(MCP9600_TYPE_K);
  egt_sens.setFilterCoefficient(3);
  //egt_sens.setAlertTemperature(1, 30);
  egt_sens.enable(true);
  
  if (!CAN.begin(500000)) { // 500kbps
    LED_Control(LED_STATUS_CAN, 1);
  }
      
  CAN.onReceive(packetReceive);
  
  //delay_ms(1000);
  cli();
  wdt_reset(); 
  sei();
  //Start main function.
  start();    
}

// --- Main startup sequence (orchestrator) ---
void start() {

  // 1. Read ambient (atmospheric) pressure as a reference point for boost calculations
  ambient_pressure = read_map();

  // 2. Read and store battery voltage at key-on
  v_batt = adc_read(UB_ANALOG_INPUT_PIN);

  // Turn ON power LED to indicate setup complete
  LED_Control(LED_STATUS_POWER, 1);

  // 3. Wait for engine to start (RPM > 600, confirmed twice with 1s gap)
  while (1) {
    cli();
    wdt_reset();
    sei();

    if (read_rpm() > 600) {
      delay_ms(1000);           // Wait 1 second and re-check to confirm engine is running

      cli();
      wdt_reset();
      sei();

      if (read_rpm() > 600) {
        status_flags |= STATUS_ENGINE_ON;
        break;                  // Engine confirmed running
      }
    }

    delay_ms(100);              // Polling interval while waiting for engine start
  }
  sendCAN();
}

//Infinite loop — cooperative scheduler driven by Timer2 tick flags.
void loop() {

  // Atomically snapshot and clear only the flags we're about to service
  cli();
  uint8_t flags = tick_flags;
  tick_flags &= ~flags;
  wdt_reset();
  sei();

  // Nothing pending — yield
  if (flags == 0) return;

  // --- 10ms tasks ---
  if (flags & TICK_FLAG_MAP) {
    map_mbar = read_map();               // Read MAP sensor
  }

  if (flags & TICK_FLAG_SENS) {
    #if USE_CAN_SENSORS == 0
    rpm = read_rpm();                    // Sample RPM from hardware timer
    vss = read_vss();                    // Sample VSS from hardware timer
    if (can_age_ticks > CAN_TIMEOUT_TICKS) {
      status_flags |= STATUS_VSS_FAULT;
    } else {
      status_flags &= ~STATUS_VSS_FAULT;
    }
    
    #else
    rpm = read_rpm();                    // Copy latest CAN RPM
    vss = read_vss();                    // Copy latest CAN VSS
    #endif
    gear = calculate_gear(rpm, (uint8_t)vss);
  }

  // --- 40ms task: boost control ---
  if (flags & TICK_FLAG_BOOST) {
    boost_setpoint = lookup_boost_setpoint(gear, rpm);
    mac_output = boost_control(boost_setpoint, map_mbar, rpm);
    setBoostLevel(mac_output);
  }

  // --- 100ms task: CAN transmit ---
  if (flags & TICK_FLAG_CAN_TX) {
    sendCAN();
    //debug();
  }

  // --- 200ms task: intake air temperature ---
  if (flags & TICK_FLAG_IAT) {
    iat = read_intake_temp();
  }

  // --- 500ms task: exhaust gas + ambient temperature ---
  if (flags & TICK_FLAG_EGT) {
    read_exhaust_temp();
    v_batt = adc_read(UB_ANALOG_INPUT_PIN);      // Refresh battery voltage periodically
  }
}

//Boost regulating routine (PI).
uint8_t boost_control(uint16_t boost_setpoint, uint16_t current_boost, uint16_t RPM) {
  
  if (RPM < 1500) {  //Prevent boost control below 1500 RPM.
    integ_acc = 0;
    return 0;
  }

  //Calculate error term.
  int error = (int)boost_setpoint - current_boost;  
  
  //Calculate the integral state with appropriate limiting.
  integ_acc += error;
  if (integ_acc > iMax) integ_acc = iMax;
  if (integ_acc < -iMax) integ_acc = -iMax;
  
  //Calculate proportional term.
  int32_t p_term = (int32_t)p_gain * error;
      
  //Calculate the integral term.
  int32_t i_term = (int32_t)i_gain * integ_acc;
  //i_term = i_term/10;
  
  //Calculate regulation (PI).
  int PID = (p_term + i_term) / 10; 
   
  //Clamp maximum MAC valve output.
  if (PID > 255) PID = 255; 
   
  //Clamp minimum MAC valve output.
  if (PID < 0) PID = 0;
  
  //Return calculated PWM output.
  return (uint8_t)PID;
}


uint16_t lookup_boost_setpoint(uint8_t gear, uint16_t rpm) {  
  // No boost target if gear is unknown (neutral/shifting) or profile is invalid
  if (gear == 0 || gear > BOOST_NUM_GEARS || active_profile >= BOOST_NUM_PROFILES) return 0;

  // Get pointer to the active profile's base address
  const uint16_t *map_base = boost_profiles[active_profile];

  // Gear index is 0-based in the map (gear 1 = row 0)
  uint8_t gear_row = gear - 1;

  // Read first and last RPM breakpoints from PROGMEM
  uint16_t rpm_min = pgm_read_word_near(&n_axis[0]);
  uint16_t rpm_max = pgm_read_word_near(&n_axis[BOOST_NUM_RPM_POINTS - 1]);

  // Clamp RPM to the map range
  if (rpm <= rpm_min) {
    return pgm_read_word_near(&map_base[gear_row * BOOST_NUM_RPM_POINTS]);
  }
  if (rpm >= rpm_max) {
    return pgm_read_word_near(&map_base[gear_row * BOOST_NUM_RPM_POINTS + (BOOST_NUM_RPM_POINTS - 1)]);
  }

  // Find the two RPM breakpoints bracketing the current RPM
  for (uint8_t i = 1; i < BOOST_NUM_RPM_POINTS; i++) {
    uint16_t rpm_hi = pgm_read_word_near(&n_axis[i]);

    if (rpm <= rpm_hi) {
      uint16_t rpm_lo = pgm_read_word_near(&n_axis[i - 1]);

      // Read the two boost values for this gear at the bracketing RPM points
      uint16_t boost_lo = pgm_read_word_near(&map_base[gear_row * BOOST_NUM_RPM_POINTS + (i - 1)]);
      uint16_t boost_hi = pgm_read_word_near(&map_base[gear_row * BOOST_NUM_RPM_POINTS + i]);

      // Linear interpolation: boost = boost_lo + (rpm - rpm_lo) * (boost_hi - boost_lo) / (rpm_hi - rpm_lo)
      int16_t d_boost = (int16_t)boost_hi - (int16_t)boost_lo;
      uint16_t d_rpm = rpm_hi - rpm_lo;
      int32_t interp = (int32_t)(rpm - rpm_lo) * d_boost / d_rpm;

      return (uint16_t)((int16_t)boost_lo + (int16_t)interp);
    }
  }

  return 0; // Fallback (should not reach here)
}

int16_t read_intake_temp() {
  uint16_t input_adc = adc_read(IAT_ANALOG_INPUT_PIN);
  uint16_t low_clamp = pgm_read_word_near(&tia_x_axis[0]);
  // Out of bounds - clamp to lowest temperature (which corresponds to highest ADC)
  if (input_adc >= low_clamp) return (int16_t)pgm_read_word_near(&ip_tia_mes[0]);
  
  uint16_t high_clamp = pgm_read_word_near(&tia_x_axis[14]);
  // Out of bounds - clamp to highest temperature (lowest ADC)
  if (input_adc <= high_clamp) return (int16_t)pgm_read_word_near(&ip_tia_mes[14]);
  
  return linear_interp(input_adc, tia_x_axis, ip_tia_mes, 15);
}

void read_exhaust_temp() {  
  board_amb = egt_sens.readAmbient();  
  egt = egt_sens.readThermocouple(); 
}

uint16_t read_map() {
  // Read the MAP sensor
  uint16_t input_adc = adc_read(MAP_ANALOG_INPUT_PIN);
  
  uint16_t low_clamp = pgm_read_word_near(&map_x_axis[0]);
  // Out of bounds
  if (input_adc >= low_clamp) return pgm_read_word_near(&ip_map_mes[0]);
  
  uint16_t high_clamp = pgm_read_word_near(&map_x_axis[1]);
  // Out of bounds
  if (input_adc <= high_clamp) return pgm_read_word_near(&ip_map_mes[1]);

  return linear_interp(input_adc, map_x_axis, ip_map_mes, 2);
}

#if USE_CAN_SENSORS == 1
uint16_t read_rpm() {
    uint16_t rpm;
    uint8_t age;

    cli();
    rpm = can_rpm;
    age = can_age_ticks;
    sei();

    if (age > CAN_TIMEOUT_TICKS) return 0;
    return rpm;
}

uint16_t read_vss() {
    uint8_t vss;
    uint8_t age;

    cli();
    vss = can_vss;
    age = can_age_ticks;
    sei();

    if (age > CAN_TIMEOUT_TICKS) return 0;
    return (uint16_t)vss;
}

#else
uint16_t read_rpm() {
    uint16_t period;
    
    // Safely copy the 16-bit volatile period avoiding interrupt
    cli();
    period = rpm_signal_period;
    sei();

    // Prevent divide by zero if engine is fully stopped and timer overflowed
    if (period == 0) return 0;
    
    // Timer4 runs at 16MHz / 256 = 62.5 kHz.
    // Measured points: 104.6Hz @ 3000 RPM, 24.7Hz @ 800 RPM.
    // Linear Map: RPM = 27.534 * f + 120
    // Substitute (f = 62500 / period) into mapping formula -> RPM = 1720875 / period + 120
    uint32_t rpm = (1720875UL / period) + 120;
    
    return (uint16_t)rpm;
}

uint16_t read_vss() {
  uint16_t period;
  
  // Safely copy the 16-bit hardware period
  cli();
  period = vss_period;
  sei();

  // If timer overflowed (takes> 4.19s at 1024 prescaler), car is stopped (< 1.68 km/h)
  if (period == 0) return 0;

  // Wheel Size: 215/45/R17 (Circumference ~1.9644 meters)
  // M5BF2 VSS Gears: 36-tooth Drive, 29-tooth Driven = 1.241 pulses per wheel rev
  // Travel distance per pulse = 1.9644 / 1.241 = 1.5824 meters
  // Timer3 freq @ 1024 prescaler = 15.625 kHz
  // Speed (km/h) = (15625 / period) * 1.5824 m/pulse * 3.6 = 89012 / period
  uint32_t speed_kmh = 89012UL / period;
  
  return (uint16_t)speed_kmh;
}
#endif

uint8_t calculate_gear(uint16_t rpm, uint8_t vss) {
    if (vss < 3 || rpm < 1000) return 0; // Stopped or clutch in
    
    uint16_t ratio = ((uint32_t)rpm * 10) / vss;
    
    uint16_t min_diff = 65535;
    uint8_t gear = 0;
    
    for (uint8_t i = 1; i <= 6; i++) {
        uint16_t expected = pgm_read_word_near(&id_gr_n_vs_ratio[i]);
        uint16_t diff = (ratio > expected) ? (ratio - expected) : (expected - ratio);
        
        if (diff < min_diff) {
            min_diff = diff;
            gear = i;
        }
    }
    
    // Check if within acceptable tolerance (approx ~16% margin for bounds intersection)
    uint16_t expected_best = pgm_read_word_near(&id_gr_n_vs_ratio[gear]);
    if (min_diff < (expected_best / 6)) {
        return gear;
    }
    
    return 0; // Neutral, slipping, or shifting
}

void delay_ms(uint32_t ms) {
    ms = 1450 * ms;             // I have 0 idea why this number isn't even close to 16000 when running with a 16MHz clock
    for (ms; ms > 1; ms--) {
       asm("nop");
    }
}  

void LED_Control(uint8_t LED_Name, uint8_t state) {
  if (state == 1) { // Turn ON (Active Low = sink to GND)
    if (LED_Name == LED_STATUS_CAN) PORTD &= ~(1 << PORTD7);
    else PORTB &= ~(1 << PORTB0);  
  }
  else { // Turn OFF (Float to 5V)
    if (LED_Name == LED_STATUS_CAN) PORTD |= (1 << PORTD7);
    else PORTB |= (1 << PORTB0);
  }    
}

void setBoostLevel(uint8_t level) {
   OCR1A = (uint16_t)level * 39;     // 255 * 39 = 9945 which is 99.4% duty cycle. 100% is 9999, but needs slow 32bit math.
}                                    // In a MAC valve 0.6% precision loss is unnoticable.

uint16_t adc_read(uint8_t channel) {
    uint32_t sum = 0;

    //  select channel 0–7
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)); 

    for (uint8_t i = 0; i < 16; i++) {      // Start conversion and sample 16 times
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC));        
        sum += ADC;
    }

    return (uint16_t)(sum >> 4); // Get the average reading.
}


//int16_t linear_interp(uint16_t input_adc, const PROGMEM uint16_t *x_axis, const PROGMEM int16_t *y_axis, uint8_t num_points) {
int16_t linear_interp(uint16_t input_adc, const uint16_t *x_axis, const int16_t *y_axis, uint8_t num_points) {
  // Linear interpolation between points. (Looping design supports freely adding more points later!)
  for (uint8_t i = 1; i < num_points; i++) {
      uint16_t x1 = pgm_read_word_near(&x_axis[i]);
      if (input_adc >= x1) {
          uint16_t x0 = pgm_read_word_near(&x_axis[i - 1]);
          int16_t y0 = (int16_t)pgm_read_word_near(&y_axis[i - 1]);
          int16_t y1 = (int16_t)pgm_read_word_near(&y_axis[i]);
          
          // X axis is reversed (decreasing ADC values)
          int16_t dx = x0 - x1;
          int16_t dy = y1 - y0;
          int32_t dx_in = x0 - input_adc;
          
          return y0 + (dx_in * dy / dx);
      }
  }
  return 0; // Fallback
}

void sendCAN() {
    uint8_t attempt = 0;

    PORTD &= ~(1 << PORTD6); // set standby low

    while (attempt < MAX_RETRIES) {
      if (CAN.beginPacket(0xFE, 8)) {
        // [0] error flag
        CAN.write(status_flags);

        // [1-2] lambda (high byte, low byte)
        CAN.write(highByte(map_mbar));
        CAN.write(lowByte(map_mbar));

        // [3-4] EGT (high byte, low byte)
        CAN.write(highByte(egt));
        CAN.write(lowByte(egt));

        // [5-6] IAT (high byte, low byte)
        CAN.write(highByte(iat));
        CAN.write(lowByte(iat));

        // [7] battery voltage (already a byte)
        CAN.write(v_batt);

        CAN.endPacket();
        break; // transmission succeeded, exit loop
      }

        attempt++;
        delay_ms(2);  // small delay before retry
    }

    PORTD |= (1 << PORTD6); // release standby
}

void packetReceive(int packetSize) {

  PORTD &= ~(1 << PORTD6);
  uint8_t buffCAN[8];
  uint8_t i = 0;
  // --- Control frame from dashboard (ID 0xFF, 2 bytes) ---
  if (CAN.packetId() == 0xFF && packetSize == 1) {
    while (i < 1 && CAN.available()) {
        buffCAN[i++] = (uint8_t)CAN.read();
    }
    active_profile = buffCAN[0] & 0x03;          // Byte 0 bits [1:0] = profile select (0-3)
  }

  #if USE_CAN_SENSORS == 1 
  // --- DME1 frame from ECU (ID 0x316, 8 bytes, every 10ms) ---
  if (CAN.packetId() == DME1_CAN_ID && packetSize == 8) {
    i = 0;
    while (i < 8 && CAN.available()) {
        buffCAN[i++] = (uint8_t)CAN.read();
    }

    // Byte 2 = N_ENG LSB, Byte 3 = N_ENG MSB
    // RPM = ((MSB * 256) + LSB) * 0.15625 = ((MSB * 256) + LSB) * 5 / 32
    uint16_t raw_n = ((uint16_t)buffCAN[3] << 8) | buffCAN[2];
    can_rpm = ((uint32_t)raw_n * 5) / 32;

    // Byte 6 = VS_CAN (direct km/h)
    can_vss = buffCAN[6];
    can_age_ticks = 0;
  }
  #endif

  // Flush any remaining unread bytes
  while (CAN.available()) CAN.read();

  PORTD |= (1 << PORTD6);
}

/*void debug() {

  Serial.print("UBat: ");
  Serial.print(v_batt);
  Serial.print(" | UR: ");
  Serial.print(sens_resistance);
  Serial.print(" (opt=");
  Serial.print(sens_resistance_optimal);
  Serial.print(") | UA: ");
  Serial.print(sens_signal);
  Serial.print(" (opt=");
  Serial.print(sens_signal_optimal);
  Serial.print(")");

  Serial.print(" | Lambda: ");
  Serial.print(lambda_value);
  Serial.print(" | Temp: ");
  Serial.print(sens_temp);

  Serial.print(" | HeaterPWM: ");
  Serial.print(sens_heater_out);
  Serial.print(" | Flags: ");
  Serial.println(status_flags, BIN);
  Serial.print(" | FlagEMU: ");
  Serial.println(flagEMU, HEX);

}*/


#if USE_CAN_SENSORS == 0
ISR(TIMER4_CAPT_vect) {
    uint16_t current_val = ICR4;
    rpm_signal_period = current_val - last_rpm_signal_val;
    last_rpm_signal_val = current_val;
}

ISR(TIMER4_OVF_vect) {
    // If Timer4 overflows (takes ~1.048 seconds at 256 prescaler), signal is lost / engine is off
    rpm_signal_period = 0; 
}

ISR(TIMER3_CAPT_vect) {
  uint16_t current_val = ICR3;
  vss_period = current_val - last_vss_val;
  last_vss_val = current_val;
}

ISR(TIMER3_OVF_vect) {
  // Overflows at 4.19 seconds interval (speed < 1.68 km/h). Car is effectively stopped.
  vss_period = 0;
}
#endif

// --- Timer2 Compare Match A: 5ms scheduler tick ---
ISR(TIMER2_COMPA_vect) {
  tick_count++;

  // 10ms — MAP + RPM/VSS (every 2 ticks)
  if ((tick_count & 0x01) == 0) {
    tick_flags |= TICK_FLAG_MAP | TICK_FLAG_SENS;
  }

  // 40ms — Boost lookup + PI + MAC drive (every 8 ticks)
  if ((tick_count & 0x07) == 0) {
    tick_flags |= TICK_FLAG_BOOST;
  }

  // 100ms — CAN TX (every 20 ticks)
  if ((tick_count % 20) == 0) {
    tick_flags |= TICK_FLAG_CAN_TX;
  }

  // 200ms — IAT (every 40 ticks)
  if ((tick_count % 40) == 0) {
    tick_flags |= TICK_FLAG_IAT;
  }

  // 500ms — EGT + ambient (every 100 ticks, then wrap counter)
  if (tick_count >= 100) {
    tick_flags |= TICK_FLAG_EGT;
    tick_count = 0;
  }

  #if USE_CAN_SENSORS == 1
  if (can_age_ticks < 255) {
    can_age_ticks++;
  }
  #endif
}
