
#include "SPI.h"
#include "CAN.h"
#include "Adafruit_MCP4725.h"
#include "avr/io.h"
#include "avr/interrupt.h"
#include "lookup_tables.h"
#include "OpenAFR.h"
#include "avr/wdt.h"
Adafruit_MCP4725 dac;

//Global variables.
uint16_t sens_signal = 0;                                           /* ADC value read from the CJ125 UA output pin */ 
uint16_t sens_resistance = 0;                                       /* ADC value read from the CJ125 UR output pin */
uint16_t v_batt = 0;                                                /* ADC value read from the voltage divider caluclating Ubat */
uint16_t sens_signal_optimal = 0;                                   /* UA ADC value stored when CJ125 is in calibration mode, λ=1 */ 
uint16_t sens_resistance_optimal = 0;                               /* UR ADC value stored when CJ125 is in calibration mode, optimal temperature */
uint8_t  sens_heater_out = 0;                                       /* Current PWM output value (0-255) of the heater output pin */
uint8_t  buffCAN[8];
uint32_t CJ125_Status = 0;                                          /* Latest stored DIAG registry response from the CJ125 */
uint8_t  flagEMU = 0;
uint8_t  flagDebug = 0;
uint8_t  com_flag= 0;
uint8_t  status_flags = 0;                                          /* 8-bit flag variable, see OpenAFR.h definitions for each bit */
uint16_t analog_out = 0;
int16_t  calibration = 0;

//PID regulation variables.                                         /* Last position input. */
int32_t       integ_acc;                                            /* Integrator state. */
const int16_t iMax = 2550;                                          /* Maximum allowable integrator state. */
const uint8_t p_gain = 50;                                          /* Proportional gain. Default = 5*/
const uint8_t i_gain = 1;                                           /* Integral gain. Default = 0.2*/

uint16_t lambda_value, afr_value;
uint16_t sens_temp = 0;
uint16_t v_batt_Mem;


void setup() {
  CAN.setPins(9, 2);
  //Serial.begin(9600);
  dac.begin(0x60);
  dac.setVoltage(0, false); // 0V on emulation
  wdt_enable(WDTO_2S);
  //Set up digital output pins.

  // Set pin 10 CJ125_NSS_PIN (PB2) as output 
  DDRB |= (1 << DDB2);  
  // Set pin 26 LED_STATUS_POWER (PC2) as output
  DDRE |= (1 << DDE3);  
  // Set pin 25 LED_STATUS_CAN (PC1) as output
  DDRE |= (1 << DDE2);  
  // Set pin 5 HEATER_OUTPUT_PIN (PD5, OC0B) as output
  DDRD |= (1 << DDD5);  
  // Set pin 7 CAN_STANDBY (PD7) as output
  DDRD |= (1 << DDD7);

  /******************
  * TIMER PWM SETUP *
  ******************/
  TCCR0A = (1<<WGM00) | (1<<WGM01) | (1<<COM0B1); // enable OC0B (PD5)
  TCCR0B = (1 << CS01) | (1 << CS00);             // same mode/clock

  // Setup timer for communication window every  100ms
  TCCR1A = 0;                          // normal port operation
  TCCR1B = (1 << WGM12);
  // WGM12=1 -> CTC mode
  OCR1A = 24999;                       // 100ms compare value
  TIMSK1 = (1 << OCIE1A);              // enable compare match A interrupt

    // Start Timer1 for periodic CAN communication
  TCCR1B |= (1 << CS11) | (1 << CS10);

  /******************
  *    ADC SETUP    *
  ******************/
  ADMUX  = (1 << REFS0); // REFS1=0, REFS0=1 -> AVCC 
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler = 128 (16 MHz / 128 = 125 kHz ADC clock)


  //Set initial values.
  // digitalWrite(CJ125_NSS_PIN, HIGH);
  PORTB |= (1 << PORTB2);  
  // digitalWrite(LED_STATUS_POWER, HIGH);
  PORTE |= (1 << PORTE3);  
  // digitalWrite(LED_STATUS_CAN, HIGH);
  PORTE |= (1 << PORTE2);  
  // digitalWrite(CAN_STANDBY, LOW);
  PORTD &= ~(1 << PORTD7);

  v_batt_Mem = adc_read(UB_ANALOG_INPUT_PIN, 20);  //read battery voltage during key on and store it
  
  setHeatLevel(0); /* PWM is initially off. */ 
  
  if (!CAN.begin(1000000)) {
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

void delay_ms(uint32_t ms) {
    ms = 1450 * ms;             // I have 0 idea why this number isn't even close to 16000 when running with a 16MHz clock
    for (ms; ms > 1; ms--) {
       asm("nop");
    }
}  

void LED_Control(uint8_t LED_Name, uint8_t state) {
  if (state == 1) {
    if (LED_Name == LED_STATUS_CAN) PORTE &= ~(1 << PORTE2);
    else PORTE &= ~(1 << PORTE3);  
  }
  else {
    if (LED_Name == LED_STATUS_CAN) PORTE |= (1 << PORTE2);
    else PORTE |= (1 << PORTE3);
  }    
  
}

void setHeatLevel(uint8_t level) {
   OCR0B = level;      
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

//Function for transfering SPI data to the CJ125.
uint16_t COM_SPI(uint32_t TX_data) {
  cli();
  //Configure SPI for CJ125 controller.
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));  
  //Set chip select pin low, chip in use. 
  PORTB &= ~(1 << PORTB2);

  //Transmit request.
  uint32_t Response =  SPI.transfer16(TX_data);
  //Set chip select pin high, chip not in use.
  PORTB |= (1 << PORTB2);
  SPI.endTransaction();
  sei();
  return Response;
}

//Temperature regulating software (PI).
uint8_t heater_control(uint16_t input) {
  
  //Calculate error term.
  int error = (int16_t)sens_resistance_optimal - input;  
  
  //Calculate the integral state with appropriate limiting.
  integ_acc += error;
  if (integ_acc > iMax) integ_acc = iMax;
  if (integ_acc < -iMax) integ_acc = -iMax;
  
  //Calculate proportional term.
  int32_t p_term = -(int32_t)p_gain * error;
      
  //Calculate the integral term.
  int32_t i_term = -(int32_t)i_gain * integ_acc;
  //i_term = i_term/10;
  
  //Calculate regulation (PI).
  int PID = (p_term + i_term) / 10; 
   
  //Set maximum heater output (full power).
  if (PID > 255) PID = 255; 
   
  //Set minimum heater value (cooling).
  if (PID < 0) PID = 0;
  
  //Return calculated PWM output.
  return (uint8_t)PID;
  
}

//Displays the AFR value with narrowband range (0-1V signal) from the DAC. 
// User choice between: Wideband Emulation (0V = AFR 20, 1V = AFR 10) OR Narrowband Emulation.

void update_analog() {
  //Local constants.
  const uint8_t afr = 147;
  const uint16_t max_out = 819; /* 1V */
  const uint8_t min_out = 0;  /* 0V */
  
  //Local variables.
  uint16_t analog_out = 0;
  uint32_t lambda_afr = (uint32_t)lambda_value * afr;
  lambda_afr = lambda_afr / 100;

  if (flagEMU == 1) {  //Narrowband Emulation

    if (lambda_afr > 1500) analog_out = min_out;
    else if (lambda_afr < 1400) analog_out = max_out;
    else analog_out =  max_out - ((1500 - lambda_afr) * 8);
  }
  else {  // Wideband Emulation
    if (lambda_afr >= 2000) analog_out = min_out;      // AFR ≥20 → 0V
    else if (lambda_afr <= 1000) analog_out = max_out; // AFR ≤10 → 1V
    else {
      // Linear map: 10 AFR → 819, 20 AFR → 0
      analog_out = (uint16_t)(((int32_t)(2000 - lambda_afr) * max_out) / 1000);
    }
  }
  //Set DAC output.
  dac.setVoltage(analog_out, false);
  
}

//Convert ADC to Lambda.
uint16_t lookup_lambda(uint16_t input_adc) {  
  
    //Declare and set default return value.
    uint16_t lambda_value = 0;

    //Validate ADC range for lookup table.
    if (input_adc < 104) input_adc = 104;
    if (input_adc > 447) input_adc = 447;

    //Lookup Lambda Value.
    lambda_value = pgm_read_word_near(Lambda_Conversion + (input_adc-104));

    //Return value.
    return lambda_value;  
}

uint16_t lookup_temp(uint16_t input_adc) {
  
    //Declare and set default return value.
    uint16_t nernst_temp = 0;

    //Validate ADC range for lookup table.
    uint32_t resistance = ((uint32_t)input_adc * 300)/sens_resistance_optimal;
    uint16_t rounded_r = ((resistance + 5) / 10) * 10;
    
    
    resistance = (rounded_r - 80) / 10;
    if (resistance > 92) resistance = 920;
    //Lookup Nernst cell temp.
    nernst_temp = pgm_read_word_near(Temp_Conversion + resistance);
    
    //Return value.
    return nernst_temp;
    
}

// --- Phase 1: Wait for supply voltage and CJ125 sensor to be ready ---
void waitForPowerAndSensor(uint16_t battery_volt) {
  // Stay here until supply voltage is OK AND CJ125 reports "status OK"

  while (v_batt < UBAT_MIN || CJ125_Status != CJ125_DIAG_REG_STATUS_OK) {
    // Sample battery voltage
    v_batt = adc_read(UB_ANALOG_INPUT_PIN, 20);   
    
    // Update low battery flag
    if (v_batt < UBAT_MIN) 
      status_flags |= STATUS_LOW_BAT; 
    else 
      status_flags &= ~STATUS_LOW_BAT;

    // Query CJ125 diagnostic register
    CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
    CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

    // Update sensor-related flags
    if (CJ125_Status != CJ125_DIAG_REG_STATUS_OK) {
      if (CJ125_Status == CJ125_DIAG_REG_STATUS_NOSENSOR) 
        status_flags &= ~STATUS_SENSOR;   // sensor not detected
      else 
        status_flags |= STATUS_SHORT_CIRC; // wiring fault or short circuit
    }
    cli();
    wdt_reset();
    sei();    
  }

  while (((battery_volt + 10) > v_batt) && (v_batt < 250) ) {
    status_flags &= ~STATUS_ENGINE_ON;
    v_batt = adc_read(UB_ANALOG_INPUT_PIN, 20);
    cli();
    wdt_reset();
    sei();
    delay_ms(50);
    if (com_flag == 1) sendCAN();
    // debug v_batt += 5;
  }

  status_flags |= STATUS_ENGINE_ON;
  
}

// --- Phase 2: Calibration of CJ125 chip ---
void calibrateCJ125() {
  // Enter calibration mode
  COM_SPI(CJ125_INIT_REG1_MODE_CALIBRATE);
  delay_ms(500);  // let values settle
  sendCAN();

  // Store optimal reference values for lambda and temp
  sens_signal_optimal = adc_read(UA_ANALOG_INPUT_PIN, 20);
  if (sens_signal_optimal != 306) calibration = 306 - sens_signal_optimal;  
  
  sens_resistance_optimal = adc_read(UR_ANALOG_INPUT_PIN, 20);

  // Switch CJ125 into normal operating mode (V=8 gain)
  COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V8);  
  delay_ms(500);
  sendCAN();
}

// --- Phase 3: Heater warmup sequence ---
void heaterWarmup() {
  // Calculate supply voltage in fixed point (e.g. 12.3V → 123)
  v_batt = adc_read(UB_ANALOG_INPUT_PIN, 20);
  uint16_t SupplyVoltage = ((uint32_t)v_batt * 550) / 1023; 

  // === 3a. Condensation phase ===
  // Start with low heater voltage (about 2V) for 5 seconds
  uint8_t UHeater = UHEATER_START;
  uint16_t CondensationPWM = ((uint16_t)UHeater * 255) / SupplyVoltage;
  uint8_t t = 0;
  
  setHeatLevel(CondensationPWM);
  
  while (t < 40 && v_batt > UBAT_MIN) {
    // Blink CAN LED during condensation phase
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    sens_temp = lookup_temp(sens_resistance);
    sendCAN();
    LED_Control(LED_STATUS_POWER, 0);  
    delay_ms(500); 
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    sens_temp = lookup_temp(sens_resistance);
    sendCAN();     
    LED_Control(LED_STATUS_POWER, 1);
    delay_ms(500);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    sens_temp = lookup_temp(sens_resistance);
    sendCAN();

    t++;
    v_batt = adc_read(UB_ANALOG_INPUT_PIN, 20);
    cli();
    wdt_reset();
    sei();
    
  }
  // debug sens_resistance = 910;
  // === 3b. Ramp-up phase ===
  // Increase heater voltage slowly (+0.2V/s) until UR reaches optimum
  UHeater = 75;
  SupplyVoltage = ((uint32_t)v_batt * 550) / 1023;
  //sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
  while (UHeater < UHEATER_MAX && v_batt > UBAT_MIN && sens_resistance > (sens_resistance_optimal+50)) {
    // Convert target voltage into PWM duty cycle    
    CondensationPWM = ((uint16_t)UHeater * 255) / SupplyVoltage;
    if (CondensationPWM > 255) CondensationPWM = 255; // saturate at 100% PWM

    setHeatLevel(CondensationPWM);
    
    // Blink CAN LED during ramp phase
    LED_Control(LED_STATUS_CAN, 0);
    delay_ms(200);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    if (sens_resistance < sens_resistance_optimal) break;
    sens_temp = lookup_temp(sens_resistance);
    sendCAN();
        
    delay_ms(200);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    if (sens_resistance < sens_resistance_optimal) break;
    sens_temp = lookup_temp(sens_resistance);    
    sendCAN();
    
    delay_ms(100);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    if (sens_resistance < sens_resistance_optimal) break;
    sens_temp = lookup_temp(sens_resistance);    
    sendCAN();

    LED_Control(LED_STATUS_CAN, 1);
    delay_ms(200);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    if (sens_resistance < sens_resistance_optimal) break;
    sens_temp = lookup_temp(sens_resistance);    
    sendCAN();
    
    delay_ms(200);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    if (sens_resistance < sens_resistance_optimal) break;
    sens_temp = lookup_temp(sens_resistance);    
    sendCAN();
    
    delay_ms(100);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    if (sens_resistance < sens_resistance_optimal) break;
    sens_temp = lookup_temp(sens_resistance);    
    sendCAN();

    // Update measurements
    v_batt = adc_read(UB_ANALOG_INPUT_PIN, 20);
    // deubg sens_resistance = 91;

    // Increase heater voltage for next cycle
    UHeater += UHEATER_RAMP_STEP;
    cli();
    wdt_reset();
    sei();
  }
  
  // === 3c. Stabilization phase ===
  // Hold heater power until sensor reaches optimal UR value
  while (sens_resistance > sens_resistance_optimal && v_batt > UBAT_MIN) {
    setHeatLevel(CondensationPWM);
    if (com_flag == 1) sendCAN();

    // Update measurements
    v_batt = adc_read(UB_ANALOG_INPUT_PIN, 20);
    sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
    sens_temp = lookup_temp(sens_resistance);
    // deubg sens_resistance = 91;
    cli();
    wdt_reset();
    sei();
  }

  // === 3d. Handover to PID control ===
  // Heater is now warmed up and ready for closed-loop regulation
  LED_Control(LED_STATUS_CAN, 0);   // solid LED = ready
  setHeatLevel(0);                  // PID will take over

}

// --- Main startup sequence (orchestrator) ---
void start() {
  // Initial flags: assume heater and sensor present
  setHeatLevel(0);
  uint16_t v_batt_Mem = adc_read(UB_ANALOG_INPUT_PIN, 20);  //read battery voltage during key on and store it
  
  delay_ms(100);

  // Turn ON power LED
  LED_Control(LED_STATUS_POWER, 1); 
   
  // Run startup phases  
  waitForPowerAndSensor(v_batt_Mem);
  status_flags |= STATUS_SENSOR; 
  sendCAN();
  cli();
  wdt_reset();
  sei();
  calibrateCJ125();
  cli();
  wdt_reset();
  sei();
  status_flags |= STATUS_HEAT; 
  heaterWarmup();
  cli();
  wdt_reset();
  sei();
  
  status_flags &= ~STATUS_HEAT;
  status_flags |= STATUS_NORMAL;
  sendCAN();
  //debug();
}

//Infinite loop.
void loop() {
  LED_Control(LED_STATUS_POWER, 1);
  
  //Update CJ125 diagnostic register from SPI.
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
  CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);

  //Update analog inputs.
  sens_signal = adc_read(UA_ANALOG_INPUT_PIN, 20) + calibration;
  v_batt = adc_read(UB_ANALOG_INPUT_PIN, 20);
  sens_resistance = adc_read(UR_ANALOG_INPUT_PIN, 20);
  // debug sens_resistance = 91;
    
  if (v_batt < UBAT_MIN || CJ125_Status != CJ125_DIAG_REG_STATUS_OK) {
 
    setHeatLevel(0);
    status_flags &= ~STATUS_NORMAL;
    status_flags |= STATUS_HEAT; // force re-init path
    //debug();
    //Serial.print(" restart");
    
    start(); // skip rest of loop    
  }
  status_flags |= STATUS_SENSOR; 
  status_flags &= ~STATUS_HEAT;
  status_flags |= STATUS_NORMAL;
  
  //Adjust PWM output by calculated PID regulation.
  sens_heater_out = heater_control(sens_resistance);  //Calculate and set new heater output.
  setHeatLevel(sens_heater_out); 
 
  //Calculate Lambda Value.
  if (status_flags & STATUS_DEBUG) lambda_value = lookup_lambda(sens_signal_optimal);
  else lambda_value = lookup_lambda(sens_signal);      
  //Calculate Oxygen Content.
  uint32_t afr = ((uint32_t)lambda_value * 147);
  afr = afr/100;
  uint8_t afr_int = afr / 100;      // Get the part before the decimal
  uint8_t afr_decimal = afr % 100;    // Get the part after the decimal
  afr_value = ((uint16_t)afr_int << 8) | (uint16_t)afr_decimal;
  //Update analog output.
  sens_temp = lookup_temp(sens_resistance);
  
  update_analog();

  //delay_ms(10);
  if (com_flag == 1) sendCAN();
  //Serial.print(flagEMU); 
  cli();
  wdt_reset();
  sei();
  //debug();     
}

void sendCAN() {
    uint8_t attempt = 0;

    PORTD &= ~(1 << PORTD7); // set standby low

    while (attempt < MAX_RETRIES) {
      if (CAN.beginPacket(0xFE, 8)) {
        // [0] error flag
        CAN.write(status_flags);

        // [1-2] lambda (high byte, low byte)
        CAN.write(highByte(lambda_value));
        CAN.write(lowByte(lambda_value));

        // [3-4] O2 (high byte, low byte)
        CAN.write(highByte(afr_value));
        CAN.write(lowByte(afr_value));

        // [5-6] sensor temp (high byte, low byte)
        CAN.write(highByte(sens_temp));
        CAN.write(lowByte(sens_temp));

        // [7] battery voltage (already a byte)
        CAN.write(v_batt);

        CAN.endPacket();
        break; // transmission succeeded, exit loop
      }

        attempt++;
        delay_ms(2);  // small delay before retry
    }

    PORTD |= (1 << PORTD7); // release standby

    if (status_flags & STATUS_RECEIVE) status_flags &= ~STATUS_RECEIVE;
    com_flag = 0;
}


ISR(TIMER1_COMPA_vect) {
  com_flag = 1;
  //sendCAN();
}

void packetReceive(uint8_t packetSize) {

  PORTD &= ~(1 << PORTD7);
  if (CAN.packetId() == 0xFF && packetSize == 2) {
    uint8_t i = 0;
    uint8_t buffCAN[2];
 
     while (i < 2 && CAN.available()) {
        buffCAN[i++] = (uint8_t)CAN.read();
    }
    flagEMU = buffCAN[0]; 
    flagDebug = buffCAN[1];       
    if (flagDebug ==  1) status_flags |= STATUS_DEBUG;
    else status_flags &= ~STATUS_DEBUG;
    }   

    PORTD |= (1 << PORTD7);
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
