// Sensor Source Selection (1 = CAN bus from ECU, 0 = Analog hardware timers)
#define           USE_CAN_SENSORS                     1
#define           DEBUG                               1
#define           DME1_CAN_ID                         0x316         /* ECU broadcast frame, 10ms refresh */

//Define pin assignments.
#define           MAC_OUTPUT_PIN                      9             /* PB1/OC1A MAC Valve PWM Output */
#define           UB_ANALOG_INPUT_PIN                 2             /* Analog input for power supply.*/
#define           IAT_ANALOG_INPUT_PIN                1             /* Analog input for temperature.*/
#define           MAP_ANALOG_INPUT_PIN                0             /* Analog input for lambda.*/
#define           CAN_STANDBY                         6             /* PD6 MCP25625 STDBY pin.*/
#define           MAX_RETRIES                         3             /* Max send retries for CAN.*/
#define           LED_STATUS_POWER                    1
#define           LED_STATUS_CAN                      2

//MAP Fault
#define           MAP_ADC_MIN_VALID                   20
#define           MAP_ADC_MAX_VALID                   930
#define           AMBIENT_MAX_MBAR                    1100
#define           AMBIENT_MIN_MBAR                    750

//Status Flag Bits
#define           STATUS_EGT_FAIL                     (1 << 7)
#define           STATUS_ENGINE_ON                    (1 << 6)
#define           STATUS_OVERBOOST                    (1 << 5)
#define           STATUS_AMB_FAULT                    (1 << 4)
#define           STATUS_EGT_ALERT                    (1 << 3)
#define           STATUS_MAP_FAULT                    (1 << 2)
#define           STATUS_VSS_FAULT                    (1 << 1)
#define           STATUS_RPM_FAULT                    (1 << 0)

//Define adjustable parameters.                       
#define           UBAT_MIN                            204            /* Minimum voltage (ADC value) on Ubat to operate. */
#define           MCP_ADDRESS                         0x67
#define           BOOST_CONTROL_MIN_RPM               1500

// Gauge pressure, mbar above boost target
#define           OVERBOOST_CUT_MBAR                  200
#define           OVERBOOST_CLEAR_MBAR                200

// Optional: require throttle lift / RPM drop / target drop later.
// For now this is a simple latch with hysteresis.
#define           OVERBOOST_LATCHED                   1

// Optional effective MAC minimum.
// If your valve does nothing below ~20% duty, use this.
#define           MAC_MIN_EFFECTIVE_DUTY              2000 // 2000 = 20%

//Boost Profile Map Dimensions
#define           BOOST_NUM_PROFILES                  4              /* Number of selectable boost profiles */
#define           BOOST_NUM_GEARS                     6              /* Rows per profile map (gears 1-6) */
#define           BOOST_NUM_RPM_POINTS                15             /* Columns per profile map (RPM breakpoints) */

// Vehicle gears
#define           VEHICLE_NUM_GEARS                   5

//Timer2 Scheduler Tick Flags (5ms base tick)
#define           TICK_FLAG_MAP                       (1 << 0)       /* MAP sensor read — every 10ms (2 ticks) */
#define           TICK_FLAG_SENS                      (1 << 1)       /* RPM/VSS sample — every 10ms (2 ticks) */
#define           TICK_FLAG_BOOST                     (1 << 2)       /* Boost lookup + PI + MAC drive — every 40ms (8 ticks) */
#define           TICK_FLAG_CAN_TX                    (1 << 3)       /* CAN transmit + debug — every 100ms (20 ticks) */
#define           TICK_FLAG_IAT                       (1 << 4)       /* Intake air temp read — every 200ms (40 ticks) */
#define           TICK_FLAG_EGT                       (1 << 5)       /* EGT + ambient temp read — every 1000ms (200 ticks) */

#define           CAN_TIMEOUT_TICKS                   20



void setup(void);
void loop(void);
void start(void);

/* IO / utility */
void delay_ms(uint32_t ms);
void LED_Control(uint8_t LED_Name, uint8_t state);
void setBoostLevel(uint16_t pwm_counts);
uint16_t adc_read(uint8_t channel);

/* Control */
uint16_t boost_control(uint16_t boost_setpoint, uint16_t current_boost, uint16_t rpm);
uint16_t lookup_boost_setpoint(uint8_t gear, uint16_t rpm);
uint8_t calculate_gear(uint16_t rpm, uint16_t vss);

/* Sensors */
int16_t  read_intake_temp(void);
void     read_exhaust_temp(void);
uint16_t read_map(void);
uint16_t read_rpm(void);
uint8_t  read_vss(void);

/* Interpolation / table helpers */
int16_t linear_interp(uint16_t input_adc, const uint16_t *x_axis, const int16_t *y_axis, uint8_t num_points);

/* CAN */
void sendCAN(void);
void packetReceive(int packetSize);



#if DEBUG == 1
  #define DEBUG_CAN_ID_STATUS   0x600
  #define DEBUG_CAN_ID_SENSORS  0x601
  #define DEBUG_CAN_ID_THERMO   0x602
  #define DEBUG_CAN_ID_PWM      0x603
  
  #define DEBUG_FLAG_CAN_FRESH  (1 << 0)
  #define DEBUG_FLAG_MAP_OK     (1 << 1)
  #define DEBUG_FLAG_IAT_OK     (1 << 2)
  #define DEBUG_FLAG_EGT_OK     (1 << 3)
  #define DEBUG_FLAG_RPM_OK     (1 << 4)
  #define DEBUG_FLAG_VSS_OK     (1 << 5)
  uint8_t pwm;
  uint8_t flags = 0;
#endif
