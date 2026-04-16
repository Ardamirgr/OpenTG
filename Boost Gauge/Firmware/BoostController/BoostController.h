// Sensor Source Selection (1 = CAN bus from ECU, 0 = Analog hardware timers)
#define           USE_CAN_SENSORS                     1
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

//Status Flag Bits
#define           STATUS_EGT_FAIL                     (1 << 7)
#define           STATUS_ENGINE_ON                    (1 << 6)
#define           STATUS_PROF_2                       (1 << 5)
#define           STATUS_PROF_1                       (1 << 4)
#define           STATUS_EGT_ALERT                    (1 << 3)
#define           STATUS_MAP_FAULT                    (1 << 2)
#define           STATUS_VSS_FAULT                    (1 << 1)
#define           STATUS_RPM_FAULT                    (1 << 0)

//Define adjustable parameters.                       
#define           UBAT_MIN                            204            /* Minimum voltage (ADC value) on Ubat to operate. */
#define           MCP_ADDRESS                         0x67

//Boost Profile Map Dimensions
#define           BOOST_NUM_PROFILES                  4              /* Number of selectable boost profiles */
#define           BOOST_NUM_GEARS                     6              /* Rows per profile map (gears 1-6) */
#define           BOOST_NUM_RPM_POINTS                15             /* Columns per profile map (RPM breakpoints) */

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
void setBoostLevel(uint8_t level);
uint16_t adc_read(uint8_t channel);

/* Control */
uint8_t boost_control(uint16_t boost_setpoint, uint16_t current_boost, uint16_t rpm);
uint16_t lookup_boost_setpoint(uint8_t gear, uint16_t rpm);
uint8_t calculate_gear(uint16_t rpm, uint16_t vss);

/* Sensors */
int16_t  read_intake_temp(void);
void     read_exhaust_temp(void);
uint16_t current_map(void);
uint16_t read_rpm(void);
uint16_t read_vss(void);

/* Interpolation / table helpers */
int16_t linear_interp(uint16_t input_adc, const uint16_t *x_axis, const int16_t *y_axis, uint8_t num_points);

/* CAN */
void sendCAN(void);
void packetReceive(int packetSize);
