#define           CJ125_IDENT_REG_REQUEST             0x4800        /* Identify request, gives revision of the chip. */
#define           CJ125_DIAG_REG_REQUEST              0x7800        /* Dignostic request, gives the current status. */
#define           CJ125_INIT_REG1_REQUEST             0x6C00        /* Requests the first init register. */
#define           CJ125_INIT_REG2_REQUEST             0x7E00        /* Requests the second init register. */
#define           CJ125_INIT_REG1_MODE_CALIBRATE      0x569C        /* Sets the first init register in calibration mode. */
#define           CJ125_INIT_REG1_MODE_NORMAL_V8      0x5688        /* Sets the first init register in operation mode. V=8 amplification. */
#define           CJ125_INIT_REG1_MODE_NORMAL_V17     0x5689        /* Sets the first init register in operation mode. V=17 amplification. */
#define           CJ125_DIAG_REG_STATUS_OK            0xE8FF        /* The response of the diagnostic register when everything is ok. */
#define           CJ125_DIAG_REG_STATUS_NOPOWER       0xE855        /* The response of the diagnostic register when power is low. */
#define           CJ125_DIAG_REG_STATUS_NOSENSOR      0xE87F        /* The response of the diagnostic register when no sensor is connected. */
#define           CJ125_INIT_REG1_STATUS_0            0xE888        /* The response of the init register when V=8 amplification is in use. */
#define           CJ125_INIT_REG1_STATUS_1            0xE889        /* The response of the init register when V=17 amplification is in use. */

//Define pin assignments.
#define           CJ125_NSS_PIN                       10            /* Pin used for chip select in SPI communication. */
#define           LED_STATUS_POWER                    26            /* Pin used for power the status LED, indicating we have power. */
#define           LED_STATUS_CAN                      25            /* Pin used for the heater status LED, indicating heater activity. */
#define           HEATER_OUTPUT_PIN                   5             /* Pin used for the PWM output to the heater circuit. */
#define           ANALOG_OUTPUT_PIN                   3             /* Pin used for the PWM to the 0-1V analog output. */
#define           UB_ANALOG_INPUT_PIN                 2             /* Analog input for power supply.*/
#define           UR_ANALOG_INPUT_PIN                 1             /* Analog input for temperature.*/
#define           UA_ANALOG_INPUT_PIN                 0             /* Analog input for lambda.*/
#define           CAN_STANDBY                         7             /* MCP25625 STDBY pin.*/
#define           MAX_RETRIES                         3             /* Max send retries for CAN.*/

//Status Flag Bits
#define           STATUS_DEBUG                        (1 << 7)
#define           STATUS_ENGINE_ON                    (1 << 6)
#define           STATUS_RECEIVE                      (1 << 5)
#define           STATUS_NORMAL                       (1 << 4)
#define           STATUS_SHORT_CIRC                   (1 << 3)
#define           STATUS_SENSOR                       (1 << 2)
#define           STATUS_HEAT                         (1 << 1)
#define           STATUS_LOW_BAT                      (1 << 0)

//Define adjustable parameters.

#define           UHEATER_START                       20             /* Start value of UHeater.*/
#define           UHEATER_MAX                         120            /* Max value of UHeater.*/
#define           UHEATER_RAMP_STEP                   2              /* Ramp step of UHeater.*/                         
#define           UBAT_MIN                            204            /* Minimum voltage (ADC value) on Ubat to operate. */
