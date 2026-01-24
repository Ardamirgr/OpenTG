#define           TFT_DC                              24
#define           TFT_CS                              9
#define           TFT_RST                             23

#define           BUTTON1                             3
#define           BUTTON2                             5
#define           MENU_BUTTON                         3
#define           ENTER_BUTTON                        2
#define           BACKLIGHT_PIN                       6
#define           CAN_STANDBY                         4

#define           MAX_RETRIES                         10                                /* Max send retries for CAN.*/
#define           SERIAL_RATE                         19200                             /* Serial refresh rate in HZ (1-100). */  
#define           BAUDRATE                            (F_CPU / 16 / SERIAL_RATE) - 1

#define           STATUS_DEBUG                        (1 << 7)
#define           STATUS_ENGINE_ON                    (1 << 6)
#define           STATUS_RECEIVE                      (1 << 5)
#define           STATUS_NORMAL                       (1 << 4)
#define           STATUS_SHORT_CIRC                   (1 << 3)
#define           STATUS_SENSOR                       (1 << 2)
#define           STATUS_HEAT                         (1 << 1)
#define           STATUS_LOW_BAT                      (1 << 0)

#define           ERR_SENS                            0
#define           ERR_SHRT                            1
#define           ERR_BAT                             2
#define           PHOTO_SENSOR                        7
