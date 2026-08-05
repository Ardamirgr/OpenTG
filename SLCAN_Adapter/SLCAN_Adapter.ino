#include <Arduino.h>
#include <SPI.h>
#include <CAN.h>
#include <avr/wdt.h>
#include <util/atomic.h>

/*
 * ================================================================
 *                     HARDWARE CONFIGURATION
 * ================================================================
 */

#define UART_BAUDRATE          1000000UL
#define CAN_CS_PIN             10
#define CAN_INT_PIN            2


/*
 * The MCP25625 STBY input is normally:
 *   LOW  = normal operation
 *   HIGH = standby
 */
#define CAN_STANDBY_PIN        4
#define CAN_STANDBY_ACTIVE     LOW

/*
 * Maximum complete SLCAN command length.
  * Longest classical CAN command:
  * T1234567881122334455667788
  * 1 command + 8 ID + 1 DLC + 16 data = 26 characters.
 */
#define SLCAN_COMMAND_MAX      32

/*
 * CAN receive queue size.
 * Must be a power of two because indexes wrap using a mask.
 */
#define CAN_RX_QUEUE_SIZE      16
#define CAN_RX_QUEUE_MASK      (CAN_RX_QUEUE_SIZE - 1)

/*
 * ================================================================
 *                         DATA STRUCTURES
 * ================================================================
 */

struct CanFrame
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool extended;
    bool rtr;
};

volatile CanFrame canRxQueue[CAN_RX_QUEUE_SIZE];

volatile uint8_t canRxWriteIndex = 0;
volatile uint8_t canRxReadIndex = 0;

volatile bool canRxOverflow = false;

char commandBuffer[SLCAN_COMMAND_MAX];
uint8_t commandLength = 0;

bool canChannelOpen = false;
bool timestampsEnabled = false;

/*
 * Bitrate selected by the most recent S command.
 */
uint32_t selectedCanBitrate = 500000UL;

/*
 * Timestamp base.
 *
 * Lawicel timestamps are normally four hexadecimal digits representing
 * milliseconds modulo 60000. python-can accepts frames without timestamps,
 * so timestamps are disabled by default.
 */
uint32_t timestampBaseMs = 0;

/*
 * ================================================================
 *                       FUNCTION PROTOTYPES
 * ================================================================
 */

void uartWriteByte(uint8_t value);
void uartWriteText(const char *text);
void uartWriteHexNibble(uint8_t value);
void uartWriteHexByte(uint8_t value);
void uartWriteHexFixed(uint32_t value, uint8_t digits);

int8_t hexToNibble(char character);
bool parseHex(const char *text, uint8_t digits, uint32_t &value);

void sendOk();
void sendError();

void processSerialInput();
void processCommand(const char *command, uint8_t length);

bool openCanChannel(bool listenOnly);
void closeCanChannel();
bool configureCanBitrate (uint32_t bitrate);

void processTransmitCommand (
    const char *command,
    uint8_t length,
    bool extended,
    bool rtr
);

void packetReceive(int packetSize);
void processQueuedCanFrames();
bool dequeueCanFrame(CanFrame &frame);
void outputSlcanFrame(const CanFrame &frame);

uint16_t getSlcanTimestamp();
void clearCanReceiveQueue();

/*
 * ================================================================
 *                              SETUP
 * ================================================================
 */

void setup() {
    /*
     * MCP25625 SPI chip-select and interrupt pins.
     */
    CAN.setPins(CAN_CS_PIN, CAN_INT_PIN);

    /*
     * ============================================================
     *                        GPIO SETUP
     * ============================================================
     */

    DDRC |= 0b00111111;
    DDRE |= (1 << DDE2);
    DDRD |= 0b11010000;

    /*
     * Existing LED/output default states.
     */
    PORTC |= 0b00111111;
    PORTE |= (1 << PE2);
    PORTD |= (1 << PD7);

    /*
     * CAN interrupt pin pull-up.
     *
     * Arduino pin 2 is normally PD2.
     */
    PORTD |= (1 << PORTD2);

    /*
     * MCP25625 transceiver out of standby.
     * Replace CAN_STANDBY_PIN if your STBY input is connected elsewhere.
     */
    pinMode(CAN_STANDBY_PIN, OUTPUT);
    digitalWrite(CAN_STANDBY_PIN, CAN_STANDBY_ACTIVE);

    /*
     * ============================================================
     *                         UART SETUP
     * ============================================================
     */
    Serial.begin(UART_BAUDRATE);

    /*
     * Wait briefly for the FTDI side to settle.
     */
    delay(20);

    /*
     * Keep CAN closed initially. python-can will send:
     *
     *   C
     *   S6
     *   O
     */
    canChannelOpen = false;

    /*
     * Install receive callback now. The callback is only useful after
     * CAN.begin() has successfully initialized the controller.
     */
    CAN.onReceive(packetReceive);

    /*
     * The watchdog is reset continuously in loop().
     */
    wdt_enable(WDTO_4S);
}

/*
 * ================================================================
 *                               LOOP
 * ================================================================
 */

void loop() {
    wdt_reset();

    processSerialInput();
    processQueuedCanFrames();
}

/*
 * ================================================================
 *                         UART FUNCTIONS
 * ================================================================
 */

void uartWriteByte(uint8_t value) {
    Serial.write(value);
}

void uartWriteText(const char *text) {
    Serial.print(text);
}

void uartWriteHexNibble(uint8_t value) {
    value &= 0x0F;

    if (value < 10) {
        uartWriteByte((uint8_t)('0' + value));
    }
    else {
        uartWriteByte((uint8_t)('A' + value - 10));
    }
}

void uartWriteHexByte(uint8_t value) {
    uartWriteHexNibble(value >> 4);
    uartWriteHexNibble(value);
}

void uartWriteHexFixed(uint32_t value, uint8_t digits) {
    while (digits > 0) {
        digits--;

        uint8_t nibble = (uint8_t)(value >> (digits * 4));
        uartWriteHexNibble(nibble);
    }
}

void sendOk() {
    uartWriteByte('\r');
}

void sendError() {
    uartWriteByte('\a');
}

/*
 * ================================================================
 *                          HEX PARSING
 * ================================================================
 */

int8_t hexToNibble(char character) {
    if (character >= '0' && character <= '9') {
        return character - '0';
    }

    if (character >= 'A' && character <= 'F') {
        return character - 'A' + 10;
    }

    if (character >= 'a' && character <= 'f') {
        return character - 'a' + 10;
    }

    return -1;
}

bool parseHex(const char *text, uint8_t digits, uint32_t &value) {
    value = 0;

    for (uint8_t index = 0; index < digits; index++) {
        int8_t nibble = hexToNibble(text[index]);

        if (nibble < 0) {
            return false;
        }

        value = (value << 4) | (uint8_t)nibble;
    }

    return true;
}

/*
 * ================================================================
 *                       SERIAL COMMAND PARSER
 * ================================================================
 */

void processSerialInput() {
    while (Serial.available() > 0) {
        char character = (char)Serial.read();

        /*
         * Commands are terminated by carriage return.
         */
        if (character == '\r') {
            if (commandLength > 0) {
                commandBuffer[commandLength] = '\0';
                processCommand(commandBuffer, commandLength);
            }
            else {
                /*
                 * Empty command.
                 */
                sendError();
            }

            commandLength = 0;
            continue;
        }

        /*
         * Ignore line feeds. Some terminals send CR+LF.
         */
        if (character == '\n') {
            continue;
        }

        if (commandLength < (SLCAN_COMMAND_MAX - 1)) {
            commandBuffer[commandLength++] = character;
        }
        else {
            /*
             * Command was too long. Discard it and report an error.
             */
            commandLength = 0;
            sendError();
        }
    }
}

void processCommand(const char *command, uint8_t length)
{
    switch (command[0]) {
        /*
         * Close CAN channel.
         */
        case 'C': {
            if (length != 1) {
                sendError();
                return;
            }

            closeCanChannel();
            sendOk();
            return;
        }

        /*
         * Open CAN channel.
         */
        case 'O': {
            if (length != 1 || canChannelOpen) {
                sendError();
                return;
            }

            if (openCanChannel(false)) {
                sendOk();
            }
            else {
                sendError();
            }

            return;
        }

        /*
         * Listen-only mode.
         *
         * The Arduino CAN library does not expose a portable listen-only
         * function through its public API, so this currently opens normal
         * mode. GKFlasher does not use listen-only mode.
         */
        case 'L': {
            if (length != 1 || canChannelOpen) {
                sendError();
                return;
            }

            if (openCanChannel(true)) {
                sendOk();
            }
            else {
                sendError();
            }

            return;
        }

        /*
         * Select one of the standard Lawicel bitrates.
         *
         * This command must be issued while the channel is closed.
         */
        case 'S': {
            if (length != 2 || canChannelOpen) {
                sendError();
                return;
            }

            switch (command[1]) {
                case '0':
                    selectedCanBitrate = 10000UL;
                    break;

                case '1':
                    selectedCanBitrate = 20000UL;
                    break;

                case '2':
                    selectedCanBitrate = 50000UL;
                    break;

                case '3':
                    selectedCanBitrate = 100000UL;
                    break;

                case '4':
                    selectedCanBitrate = 125000UL;
                    break;

                case '5':
                    selectedCanBitrate = 250000UL;
                    break;

                case '6':
                    selectedCanBitrate = 500000UL;
                    break;

                /*
                 * The Arduino CAN library does not list 750 kbit/s among
                 * its supported MCP2515 rates, so reject S7.
                 */
                case '7':
                    sendError();
                    return;

                case '8':
                    selectedCanBitrate = 1000000UL;
                    break;

                default:
                    sendError();
                    return;
            }

            sendOk();
            return;
        }

        /*
         * Set custom BTR0/BTR1.
         *
         * Not implemented because the high-level Arduino CAN API does not
         * expose raw CNF register configuration.
         */
        case 's': {
            sendError();
            return;
        }

        /*
         * Standard 11-bit CAN data frame.
         */
        case 't': {
            processTransmitCommand(command, length, false, false);
            return;
        }

        /*
         * Extended 29-bit CAN data frame.
         */
        case 'T': {
            processTransmitCommand(command, length, true, false);
            return;
        }

        /*
         * Standard RTR frame.
         */
        case 'r': {
            processTransmitCommand(command, length, false, true);
            return;
        }

        /*
         * Extended RTR frame.
         */
        case 'R': {
            processTransmitCommand(command, length, true, true);
            return;
        }

        /*
         * Hardware/software version.
         *
         * Format:
         *   Vhhss
         *
         * hh = hardware version
         * ss = software version
         */
        case 'V': {
            if (length != 1) {
                sendError();
                return;
            }

            uartWriteText("V0101\r");
            return;
        }

        /*
         * Serial number.
         */
        case 'N': {
            if (length != 1) {
                sendError();
                return;
            }

            uartWriteText("N0001\r");
            return;
        }

        /*
         * Timestamp enable/disable.
         */
        case 'Z': {
            if (length != 2) {
                sendError();
                return;
            }

            if (command[1] == '0') {
                timestampsEnabled = false;
                sendOk();
            }
            else if (command[1] == '1') {
                timestampsEnabled = true;
                timestampBaseMs = millis();
                sendOk();
            }
            else {
                sendError();
            }

            return;
        }

        /*
         * Status flags.
         *
         * Bit 0 is used here to indicate receive-queue overflow.
         */
        case 'F': {
            if (length != 1) {
                sendError();
                return;
            }

            uint8_t status = 0;

            if (canRxOverflow) {
                status |= 0x01;
            }

            uartWriteByte('F');
            uartWriteHexByte(status);
            uartWriteByte('\r');

            canRxOverflow = false;
            return;
        }

        default: {
            sendError();
            return;
        }
    }
}

/*
 * ================================================================
 *                       CAN CHANNEL CONTROL
 * ================================================================
 */

bool configureCanBitrate(uint32_t bitrate) {
    /*
     * CAN.begin() initializes the MCP25625/MCP2515 and enters normal
     * operating mode.
     */
    if (!CAN.begin(bitrate)) {
        return false;
    }

    CAN.onReceive(packetReceive);

    return true;
}

bool openCanChannel(bool listenOnly) {
    /*
     * listenOnly is intentionally unused because the public library API
     * does not expose MCP2515 listen-only mode.
     */
    (void)listenOnly;

    digitalWrite(CAN_STANDBY_PIN, CAN_STANDBY_ACTIVE);

    clearCanReceiveQueue();

    if (!configureCanBitrate(selectedCanBitrate)) {
        canChannelOpen = false;
        return false;
    }

    canChannelOpen = true;
    timestampBaseMs = millis();

    return true;
}

void closeCanChannel() {
    canChannelOpen = false;

    CAN.onReceive(NULL);
    CAN.end();

    clearCanReceiveQueue();
}

/*
 * ================================================================
 *                         CAN TRANSMISSION
 * ================================================================
 */

void processTransmitCommand(
    const char *command,
    uint8_t length,
    bool extended,
    bool rtr
)
{
    if (!canChannelOpen) {
        sendError();
        return;
    }

    const uint8_t idDigits = extended ? 8 : 3;
    const uint8_t dlcPosition = 1 + idDigits;
    const uint8_t dataPosition = dlcPosition + 1;

    /*
     * Minimum:
     *   command + ID + DLC
     */
    const uint8_t minimumLength = dataPosition;

    if (length < minimumLength) {
        sendError();
        return;
    }

    uint32_t identifier = 0;

    if (!parseHex(&command[1], idDigits, identifier)) {
        sendError();
        return;
    }

    if ((!extended && identifier > 0x7FFUL) ||
        (extended && identifier > 0x1FFFFFFFUL))
    {
        sendError();
        return;
    }

    int8_t dlcNibble = hexToNibble(command[dlcPosition]);

    if (dlcNibble < 0 || dlcNibble > 8) {
        sendError();
        return;
    }

    uint8_t dlc = (uint8_t)dlcNibble;

    /*
     * RTR frames do not contain data bytes.
     */
    uint8_t expectedLength = minimumLength;

    if (!rtr) {
        expectedLength += dlc * 2;
    }

    if (length != expectedLength) {
        sendError();
        return;
    }

    uint8_t data[8] = {0};

    if (!rtr) {
        for (uint8_t index = 0; index < dlc; index++) {
            int8_t highNibble =
                hexToNibble(command[dataPosition + index * 2]);

            int8_t lowNibble =
                hexToNibble(command[dataPosition + index * 2 + 1]);

            if (highNibble < 0 || lowNibble < 0) {
                sendError();
                return;
            }

            data[index] =
                ((uint8_t)highNibble << 4) |
                (uint8_t)lowNibble;
        }
    }

    int beginResult;

    if (extended) {
        beginResult = CAN.beginExtendedPacket(identifier, dlc, rtr);
    }
    else {
        beginResult = CAN.beginPacket(identifier, dlc, rtr);
    }

    if (!beginResult) {
        sendError();
        return;
    }

    if (!rtr && dlc > 0) {
        if (CAN.write(data, dlc) != dlc) {
            sendError();
            return;
        }
    }

    if (!CAN.endPacket()) {
        sendError();
        return;
    }

    /*
     * Lawicel acknowledgement.
     */
    sendOk();
}

/*
 * ================================================================
 *                          CAN RECEPTION
 * ================================================================
 */

/*
 * This callback is called by the CAN library when a packet is available.
 *
 * Do not send serial data directly from here. Copy the packet into a RAM
 * queue and let loop() format it as SLCAN ASCII.
 */
void packetReceive(int packetSize) {
    if (!canChannelOpen)
    {
        /*
         * Consume any bytes left in the current packet.
         */
        while (CAN.available()) {
            CAN.read();
        }

        return;
    }

    uint8_t currentWriteIndex = canRxWriteIndex;
    uint8_t nextWriteIndex =
        (currentWriteIndex + 1) & CAN_RX_QUEUE_MASK;

    if (nextWriteIndex == canRxReadIndex) {
        /*
         * Queue full. Consume the packet so the controller can continue.
         */
        canRxOverflow = true;

        while (CAN.available()) {
            CAN.read();
        }

        return;
    }

    CanFrame frame;

    frame.id = (uint32_t)CAN.packetId();
    frame.extended = CAN.packetExtended();
    frame.rtr = CAN.packetRtr();

    uint8_t dlc = (uint8_t)CAN.packetDlc();

    if (dlc > 8) {
        dlc = 8;
    }

    frame.dlc = dlc;

    for (uint8_t index = 0; index < 8; index++) {
        frame.data[index] = 0;
    }

    if (!frame.rtr) {
        uint8_t index = 0;

        while (CAN.available() && index < dlc) {
            frame.data[index++] = (uint8_t)CAN.read();
        }

        /*
         * Consume any unexpected excess bytes.
         */
        while (CAN.available()) {
            CAN.read();
        }
    }
    else {
        while (CAN.available()) {
            CAN.read();
        }
    }

    canRxQueue[currentWriteIndex] = frame;

    /*
     * Publish the new frame only after it has been fully copied.
     */
    canRxWriteIndex = nextWriteIndex;
}

bool dequeueCanFrame(CanFrame &frame) {
    bool available = false;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (canRxReadIndex != canRxWriteIndex) {
            uint8_t index = canRxReadIndex;

            frame.id = canRxQueue[index].id;
            frame.dlc = canRxQueue[index].dlc;
            frame.extended = canRxQueue[index].extended;
            frame.rtr = canRxQueue[index].rtr;

            for (uint8_t dataIndex = 0; dataIndex < 8; dataIndex++) {
                frame.data[dataIndex] =
                    canRxQueue[index].data[dataIndex];
            }

            canRxReadIndex =
                (canRxReadIndex + 1) & CAN_RX_QUEUE_MASK;

            available = true;
        }
    }

    return available;
}

void processQueuedCanFrames() {
    CanFrame frame;

    while (dequeueCanFrame(frame)) {
        outputSlcanFrame(frame);
    }
}

void outputSlcanFrame(const CanFrame &frame) {
    if (frame.rtr) {
        uartWriteByte(frame.extended ? 'R' : 'r');
    }
    else {
        uartWriteByte(frame.extended ? 'T' : 't');
    }

    if (frame.extended) {
        uartWriteHexFixed(frame.id, 8);
    }
    else {
        uartWriteHexFixed(frame.id, 3);
    }

    uartWriteHexNibble(frame.dlc);

    if (!frame.rtr) {
        for (uint8_t index = 0; index < frame.dlc; index++) {
            uartWriteHexByte(frame.data[index]);
        }
    }

    if (timestampsEnabled) {
        uartWriteHexFixed(getSlcanTimestamp(), 4);
    }

    uartWriteByte('\r');
}

uint16_t getSlcanTimestamp() {
    uint32_t elapsed = millis() - timestampBaseMs;

    /*
     * Traditional Lawicel timestamp range is 0–59999 ms.
     */
    return (uint16_t)(elapsed % 60000UL);
}

void clearCanReceiveQueue() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        canRxWriteIndex = 0;
        canRxReadIndex = 0;
        canRxOverflow = false;
    }
}