/**
 * Group 42
 * Authors:
 * Alp Eren Yalcin - 2522126
 * Batuhan Akcan - 2580181
 * Erencan Ceyhan - 2521342
 */

/**
 * In each iteration of the loop, the parser works if the input buffer is not empty
 * and the ADC operation is started if altitude calculation is needed.
 * 
 * RB buttons, TIMER0 timer, ADC and serial communication are handled using interrupts.
 * Buffer operations (push and pop) disable interrupts temporarily since serial
 * interrupts also use them, which may create race conditions.
 * 
 * When $END# message is received, the system resets itself. This is due to the
 * fact that resetting all the variables in END is cumbersome, so that they can be
 * reset using the same initialization code used at the start of the operation.
 * 
 * The parser reads all the characters one by one and uses a simple state machine
 * to parse. PARSE_IDLE corresponds to waiting the start of the next message.
 * PARSE_HEADER corresponds to parsing of the letter part of the message: END, GOO,
 * ALT etc. PARSE_BODY corresponds to parsing of the number part of the message,
 * count of parsing digits being determined using the message type parsed in the header.
 * 
 * ADC is saved as it is and only converted to altitude value when needed. ADC is
 * also only read when altitude period is not 0, checked in adc_task().
 * 
 * RB interrupt also uses a small delay in order to prevent the effects of re-bouncing.
 * 
 * There are a few issues in the code when run with the autopilot simulator. Sometimes
 * the distance message is not sent, maybe due to disabling of the interrupts. The
 * biggest problem frequently (but not always) happening right after the altitude mode
 * is deactivated: a 100ms difference in distance messages. One distance message
 * after altitude disable is not sent and the distance is not subtracted, shifting
 * all distance messages by 100ms. We have not been able to solve this problem and
 * honestly do not have any information about the reason of it.
 * 
 * Also we have encountered a problem that simulator does not give $END# message.
 * If one tries to use the simulator again with our board, they cannot, there will
 * be errors everywhere. The solution is either sending $END# (we used Cutecom separately),
 * or just pressing the hardware reset button (which actually does more-or-less the same thing).
 * 
 * Since we divide ALT messages by 100, technically any multiple of 100ms are legal as
 * altitude period. However, we cannot guarantee that they work.
 */

#include <xc.h>
#include "pragmas.h"
#include "main.h"
#include <stdint.h>

// These are used to disable/enable UART interrupts before/after
// buffering functions are called from the main thread. This prevents
// critical race conditions that result in corrupt buffer data and hence
// incorrect processing


// Disables all interrupts to enforce synchronization

inline void disable_interrupts(void) {
    INTCONbits.GIE = 0;
}

// Enables all interrupts

inline void enable_interrupts(void) {
    INTCONbits.GIE = 1;
}

// Enables TIMER0 module for counting 100ms

inline void enable_timer0() {
    INTCONbits.TMR0IE = 1;
    T0CONbits.TMR0ON = 1;
}

// Disables TIMER0 module

inline void disable_timer0() {
    INTCONbits.TMR0IE = 0;
    T0CONbits.TMR0ON = 0;
}

/* Taken from sample code written by Uluc Saranli */

/* **** Ring-buffers for incoming and outgoing data **** */
// These buffer functions are modularized to handle both the input and
// output buffers with an input argument.

typedef enum {
    INBUF = 0, OUTBUF = 1
} buf_t;

#define BUFSIZE 255     /* Static buffer size. Maximum amount of data */
uint8_t inbuf[BUFSIZE]; /* Preallocated buffer for incoming data */
uint8_t outbuf[BUFSIZE]; /* Preallocated buffer for outgoing data  */
uint8_t head[2] = {0, 0}; /* head for pushing, tail for popping */
uint8_t tail[2] = {0, 0};

/* Check if a buffer had data or not */
#pragma interrupt_level 2 // Prevents duplication of function

uint8_t buf_isempty(buf_t buf) {
    return (head[buf] == tail[buf]) ? 1 : 0;
}
/* Place new data in buffer */
#pragma interrupt_level 2 // Prevents duplication of function

void buf_push(uint8_t v, buf_t buf) {
    if (buf == INBUF) inbuf[head[buf]] = v;
    else outbuf[head[buf]] = v;
    head[buf]++;
    if (head[buf] == BUFSIZE) head[buf] = 0;
}

/* Retrieve data from buffer */
#pragma interrupt_level 2 // Prevents duplication of function

uint8_t buf_pop(buf_t buf) {
    uint8_t v;
    if (buf_isempty(buf)) {
        return 0xFF;
    } else {
        if (buf == INBUF) v = inbuf[tail[buf]];
        else v = outbuf[tail[buf]];
        tail[buf]++;
        if (tail[buf] == BUFSIZE) tail[buf] = 0;
        return v;
    }
}

/* End of Uluc Saranli's code */

/* **** ISR functions **** */

/* Interrupt callback for manual control mode, RB4-7 */
void portb_isr() {

    /* Reading from PORTB may fluctuate if we don't wait */
    __delay_us(1000);

    /* We store current and previous states since the action shall happen after
     * when the button is pressed and released */
    bool current[4]; // Current state of the PORTB
    // Load the current state
    current[0] = PORTBbits.RB4;
    current[1] = PORTBbits.RB5;
    current[2] = PORTBbits.RB6;
    current[3] = PORTBbits.RB7;

    /* If the LED of RBX (i.e. RX0) is on (i.e. portb_enable) and the button was pressed
     * in the previous interrupt and now it is released, we flag that we need to send PRS0X
     */
    if (portb_enable[0] && !current[0] && portb_prev[0]) { // PRS04
        portb_send[0] = true;
    }
    if (portb_enable[1] && !current[1] && portb_prev[1]) { // PRS05
        portb_send[1] = true;
    }
    if (portb_enable[2] && !current[2] && portb_prev[2]) { // PRS06
        portb_send[2] = true;
    }
    if (portb_enable[3] && !current[3] && portb_prev[3]) { // PRS07
        portb_send[3] = true;

    }

    /* Save the current state as the previous for further interrupts */
    portb_prev[0] = current[0];
    portb_prev[1] = current[1];
    portb_prev[2] = current[2];
    portb_prev[3] = current[3];

    INTCONbits.RBIF = 0; // Acknowledge interrupt
}

void receive_isr() {
    /* Clear any error */
    RCSTAbits.OERR = 0;
    RCSTAbits.FERR = 0;

    /* Re-enable receive interrupt in case of an error */
    RCSTAbits.CREN = 1;

    /* Save the received data to the buffer */
    buf_push(RCREG1, INBUF); // Buffer incoming byte

    PIR1bits.RC1IF = 0; // Acknowledge interrupt
}

void transmit_isr() {
    PIR1bits.TX1IF = 0; // Acknowledge interrupt

    if (buf_isempty(OUTBUF)) { // If all bytes are transmitted
        while (!TXSTA1bits.TRMT); // If we remove this line, the last sent character cannot be transmitted, it waits for a couple of ns
        TXSTA1bits.TXEN = 0; // Turn off the transmission
    } else { // Otherwise
        TXREG1 = buf_pop(OUTBUF); // Load next byte to the register
    }
}

void timer_isr() {
    INTCONbits.TMR0IF = 0; // Acknowledge interrupt
    TMR0H = TMR0H_INIT; // initialize TIMER0 value
    TMR0L = TMR0L_INIT; // initialize TIMER0 value

    // Decrement speed from distance in every timer interrupt regardless of which message is sent
    if (dist >= speed)
        dist -= speed;
    else // Do not get below of 0 distance
        dist = 0;

    // Increase the number of sent messages by one to track message count for altitude messages
    counter++;
    /* If altitude_period is 0, since counter is always increased, it will not get into send_altitude if block
     * Otherwise, when the period comes, the send_altitude if block will be executed
     * If the PORTB interrupt callback has flagged that any button was pressed, their message
     * will be sent instead of distance message.
     * If there isn't any waiting message (altitude or button), distance message is sent as usual
     */
    if (altitude_period != PERIOD_0 && counter == altitude_period) {
        send_altitude(adc);
        counter = 0;
    } else if (portb_send[0]) {
        send_button_press(4);
        portb_send[0] = false;
    } else if (portb_send[1]) {
        send_button_press(5);
        portb_send[1] = false;
    } else if (portb_send[2]) {
        send_button_press(6);
        portb_send[2] = false;
    } else if (portb_send[3]) {
        send_button_press(7);
        portb_send[3] = false;
    } else {
        send_distance(dist);
    }

    /* If altitude_period is 0, the send_altitude if block will never be executed;
     * hence, we need to reset the counter explicitly if altitude_period is 0
     */
    if (altitude_period == 0)
        counter = 0;
}

void adc_isr() {
    // Save ADC value to the variable
    adc = (ADRESH << 8) | (ADRESH & 0xF);

    PIR1bits.ADIF = 0; // Acknowledge interrupt
}

void __interrupt(high_priority) highPriorityISR(void) {
    /* Dispatch the interrupt callbacks */
    if (PIR1bits.RC1IF) receive_isr();
    if (PIR1bits.TX1IF) transmit_isr();
    if (INTCONbits.TMR0IF) timer_isr();
    if (INTCONbits.RBIF) portb_isr();
    if (PIR1bits.ADIF) adc_isr();
}

void __interrupt(low_priority) lowPriorityISR(void) {
    // No use for low priority interrupts
}

/* **** Initialization functions **** */

/* Initialize the global variables to 0 in case of reset */
void init_vars() {
    dist = 0;
    altitude_period = PERIOD_0;
    is_manual = false;
    adc = 0;
    counter = 0;
    speed = 0;
    parse_state = PARSE_IDLE;
    message_pos = 0;
    parsed_number = 0;
    digit_count_to_be_parsed = 0;
    parsed_digit_count = 0;
    portb_prev[0] = portb_prev[1] = portb_prev[2] = portb_prev[3] = false;
    portb_enable[0] = portb_enable[1] = portb_enable[2] = portb_enable[3] = false;
    portb_send[0] = portb_send[1] = portb_send[2] = portb_send[3] = false;

    head[INBUF] = 0;
    head[OUTBUF] = 0;
    tail[INBUF] = 0;
    tail[OUTBUF] = 0;
}

/* Initialize the ports */
void init_ports() {
    TRISA = 0b11111110; // RA0 is output, others are input
    TRISB = 0b11111110; // RB0 is output, others are input including RB4-7
    TRISC = 0b11111110; // RC0 is output, others are input
    TRISD = 0b11111110; // RD0 is output, others are input
    TRISH = 0b00010000; // RH4 is input for the ADC
    /* Initialize all the used ports and latches as 0 */
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTH = 0;
    LATA = 0;
    LATB = 0;
    LATC = 0;
    LATD = 0;
    LATH = 0;
}

/* Initialize the serial communication through UCART1 */
void init_serial() {
    /* We will configure EUSART1 for asynchronous, 115200 bps
     * 8-bit baudrate generator (high-speed mode), for a 40MHz crystal.
     * SPBRG = 21 from the table in the manual.
     */
    TXSTA1bits.SYNC = 0;
    BAUDCON1bits.BRG16 = 0;
    TXSTA1bits.BRGH = 1;
    SPBRG1 = 21;

    RCSTA1bits.SPEN = 1; // RC6-7 are used for serial communication
    PIE1bits.TXIE = 1; // Enable interrupt for transmission
    PIE1bits.RCIE = 1; // Enable interrupt for reception
    RCSTAbits.CREN = 1; // Enable reception
}

/* Initialize the interrupt flags */
void init_interrupts() {
    // Enable peripheral interrupts
    INTCONbits.PEIE = 1;

    // Enable reception and transmission interrupts
    PIE1bits.RC1IE = 1;
    PIE1bits.TX1IE = 1;

    // Enable ADC interrupt
    PIE1bits.ADIE = 1;

    // Enable RB button press interrupt
    INTCONbits.RBIE = 1;

    // Enable all interrupts
    enable_interrupts();
}

/* Initialize the ADC */
void init_adc() {
    ADCON0 = 0x31; // Turn on the ADC with Channel 12
    ADCON1 = 0x00; // All pins are analog
    ADCON2 = 0xAA; // Right align, 12 Tad, Fosc/32 sampling
    ADRESH = 0x00; // Zero the conversion result at the start
    ADRESL = 0x00; // Zero the conversion result at the start
}

/* Initialize the TIMER0 */
void init_timer() {
    // Initialize timer that counts 100 ms
    T0CON = 0b00000011; // 16-bit, 1:16 pre-scaler, turned off at start

    /* Initialize the TIMER0 value that will count to 100ms */
    TMR0H = TMR0H_INIT;
    TMR0L = TMR0L_INIT;
}

/* Start system */
void start_system() {
    // Enable the interrupts
    INTCONbits.GIE = 1;
}

/* Function to be called when GOO message is received */
void get_go(uint16_t distance) {
    dist = distance; // Set the distance to the value got in the received message
    enable_timer0(); // Enable 100ms timer
}

/* Function to be called when END message is received */
void get_end() {
    dist = 0; // Zero the distance
    disable_timer0(); // Disable the 100ms timer (we will not send any message anymore)
    RESET(); // Reset the system to clean all the state
}

/* Function to be called when SPD message is received */
void get_speed(uint16_t spd) {
    speed = spd; // Set the speed to the value got in the received message
}

/* Function to be called when ALT message is received */
void get_altitude(uint16_t period) {
    /* AltitudePeriod's values are 0, 2, 4 and 6. These values
     * correspond to 0, 200, 400 and 600ms periods respectively.
     * So we can just divide by 100
     */
    altitude_period = (AltitudePeriod) (period / 100);

    /* Zero the counter */
    counter = 0;
}

/* Function to be called when MAN message is received */
void get_manual(uint8_t activation) {
    is_manual = activation; // Set the manual state
    if (is_manual) {
        INTCONbits.RBIE = 1; // Enable button interrupt if manual state is enabled
    } else {
        INTCONbits.RBIE = 0; // Disable button interrupt if manual state is disabled.
    }
}

/* Function to be called when LED message is received */
void get_led(uint8_t led) {
    switch (led) {
        case 0: // LED00
            // Turn off all LEDs
            LATAbits.LA0 = 0;
            LATBbits.LB0 = 0;
            LATCbits.LC0 = 0;
            LATDbits.LD0 = 0;
            // Disable all buttons
            portb_enable[0] = false;
            portb_enable[1] = false;
            portb_enable[2] = false;
            portb_enable[3] = false;
            break;
        case 1: // LED01
            // Turn on corresponding LED and enable the corresponding port
            LATDbits.LD0 = 1;
            portb_enable[0] = true;
            break;
        case 2: // LED02
            // Turn on corresponding LED and enable the corresponding port
            LATCbits.LC0 = 1;
            portb_enable[1] = true;
            break;
        case 3: // LED03
            // Turn on corresponding LED and enable the corresponding port
            LATBbits.LB0 = 1;
            portb_enable[2] = true;
            break;
        case 4: // LED04
            // Turn on corresponding LED and enable the corresponding port
            LATAbits.LA0 = 1;
            portb_enable[3] = true;
            break;
    }
}

/* Utility function to convert a nibble to hexadecimal character */
char to_hex(uint8_t nibble) {
    if (nibble < 10) { // Digit
        return nibble + '0';
    } else { // Character A-F
        return nibble + 'A' - 10;
    }
    return 0xFF;
}

/* Utility function to convert a hexadecimal character to a nibble */
uint8_t to_nibble(char character) {
    if ('0' <= character && character <= '9') { // 0-9
        return character - '0';
    } else if ('a' <= character && character <= 'f') { // a-f, lowercase
        return character - 'a' + 10;
    } else if ('A' <= character && character <= 'F') { // A-F, uppercase
        return character - 'A' + 10;
    }
    return 0xFF; // Invalid
}

/* Start sending the contents of the OUTBUF */
void send() {
    TXSTA1bits.TXEN = 1; // Enable transmission interrupt, entering the interrupt immediately
}

// The function that writes DIST messages into the buffer

void send_distance(uint16_t distance) {
    // Store the four nibbles of the 16-bit distance
    uint8_t nibble0 = distance & 0xF;
    distance >>= 4;
    uint8_t nibble1 = distance & 0xF;
    distance >>= 4;
    uint8_t nibble2 = distance & 0xF;
    distance >>= 4;
    uint8_t nibble3 = distance & 0xF;

    // While we are pushing some data to the buffer, the buffer should not
    // receive any other data, that's why we disable the interrupts.
    disable_interrupts();

    // Push the message to the buffer
    buf_push('$', OUTBUF);
    buf_push('D', OUTBUF);
    buf_push('S', OUTBUF);
    buf_push('T', OUTBUF);

    // Convert nibbles to corresponding hexadecimal values before pushing into the buffer
    char hex;
    hex = to_hex(nibble3);
    buf_push(hex, OUTBUF);
    hex = to_hex(nibble2);
    buf_push(hex, OUTBUF);
    hex = to_hex(nibble1);
    buf_push(hex, OUTBUF);
    hex = to_hex(nibble0);
    buf_push(hex, OUTBUF);

    buf_push('#', OUTBUF);

    // Enable interrupts in order for data reception to be able to continue
    enable_interrupts();

    // Start sending the message
    send();
}

// Utility function that converts the ADC value (that ranges between 0 and 1023)
// to altitude value (that is either 9000, or 10000, or 11000, or 12000)

uint16_t adc_to_alt(uint16_t value) {
    if (value < 256) {
        return 9000;
    } else if (value < 512) {
        return 10000;
    } else if (value < 768) {
        return 11000;
    } else {
        return 12000;
    }
}

// The function that writes ALT messages into the buffer

void send_altitude(uint16_t adc_value) {
    // Convert adc value to altitude value
    uint16_t alt = adc_to_alt(adc_value);

    // Store the four nibbles of the 16-bit altitude
    uint8_t nibble0 = alt & 0xF;
    alt >>= 4;
    uint8_t nibble1 = alt & 0xF;
    alt >>= 4;
    uint8_t nibble2 = alt & 0xF;
    alt >>= 4;
    uint8_t nibble3 = alt & 0xF;

    // While we are pushing some data to the buffer, the buffer should not
    // receive any other data, that's why we disable the interrupts.
    disable_interrupts();

    // Push the message to the buffer
    buf_push('$', OUTBUF);
    buf_push('A', OUTBUF);
    buf_push('L', OUTBUF);
    buf_push('T', OUTBUF);

    // Convert nibbles to corresponding hexadecimal values before pushing into the buffer
    char hex;
    hex = to_hex(nibble3);
    buf_push(hex, OUTBUF);
    hex = to_hex(nibble2);
    buf_push(hex, OUTBUF);
    hex = to_hex(nibble1);
    buf_push(hex, OUTBUF);
    hex = to_hex(nibble0);
    buf_push(hex, OUTBUF);

    buf_push('#', OUTBUF);

    // Enable interrupts in order for data reception to be able to continue
    enable_interrupts();

    // Start sending the message
    send();
}

// The function that writes PRS messages into the buffer

void send_button_press(uint8_t button) {
    // Store the two nibbles of the 8-bit button value
    uint8_t nibble0 = button & 0xF;
    button >>= 4;
    uint8_t nibble1 = button & 0xF;

    // While we are pushing some data to the buffer, the buffer should not
    // receive any other data, that's why we disable the interrupts.
    disable_interrupts();

    // Push the message to the buffer
    buf_push('$', OUTBUF);
    buf_push('P', OUTBUF);
    buf_push('R', OUTBUF);
    buf_push('S', OUTBUF);

    // Convert nibbles to corresponding hexadecimal values before pushing into the buffer
    char hex;
    hex = to_hex(nibble1);
    buf_push(hex, OUTBUF);
    hex = to_hex(nibble0);
    buf_push(hex, OUTBUF);

    buf_push('#', OUTBUF);

    // Enable interrupts in order for data reception to be able to continue
    enable_interrupts();

    // Start sending the message
    send();
}

// The function that parses received messages

void parse() {
    // While we are popping some data from the buffer, the buffer should not
    // receive any other data, that's why we disable the interrupts.
    disable_interrupts();

    while (!buf_isempty(INBUF)) { // While INBUF is not empty
        char value = buf_pop(INBUF); // Pop the next character from INBUF

        // Since we will not pop from the buffer for a while, we should enable interrupts,
        // because new data may arrive during that time.
        enable_interrupts();

        // The state machine that controls parsing operation
        switch (parse_state) {

                // Currently no message is being received
            case PARSE_IDLE:
                if (value == '$') { // If '$' character received, switch to PARSE_HEADER state
                    parse_state = PARSE_HEADER;
                }
                break;

                // In this state, we only receive the first 3 characters of the message.
                // If correctly received, go to PARSE_BODY state; else, go back to PARSE_IDLE state.
            case PARSE_HEADER:
                message_name[message_pos] = value; // Read the next character of the message into message_name
                message_pos += 1; // Increment message position to read the next character

                if (message_pos == 3) { // If 3 characters were read
                    if (message_name[0] == 'G' && message_name[1] == 'O' && message_name[2] == 'O') { // If GOO characters were read
                        message_type = MT_GO; // Set the message type as MT_GO
                        digit_count_to_be_parsed = 4; // After the GOO message, 4 digits are going to be read, so this is set as 4
                    } else if (message_name[0] == 'E' && message_name[1] == 'N' && message_name[2] == 'D') { // If END characters were read
                        message_type = MT_END; // Set the message type as MT_END
                        digit_count_to_be_parsed = 0; // After the END message, no digits are going to be read, so this is set as 0
                    } else if (message_name[0] == 'S' && message_name[1] == 'P' && message_name[2] == 'D') { // If SPD characters were read
                        message_type = MT_SPEED; // Set the message type as MT_SPEED
                        digit_count_to_be_parsed = 4; // After the SPD message, 4 digits are going to be read, so this is set as 4
                    } else if (message_name[0] == 'A' && message_name[1] == 'L' && message_name[2] == 'T') { // If ALT characters were read
                        message_type = MT_ALTITUDE; // Set the message type as MT_ALTITUDE
                        digit_count_to_be_parsed = 4; // After the ALT message, 4 digits are going to be read, so this is set as 4
                    } else if (message_name[0] == 'M' && message_name[1] == 'A' && message_name[2] == 'N') { // If MAN characters were read
                        message_type = MT_MANUAL; // Set the message type as MT_MANUAL
                        digit_count_to_be_parsed = 2; // After the MAN message, 2 digits are going to be read, so this is set as 2
                    } else if (message_name[0] == 'L' && message_name[1] == 'E' && message_name[2] == 'D') { // If LED characters were read
                        message_type = MT_LED; // Set the message type as MT_LED
                        digit_count_to_be_parsed = 2; // After the LED message, 2 digits are going to be read, so this is set as 2
                    } else { // If the message header is erroneous, go back to PARSE_IDLE state
                        parse_state = PARSE_IDLE;
                        break;
                    }

                    // If the message header was correctly read, go to PARSE_BODY state
                    parse_state = PARSE_BODY;

                    // Reset the variables in order to be re-used
                    message_pos = 0;
                    parsed_digit_count = 0;
                }
                break;

                // In this state, we receive the numeric part of the message, which is 4 characters.
                // If correctly received, the corresponding message handler is called and the state is switched to PARSE_IDLE,
                // otherwise, the state is also switched to PARSE_IDLE.
            case PARSE_BODY:
            {
                bool is_digit = (('0' <= value && value <= '9') || ('A' <= value && value <= 'F') || ('a' <= value && value <= 'f')); // If the received character is a hexadecimal digit, true
                bool is_end = value == '#'; // If the received character is '#', true

                // If the received character is not digit and not end, message is erroneous, so go back to PARSE_IDLE state
                if (!(is_digit || is_end)) {
                    parse_state = PARSE_IDLE;
                    break;
                }

                // If the received character is digit, parse until the parsed_digit_count equals to digit_count_to_be_parsed
                if (is_digit) {
                    // If an enough number of digits were received, and a digit is received again,
                    // the message is erroneous, so go back to PARSE_IDLE state
                    if (parsed_digit_count == digit_count_to_be_parsed) {
                        parse_state = PARSE_IDLE;
                        break;
                    }

                    // Store the received digit as nibble in the parsed_number variable
                    parsed_number <<= 4;
                    parsed_number |= to_nibble(value);

                    // Increment the parsed digit count
                    parsed_digit_count += 1;
                }// The end character '#' is received
                else {
                    // If the end character is received after receiving the correct number of digits,
                    // call the corresponding message handler
                    if (parsed_digit_count == digit_count_to_be_parsed) {
                        switch (message_type) {
                            case MT_GO:
                                get_go(parsed_number);
                                break;
                            case MT_END:
                                get_end();
                                break;
                            case MT_SPEED:
                                get_speed(parsed_number);
                                break;
                            case MT_ALTITUDE:
                                get_altitude(parsed_number);
                                break;
                            case MT_MANUAL:
                                get_manual((uint8_t) (parsed_number & 0xFF)); // Convert uint16_t to uint8_t and then pass it
                                break;
                            case MT_LED:
                                get_led((uint8_t) (parsed_number & 0xFF)); // Convert uint16_t to uint8_t and then pass it
                                break;
                        }
                    }

                    // Reset the variables in order to be able to receive a new message
                    parsed_digit_count = 0;
                    parse_state = PARSE_IDLE;
                    parsed_number = 0;
                }
                break;
            }
        }

        // If the while loop continues, we will be popping some data from the buffer, and the buffer should not
        // receive any other data, that's why we disable the interrupts.
        disable_interrupts();
    }

    // At the end of parsing, enable the interrupts again
    enable_interrupts();
}

// The function that reads the ADC value from the ADRES register

void adc_task() {
    if (altitude_period != PERIOD_0) { // If the altitude period is zero, we will not send the ALT message, so we don't need to use the ADC value.
        ADCON0bits.GODONE = 1;
    }
}

// Main routine

void main(void) {
    // Initialization function calls
    init_vars();
    init_ports();
    init_serial();
    init_interrupts();
    init_adc();
    init_timer();

    // Start the system by enabling global interrupts
    start_system();

    // Parse and ADC tasks
    while (1) {
        parse();
        adc_task();
    }
    return;
}

