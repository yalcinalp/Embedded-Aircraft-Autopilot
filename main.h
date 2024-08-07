/* 
 * File:   newfile.h
 * Author: Erencan
 *
 * Created on 27 May?s 2024 Pazartesi, 13:54
 */

#ifndef NEWFILE_H
#define	NEWFILE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
    
#define TMR0H_INIT 11
#define TMR0L_INIT 222

    char to_hex(uint8_t nibble);
    uint8_t to_nibble(char nibble);
    uint8_t from_hex8(char high, char low);
    uint16_t from_hex16(char nibble3, char nibble2, char nibble1, char nibble0);
    uint16_t adc_to_alt(uint16_t value);

    void init_vars();

    void parse();

    void get_go(uint16_t distance);
    void get_end();
    void get_speed(uint16_t speed);
    void get_altitude(uint16_t period);
    void get_manual(uint8_t activation);
    void get_led(uint8_t led);

    void send_distance(uint16_t distance);
    void send_altitude(uint16_t altitude);
    void send_button_press(uint8_t button);
    
    void send();

    typedef enum {
        PERIOD_0 = 0,
        PERIOD_200 = 2,
        PERIOD_400 = 4,
        PERIOD_600 = 6,
    } AltitudePeriod;

    typedef enum {
        PARSE_IDLE,
        PARSE_HEADER,
        PARSE_BODY,
    } ParseState;

    typedef enum {
        MT_GO,
        MT_END,
        MT_SPEED,
        MT_ALTITUDE,
        MT_MANUAL,
        MT_LED,
    } MessageType;

    uint16_t dist;
    AltitudePeriod altitude_period;
    uint8_t counter;
    bool is_manual;
    uint16_t adc;
    uint16_t speed;
    
    ParseState parse_state;
    MessageType message_type;
    char message_name[3];
    uint8_t message_pos;
    uint16_t parsed_number;
    uint8_t digit_count_to_be_parsed;
    uint8_t parsed_digit_count;
    
    bool portb_prev[4];
    bool portb_enable[4];
    bool portb_send[4];


#ifdef	__cplusplus
}
#endif

#endif	/* NEWFILE_H */

