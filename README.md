# Embedded-Aircraft-Autopilot
* Critical Flight Director Program for Aircrafts. It is utilizing USART and ADC modules.

* ADC: analog-to-digital converter

* USART: universal synchronous/asynchronous receiver/transmitter

* Core implementation is in the main.c file

# Device:  
![image](https://github.com/yalcinalp/Assembly-Tetris/assets/95969634/d700872b-76cb-4cf2-bafd-2614f9b04cf8)

# Simulation:  
![Screenshot 2024-08-08 at 01 09 45](https://github.com/user-attachments/assets/d60680f7-8282-4bbd-b52d-5bfe67fa7957)

# Flight Commands:  
![Screenshot 2024-08-08 at 01 10 54](https://github.com/user-attachments/assets/0fbc62a3-fefb-48bf-aa0b-05c0869270ff)

# Commands Continued & Sensor Responses:
![Screenshot 2024-08-08 at 01 11 15](https://github.com/user-attachments/assets/22839c3d-3d0f-4327-81a4-c19a2f450f93)

# ADC Value to Altitude Mapping Table:
![Screenshot 2024-08-08 at 01 12 00](https://github.com/user-attachments/assets/d96fa538-cc3a-4e10-a92c-a032a660cb92)

# Notes & Technical Details:  
* In each iteration of the loop, the parser works if the input buffer is not empty and the ADC operation is started if altitude calculation is needed.
 
* RB buttons, TIMER0 timer, ADC and serial communication are handled using interrupts. 
Buffer operations (push and pop) disable interrupts temporarily since serial interrupts also use them, which may create race conditions.
 
* When $END# message is received, the system resets itself. This is due to the fact that resetting all the variables in END is cumbersome, so that they can be reset using the same initialization code used at the start of the operation.

* The parser reads all the characters one by one and uses a simple state machine to parse. PARSE_IDLE corresponds to waiting the start of the next message. 
PARSE_HEADER corresponds to parsing of the letter part of the message: END, GOO, ALT etc. PARSE_BODY corresponds to parsing of the number part of the message, count of parsing digits being determined using the message type parsed in the header.

* ADC is saved as it is and only converted to altitude value when needed. ADC is also only read when altitude period is not 0, checked in adc_task().

* RB interrupt also uses a small delay in order to prevent the effects of re-bouncing.
