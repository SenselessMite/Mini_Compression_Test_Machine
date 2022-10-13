/*
 * This file includes the definition for all the pin used to control the machine
 */
#ifndef PINOUT_H
#define PINOUT_H

    /*
     * Status led:
     *    - LED0 : power led
     *    - LED1 : operation led, it will blink when the machine is operationg
     */
    #define LED0 16 
    #define LED1 17
    
    /*
     * ABRT is the pin for the abort button
     * ABRT activate an interrupt that stop the motor
     */
    #define ABRT 18

    /*
     * MODS is used to switch from serial monitor mode to web mode
     *    - if MODS is LOW the machine is in serial monitor mode
     *    - if MODS is HIGH the machine is in web mode
     */
    #define MODS 2

    /*
     * SESP is used to enable or disable the web interface if it is connected
     */
    #define SESP 22
    #define ESINT 19
    /*
     * AP0 is used to read the value from the potentiometer
     */
    #define AP0 A9
    
    /*
     * Driver for the step motor: DRV8825
     * Driver configuration:
     *    - DIR is used to set the direction of rotation
     *    - STEP is used to set send the step signal 
     *    - SLP and RES are used to enable and disable the motor
     *    - MODE0, MODE1 and MODE2 are used to set the micro step level of the driver:
     *        - MODE0 = LOW, MODE1 = LOW, MODE2 = LOW for full step
     *        - MODE0 = HIGH, MODE1 = LOW, MODE2 = LOW for half step
     *        - MODE0 = LOW, MODE1 = HIGH, MODE2 = LOW for 1/4 step
     *        - MODE0 = HIGH, MODE1 = HIGH, MODE2 = LOW for 1/8 step
     *        - MODE0 = LOW, MODE1 = LOW, MODE2 = HIGH for 1/16 step
     *        - MODE0 = HIGH, MODE1 = LOW, MODE2 = HIGH for 1/32 step
     *        - MODE0 = LOW, MODE1 = HIGH, MODE2 = HIGH for 1/32 step
     *        - MODE0 = HIGH, MODE1 = HIGH, MODE2 = HIGH for 1/32 step
     *    - ENBL is used to enable the driver
     */
    #define DIR 5
    #define STEP 6
    #define SLP 7
    #define RES 8
    
    #define MODE0 9
    #define MODE1 10
    #define MODE2 11

    #define ENBL 12
 
    /*
     * Load cell driver: HX711 
     *    - P_DOUT is used to receive the data from the HX711
     *    - P_SCK drives the clock fro the HX711
     */
    #define P_DOUT 14 //digital out
    #define P_SCK 15 //clock

#endif
