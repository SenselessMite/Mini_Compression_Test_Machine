/*
 * This file includes setting parameters for the machine
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

/*
 * Setting parameters for the stepper motor
 */
const int  MOTOR_STEPS=200; // number of steps for a complete rotatio of the motor
const float PAS=0.01; // [mm] move with each step
const int MCRSTP=32; //microstep level selected

/*
 * Variables used to set the microsteplevel for the DRV8825 driver
 *    - MODE0, MODE1 and MODE2 are used to set the micro step level of the driver:
 *        - MODE0 = LOW, MODE1 = LOW, MODE2 = LOW for full step
 *        - MODE0 = HIGH, MODE1 = LOW, MODE2 = LOW for half step
 *        - MODE0 = LOW, MODE1 = HIGH, MODE2 = LOW for 1/4 step
 *        - MODE0 = HIGH, MODE1 = HIGH, MODE2 = LOW for 1/8 step
 *        - MODE0 = LOW, MODE1 = LOW, MODE2 = HIGH for 1/16 step
 *        - MODE0 = HIGH, MODE1 = LOW, MODE2 = HIGH for 1/32 step
 *        - MODE0 = LOW, MODE1 = HIGH, MODE2 = HIGH for 1/32 step
 *        - MODE0 = HIGH, MODE1 = HIGH, MODE2 = HIGH for 1/32 step
 */
const int MOD0V = HIGH;
const int MOD1V = HIGH;
const int MOD2V = HIGH;

/*
 * Setting parameters for the potentiometer
 */
const float VIN=3.3; // power supply voltage of the potentiometer
const float MDE = 55.0; // maximum range of positions recorded by the potentiometer  [mm]
const float SCE=MDE/4095; //potentiometer scale on 12 bits

/*
 * Setting parameters for the load cell
 */
//const float HX_SC=438.48; // scale for the result in grams
const float HX_SC=44713; // scale for the result in Newton (438.48*1000/9.81)

/*
 * User parameters
 */
//const float F1=1.666667;
const float F1=1;

/*
 * Limit Parameters
 *    - MAXTS is the max value for the sampling time
 *    - MINTS is the min value for the sampling time
 *    - MAXSPEED is the max value for the motor speed
 *    - MINSPEED is the min value for the motor speed
 */
const int MAXTS = 1000;
const int MINTS = 100;
const float MAXSPEED = 55.0;
const float MINSPEED = 5.0;

/*
 * Default parameters
 *    - default value for the parameters SPEED, ES, NC, TS and NN
 */
const float DEF_SPEED = 30.0;
const float DEF_ES = 5.0;
const int DEF_NC = 1;
const int DEF_TS = 500;
const int NN=1400; // dimension for the measurement arrays
#endif
