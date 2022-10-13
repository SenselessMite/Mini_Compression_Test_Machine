# Mini_Compression_Test_Machine
Implementation of a Compression test machine made using Teensy 3.1

To assemble it you need the following parts:

For the main circuit:
- teensy 3.1-3.2 for main logic
- 1 linear potentiometer 10kOhm
- SparkFun load cell amplifier HX711 to drive the load cell
- 1 load cell (Model DYLY-103)
- pololu DRV8825 to drive the motor (with heatsink)
- 1 stepper motor (Model 39BYGL215A)
- 1 push button
- 1 two position switch
- 2 1kOhm resistors (pull up for the switch and the push button)
- 2 leds and 2 resistors (the resistance value changes for the led)
- 1 100µF 16v capacitor (for the motor driver)
- 1 7805 voltage regulator (with heatsink)
- 1 1µF capacitor (to stabilize the output of the voltage regulator)
- 1 10µF capacitor (to stabilize the input of the voltage regulator)
- (optional) two position switch to turn on and off the machine
- 12v 2A power source
- jumper

For the wifi module (optional not needed)
- Esp32 wrover module
- Regolatore di Tensione AMS1117 5V
- Lcd display 16 x 2
- 2 10kohm trimmer
- jumper

For the  frame:
- 3d printed parts
- 2 M8 rods (20cm)
- 4 M8 nuts (20cm)
- 6 M8 washers
- 2 8mm rectify rods
- 1 M8 screw (30mm)
- 1 M8 screw (10-15mm)
- 4 M3 screw (15mm)
- 3 M4 screw (30mm)
- 2 M4 nuts
