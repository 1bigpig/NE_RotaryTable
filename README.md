# NE_RotaryTable
NextEngine rotary table based on an Arduino Uno using a 28BYJ-48 stepper motor/ULN2003 stepper driver.

NextEngine(c) is property of NextEngine and is not affiliated with this project or code.
All NextEngine data was acquired with an i2c bus sniffer.  No disassembly or code analysis
was performed on any NextEngine hardware or software.  Thanks to dot_bob for helping me 
with some of the i2c data analysis.

Some code based on I2C Scanner from Arduino.cc
Attribution to Krodal, Nick Gammon, Anonymous

This sketch uses the AccelStepper library by Mike McCauley, which can be found:
http://www.airspayce.com/mikem/arduino/AccelStepper/ 
This implementation is easy enough to modify to use other stepper libraries, but I 
chose this one because it can easily be modified to use step/dir or "wave drive" motor control
and it supports acceleration and decceleration.

Basic Rotary table:

![NE_RotaryTableConnector.png](https://github.com/1bigpig/NE_RotaryTable/blob/main/NE_RotaryTableConnector.png)
4pin phone jack

1 GND

2 I2C Clock

3 I2C Data

4 +12v

![NE_uController.png](https://github.com/1bigpig/NE_RotaryTable/blob/main/NE_uController.png)
rotary table Ucontroller
p5	y2	GND
p3	y4	CLOCK
p2	y3	DATA
p1	y1	+12V
