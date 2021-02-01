# NE_RotaryTable
NextEngine 3D scanner rotary table based on an Arduino Uno using a 28BYJ-48 stepper motor/ULN2003 stepper driver.

NextEngine(c) is property of NextEngine and is not affiliated with this project or code.\
All NextEngine data was acquired with an i2c bus sniffer.  No disassembly or code analysis\
was performed on any NextEngine hardware or software.  Thanks to dot_bob for helping me \
with some of the i2c data analysis.\

Some code based on I2C Scanner from Arduino.cc.\
Attribution to Krodal, Nick Gammon, Anonymous\

This sketch uses the AccelStepper library by Mike McCauley, which can be found:\
http://www.airspayce.com/mikem/arduino/AccelStepper/ \
This implementation is easy enough to modify to use other stepper libraries, but I \
chose this one because it can easily be modified to use step/dir or "wave drive" motor control\
and it supports acceleration and decceleration.\

# Hardware\
Rotary table:\

![NE_RotaryTableConnector.png](https://github.com/1bigpig/NE_RotaryTable/blob/main/NE_RotaryTableConnector.png)
\
4pin phone jack to connect to NextEngine\
1 GND\
2 I2C Clock\
3 I2C Data\
4 +12v\
\
![NE_uController.png](https://github.com/1bigpig/NE_RotaryTable/blob/main/NE_uController.png)
\
MicroController (PIC 16LF819) on rotary table\
p5	y2	GND\
p3	y4	CLOCK\
p2	y3	DATA\
p1	y1	+12V\
\
# Table Commands\
I2C Addresses:\
0x18 Unknown but the NextEngine seems to call it the most.  No commands came from the NextEngine was it was an i2c "target"\
0x1A Rotary Table -- the address we will be using\
0x1B 5th axis rotary mover.  It appears to be similar to the rotary table commands, but have not investigated.  If you connect\
to Arduinos with similar programming (you have to change the i2c address AND the ID code (see below)) it will enable the\
"Multiaxis drive" and then either crash or shutdown the NextEngine.  Much more work is needs here, but I have no access to a \
Multidrive, so that work with have to wait.\
\
\
I2C commands and responses:\
\
All commands start with 0x5F\
Second byte determines type of action\
0x05 is preamble or data\
0x01 is a move command\
0x82 am I alive/status command\
0x81 Where are you located and in motion?\
0x0A 0x01 THROUGH 0xFF (MEMORY UPDATE or secret code to enable table)\
\
\
\
___MOVE COMMAND___
Example of commands:\
w: 5f 01 00 08 5f 2a 03 00 00 10 27 02    right turn CW\
w: 5f 01 00 08 5c d6 fc ff ff 10 27 15 02   left turn [extra byte in command] CCW\
  [F5 81 00 08]---------------------------------header of report\
              [?? ??] --------------------------unknown at this time\
                    [## ## ##]------------------24bit number in degrees (negative for CCW)\
                             [10 27]------------unknown at this time (10000 in decimal: maybe total steps per revolution of table or 10000ms to complete move)\
                                   [15]---------if 0x15 then CW.  If this byte is missing it is CCW\
                                   [02:02]------unknown\
                                   \
                                   
___LOCATION RESPONSE___
Example of responses:\
r:F5 81 00 00 0A 3A 10 5E 00 00 00 00 00 00 06 0C\
r:f5 81 00 00 0a 0d 27 5a 00 00 00 00 00 00 0a 04\
r:f5 81 00 00 0a 8f bc 5b 00 00 00 00 00 00 12 04\
r:f5 81 00 00 0a e3 c2 5b 00 00 00 00 00 00 00 04\
 [F5 81 00 00] ------------------------------------header of report\
             [0A] ---------------------------------Number of response bytes [0:##]\
                [?? ??]----------------------------unknown \
                      [## ## ##]-------------------Current location in degrees +CW /-CCW\
                    ^^---------[??]----------------unknown (copy of this number +1 when index mark is tripped [maybe not true])\
                                  [## ## ##]-------Holds location when index on table is tripped\
                                           [00]----status 0x00 stopped/done moving\
                                           [06]----status 0x06 starting motion/accellerating\
                                           [0A]----status 0x0A moving (issued while in motion)\
                                           [12]----status 0x12 decelleration/stabilizing (usually repeated 3 times)\
                                              [##]-8bit XOR checksum [0:14]\
\
___UPDATE/VERIFY COMMAND and RESPONSE___
verify table [0x5F, 0x0A, 0x00, 0x01, 0x54, 0x00] -> F5 0A 00 00 02 02 00 FF\
This command responds with a circular ring of bytes.  If these bytes are not correct, the NextEngine will not communicate with the table.\
To simplify things, I just copied the real response codes into an array and send them in the order asked by the NextEngine (sequential).\
\
___Request Table ID COMMAND and RESPONSE___
Get table ID command [0x5F, 0x82, 0x00, 0x00, 0xDD]\
The response requires a valid ID.  Not just any number will do (at least not for me).  So I wrote a quick sketch to find some valid ID:\
Valid:  69 30 30 30 36 30 31  i000601\
Valid:  69 30 30 30 36 31 30  i000610\
Valid:  69 30 30 30 36 32 33  i000623\
Valid:  69 30 30 30 36 33 32  i000632\
Valid:  69 30 30 30 36 34 35  i000645\
Valid:  69 30 30 30 36 35 34  i000654\
Valid:  69 30 30 30 36 36 37  i000667\
Valid:  69 30 30 30 36 37 36  i000676\
Valid:  69 30 30 30 36 38 39  i000689\
Valid:  69 30 30 30 36 39 38  i000698\
\
The correct response looks like this:\
F5 82 00 00 09 [valid id bytes] 06 11 07\
F5 82 00 00 09 69 30 30 30 36 30 31 06 11 07\
\
This is the most common i2c command/reponse.  It seems to act as a "I am alive" message between the NextEngine and the rotary table.\


