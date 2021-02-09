//  NextEngine rotary table based on an Arduino. 
//
//  NextEngine(c) is property of NextEngine and is not affiliated with this project or code.
//  All NextEngine data was acquired with an i2c bus sniffer.  No disassembly or code analysis
//  was performed on any NextEngine hardware or software.  Thanks to dot_bob for helping me 
//  with some of the i2c data analysis.
//
//  Some code based on I2C Scanner from Arduino.cc
//  Attribution to Krodal, Nick Gammon, Anonymous
//
//  This sketch uses the AccelStepper library by Mike McCauley, which can be found:
//  http://www.airspayce.com/mikem/arduino/AccelStepper/ 
// This implementation is easy enough to modify to use other stepper libraries, but I 
// chose this one because it can easily be modified to use step/dir or "wave drive" motor control
// and it supports acceleration and decceleration
//
// There are multiple styles of coding in this sketch.  I am not a C programmer and it shows.  With 
// that said, I have tried to commented anything that is unclear or needs an explanation.  I am sure
// I missed some things.
//
// All other code and mistakes are mine.
// Bruce Clark 1-30-2021
//
// Update 2/5/2021 expanded destination and IndexMark of 32 bit signed numbers
// Update 2/8/2021 fixed IndexMark math bug and added I2C activity indicator to pin 13 LED
 
#include <Wire.h>
#include <AccelStepper.h>

// Motor pin definitions
#define motorPin1  2     // IN1 on the ULN2003 driver 1
#define motorPin2  3     // IN2 on the ULN2003 driver 1
#define motorPin3  4     // IN3 on the ULN2003 driver 1
#define motorPin4  5     // IN4 on the ULN2003 driver 1
/*
// Swap pin directions for direct solder board to Uno
#define motorPin1  5     // IN1 on the ULN2003 driver 1 
#define motorPin2  4     // IN2 on the ULN2003 driver 1
#define motorPin3  3     // IN3 on the ULN2003 driver 1
#define motorPin4  2     // IN4 on the ULN2003 driver 1
*/

// Response high and low bytes for 0xF5,0x0A commands
                      
uint8_t PICmem[256]={0x02,0x03,0x00,0x01,0x06,0x07,0x04,0x05,0x0A,0x0B,0x08,0x09,0x0E,0x0F,0xE9,0x0D,
                  0x12,0x13,0x10,0xD1,0xEF,0xE9,0xCB,0x23,0xE2,0xEC,0xE3,0xE7,0x69,0x1F,0x1C,0x1D,
                  0xDC,0xDC,0xDE,0xDF,0x26,0x27,0x24,0x25,0x2A,0x2B,0x28,0x29,0xE1,0xA5,0xD3,0xD2,
                  0xCD,0xCC,0xCF,0xCE,0xC9,0xC8,0xCB,0xCA,0xC5,0xC4,0xC7,0xC6,0xC1,0x55,0xFB,0xC2,
                  0xBD,0xD7,0x78,0x41,0xB9,0xB8,0xBB,0x3A,0x4A,0x4B,0x48,0x49,0xB1,0xB0,0xB3,0xB2,
                  0xAD,0xAC,0xAF,0xAE,0xCD,0xA8,0xAB,0xAA,0x2D,0xB7,0xA7,0xA6,0x69,0xA0,0xA3,0xA2,
                  0x9D,0x9C,0x9F,0x9E,0xA9,0xED,0x9B,0x9A,0x15,0x10,0x89,0x96,0x9B,0x90,0x93,0x92,
                  0x72,0x73,0x70,0x71,0x76,0x77,0x74,0x75,0xDA,0xC6,0x82,0x01,0x96,0x84,0xB8,0xC2,
                  0xEA,0xAD,0xFF,0xDA,0x3C,0xFA,0xE0,0xC5,0x6D,0x2F,0x77,0xB2,0xD7,0x77,0x8C,0x4C,
                  0x92,0x93,0x90,0x91,0x96,0x97,0x94,0x95,0x9A,0x9B,0x98,0x99,0x9E,0x9F,0x9C,0x9D,
                  0xA2,0xA3,0xA0,0xA1,0xA6,0xA7,0xA4,0xA5,0xD5,0x89,0x5C,0x56,0xD1,0x8D,0x58,0x52,
                  0x4D,0x4C,0x4F,0x4E,0x49,0x62,0x3E,0x5F,0xBA,0xBB,0xB8,0xB9,0xC1,0xE9,0x40,0x42,
                  0x1D,0x3F,0x3F,0x3E,0x79,0xA4,0x3B,0x3A,0xE5,0x34,0x37,0x36,0x31,0x75,0xDB,0xCD,
                  0x2D,0x97,0x38,0x2E,0xD6,0xD7,0xD4,0xD5,0xDA,0xDB,0xD8,0xD9,0xDE,0xDF,0xDC,0xDD,
                  0xE2,0xE3,0xE0,0xE1,0x19,0x18,0x1B,0x1A,0x15,0x14,0x17,0x16,0xEE,0xEF,0xEC,0xED,
                  0xF2,0xF3,0xF0,0xF1,0xE4,0xE5,0xE6,0xE7,0xFA,0xFB,0xF8,0xF9,0xFE,0xFF,0xFC,0xFD};


  uint8_t         readByte[21];         //commands from NextEgine
  uint8_t         respondByte[21];      //response from Ardunino to NextEngine
  bool            inMotion=false;       //We have a new move command
  bool            locReset=false;       //reset position flag (loc=0)
  bool            firstPass=false;      //Lets start the motion from this command
  signed long     indexLoc=0;           //Where is the turntable
  signed long     moveSteps=0;         //Steps to move
  signed long     moveDegrees=0;         //Steps to move
  signed long     indexMark=0;          //There the index mark position is located

//  There are 97200 steps required to make a full rotation
//  270 steps = 1 degree of table movement

  int             oneDegree=59;        // Number of steps in one degree of table movement [new number of steps per degree 59.41728395061728]
//  long            myStepperRatio=(97200/(4096*(47/9)));   //97200 total steps on NE table revolution, 4096 is one rev on my stepper, 47 teeth table/9 teeth pinion=5.2222
//    long            myStepperRatio=9;    //rounding up as real number is 4.544132313829787
// 
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
// 4096 = 1 full turn @ half-step rotation
// 2048 = 1 full turn @ full-step rotation
// 47 tooth wheel
// 9 tooth pinion

int ledPin = 13;  // LED connected to digital pin 13

AccelStepper stepper(AccelStepper::HALF4WIRE , motorPin1, motorPin3, motorPin2, motorPin4);   //Normal direction
//AccelStepper stepper(AccelStepper::HALF4WIRE , motorPin1, motorPin3, motorPin4, motorPin2);     //Reverse direction

void setup()
{
//  Wire.begin(0x18);                   //0x18 is unknown
//  Wire.begin(0x1B);                   //0x1B is the main 5th axis turntable address (I think)
  Wire.begin(0x1A);                     //0x1A is the main turntable address
  Wire.onRequest(i2cRequest);
  Wire.onReceive(i2cReceiveEvent);         // register event

// stepper motor setup commands -- This will be different for different stepper motors and tables
  stepper.setMaxSpeed(750.0);
  stepper.setAcceleration(200.0);
  stepper.setCurrentPosition(0);          //Reset stepper position
  stepper.setSpeed(600);


  pinMode(ledPin, OUTPUT);  // sets the digital pin 13 as output

}

bool getDataAvailable()
{ return ( Wire.available() > 0 ); }

uint8_t getData()
{ return Wire.read(); }

//basic 8bit xor checksum
int checkSum(int c)    //b is the number of byte to calculate; return checksum value
{
    int b=0;
    for (int a=0;a<=c;a++)
    {
        b^=respondByte[a];
    }
    return b;
}


void i2cRequest()          // #2 Finish request, implement received data
{

  switch (readByte[1])
    {
    case 0x01:            //Distance to move [bytes 6->8 is 24bit "degrees", neg #= CW. pos=CCW] [5f 01 00 08 5c d6 fc ff ff 10 27 15 02]
      respondByte[0]=0xf5;
      respondByte[1]=0x01;
      respondByte[2]=0x00;
      respondByte[3]=0x00;
      respondByte[4]=0x00;
      respondByte[5]=checkSum(4);
      respondByte[6]=0x00;
      respondByte[20]=6;    //number of bytes in response, index=1
      break;

    case 0x05:            //command understood [5f 05 00 07 c5 00 80 1f 03 04 00 00]
      respondByte[0]=0xf5;
      respondByte[1]=0x05;
      respondByte[2]=0x00;
      respondByte[3]=0x00;
      respondByte[4]=0x00;
      respondByte[5]=checkSum(4);
      respondByte[6]=0x00;
      respondByte[20]=6;    //number of bytes in response, index=1
      break;

    case 0x0A:            // UPDATE MEMORY/verify table [0x5F, 0x0A, 0x00, 0x01, 0x54, 0x00] -> F5 0A 00 00 02 02 00 FF
      respondByte[0]=0xf5;
      respondByte[1]=0x0A;
      respondByte[2]=0x00;
      respondByte[3]=0x00;
      respondByte[4]=0x02;
      respondByte[5]=PICmem[readByte[5]];   //data acquired from i2c bus sniffing
      respondByte[6]=readByte[5];
      respondByte[7]=checkSum(6);
      respondByte[8]=0x00;
      respondByte[20]=8;    //number of bytes in response, index=1
      break;

    case 0x82:            //I'm here response [0x5F, 0x82, 0x00, 0x00, 0xDD]
      respondByte[0]=0xf5;
      respondByte[1]=0x82;
      respondByte[2]=0x00;
      respondByte[3]=0x00;
      respondByte[4]=0x09;
      respondByte[5]=0x69;
      respondByte[6] =0x30; //i000601   0x1A 
      respondByte[7] =0x30; //i000610   0x1B --Alternate ID for another axis
      respondByte[8] =0x30;
      respondByte[9] =0x36;
      respondByte[10]=0x30;
      respondByte[11]=0x31;
      respondByte[12]=0x06;
      respondByte[13]=0x11;
      respondByte[14]=checkSum(13);
      respondByte[15]=0x00;
      respondByte[20]=15;    //number of bytes in response, index=1
      break;

    case 0x81:            // in motion/current location [0x5F, 0x81, 0x00, 0x00, 0xDE]
//
//  We are passing the exact same location data at every command.  The NE does not seem to care as long as the "stop" location is different (and correct position) from the
//  previous "stop" command.  Doing this still allows the "rotary table" alignment to work.
//
/*
Example of responses:
r:F5 81 00 00 0A 3A 10 5E 00 00 00 00 00 00 06 0C
r:f5 81 00 00 0a 0d 27 5a 00 00 00 00 00 00 0a 04
r:f5 81 00 00 0a 8f bc 5b 00 00 00 00 00 00 12 04
r:f5 81 00 00 0a e3 c2 5b 00 00 00 00 00 00 00 04
 [F5 81 00 00] ------------------------------------header of report
             [0A] ---------------------------------Number of response bytes [0:##]
                [??]-------------------------------unknown 
                   [## ## ## ##]-------------------Current location in steps +CW /-CCW
                               [## ## ## ##]-------Holds location when index on table is tripped
                                           [00]----status 0x00 stopped/done moving
                                           [06]----status 0x06 starting motion/accellerating
                                           [0A]----status 0x0A moving (issued while in motion)
                                           [12]----status 0x12 decelleration/stabilizing (usually repeated 3 times)
                                              [##]-8bit XOR checksum [0:14]

 */
      
      if (firstPass)                  //First pass "Starting Motion" response
      {
        respondByte[0] =0xF5;
        respondByte[1] =0x81;
        respondByte[2] =0x00;
        respondByte[3] =0x00;
        respondByte[4] =0x0A;
        respondByte[5] =0x7f;                             //unknown byte
        respondByte[6] =(byte) (indexLoc & 0xff);          //low byte
        respondByte[7] =(byte) ((indexLoc >> 8) & 0xff);   //low word
        respondByte[8] =(byte) ((indexLoc >> 16) & 0xff);   //high word
        respondByte[9]=(byte) ((indexLoc >> 24) & 0xff);   //high word
        respondByte[10] =(byte) (indexMark & 0xff);          //low byte
        respondByte[11] =(byte) ((indexMark >> 8) & 0xff);   //low word
        respondByte[12] =(byte) ((indexMark >> 16) & 0xff);   //high word
        respondByte[13]=(byte) ((indexMark >> 24) & 0xff);   //high word
        respondByte[14]=0x06;         //in motion 0a moving, 12 slowing/stopping, 00 stopped, 06 starting
        respondByte[15]=checkSum(14);
        respondByte[16]=0x00;   
        respondByte[20]=16;           //number of bytes in response, index=1

        indexLoc+=moveSteps;           //Update to new table index ultimate position

        firstPass=false;
        inMotion=true;
        break;      
      }

      if (!inMotion)                  // "Stopped" response
      {
        respondByte[0] =0xF5;
        respondByte[1] =0x81;
        respondByte[2] =0x00;
        respondByte[3] =0x00;
        respondByte[4] =0x0A;
        respondByte[5] =0x7f;                             //unknown byte
        respondByte[6] =(byte) (indexLoc & 0xff);          //low byte
        respondByte[7]=(byte) ((indexLoc >> 8) & 0xff);   //low word
        respondByte[8]=(byte) ((indexLoc >> 16) & 0xff);   //high word
        respondByte[9]=(byte) ((indexLoc >> 24) & 0xff);   //high word
        respondByte[10] =(byte) (indexMark & 0xff);          //low byte
        respondByte[11] =(byte) ((indexMark >> 8) & 0xff);   //low word
        respondByte[12] =(byte) ((indexMark >> 16) & 0xff);   //high word
        respondByte[13]=(byte) ((indexMark >> 24) & 0xff);   //high word
        respondByte[14]=0x00;   //in motion 0a moving, 12 slowing/stopping, 00 stopped, 06 starting
        respondByte[15]=checkSum(14);
        respondByte[16]=0x00;   
        respondByte[20]=16;     //number of bytes in response, index=1

        break;
      } else {                  // default "in Motion" command response
      
      respondByte[0] =0xF5;
      respondByte[1] =0x81;
      respondByte[2] =0x00;
      respondByte[3] =0x00;
      respondByte[4] =0x0A;
      respondByte[5] =0x7f;                             //unknown byte
      respondByte[6] =(byte) (indexLoc & 0xff);          //low byte
      respondByte[7]=(byte) ((indexLoc >> 8) & 0xff);   //low word
      respondByte[8]=(byte) ((indexLoc >> 16) & 0xff);   //high word
      respondByte[9]=(byte) ((indexLoc >> 24) & 0xff);   //high word
      respondByte[10] =(byte) (indexMark & 0xff);          //low byte
      respondByte[11] =(byte) ((indexMark >> 8) & 0xff);   //low word
      respondByte[12] =(byte) ((indexMark >> 16) & 0xff);   //high word
      respondByte[13]=(byte) ((indexMark >> 24) & 0xff);   //high word
      respondByte[14]=0x0A;   //in motion 0a moving, 12 slowing/stopping, 00 stopped, 06 starting
      respondByte[15]=checkSum(14);
      respondByte[16]=0x00;   
      respondByte[20]=16;    //number of bytes in response, index=1
      }
    }
    Wire.write(respondByte,respondByte[20]);
    
}

void i2cReceiveEvent (int numBytes)
{
//  digitalWrite(ledPin, 1);  // sets the LED --Slow method
  PORTB ^= _BV(PB5);    //Toggle LED
//  PORTB |= _BV(PB5);    //Set LED
//  PORTB &= ~_BV(PB5);   //Clear LED

  
  int a=0;
  if (numBytes > 0)
  {
    while (numBytes-- )
    {
      readByte[a++]=getData();
    }
  }
  readByte[20]=--a;   //Store how many bytes read in location 20

/*
All commands start with 5F
Second byte determines type of action
0x05 is preamble or data
0x01 is a move command
0x82 am I alive/status command
0x81 Where are you located and in motion?
0x0A 01 THROUGH 0xFF (MEMORY UPDATE or secret code to enable table)
 */
    
/*    
___MOVE COMMAND___
Example of commands:
w: 5f 01 00 08 5f 2a 03 00 00 10 27 02    right turn CW
w: 5f 01 00 08 5c d6 fc ff ff 10 27 15 02   left turn [extra byte in command] CCW
  [F5 81 00 08]---------------------------------header of report
              [??] -----------------------------unknown at this time
                 [## ## ## ##]------------------24bit number in degrees (negative for CCW)
                             [10 27]------------unknown at this time (10000 in decimal: maybe total steps per revolution of table or 10000ms to complete move)
                                   [15]---------if 0x15 then CW.  If this byte is missing it is CCW
                                   [02:02]------unknown
*/

    switch (readByte[1])
    {
      case 0x01:                              //5F 01 00 08 5C D6 FC FF FF 10 27 15 02  -- MOVE command
        moveSteps=(readByte[8]<<24) +(readByte[7]<<16) +(readByte[6]<<8) + readByte[5];    //move amount.  Command is a signed integer and move is relative to current position
        moveDegrees=moveSteps/270;            //convert steps to degrees
        moveDegrees=moveDegrees*oneDegree;    //Convert back to our real number of steps to move
        firstPass=true;                       //first pass through location response
        inMotion=false;                       //Set motion to false--need to issue starting commands before actual move takes place
        stepper.setCurrentPosition(0);        //Reset stepper position to zero, so we can use relative position of command

        signed long index=0;          //index holds the results of Table Location divided by 1/2 table steps
        index=index/48600;            //divide by half of total table steps per rev
        if (index %2) {               //Are we even or odd result?
          indexMark=(index*48600);    //if we are odd, update index mark to current table location
        }
        break;
          
      case 0x0A:                                  //5F 0A 00 01 8D D9                       -- Initialize/update memory/verify ID command
        indexLoc=0;                     //Reset the table index location to zero
        indexMark=0;                     //Reset the table index location to zero
        stepper.setCurrentPosition(0);  //Reset stepper position to zero, so we can use relative position of command
        break;
    }
}
void loop()
{
//Actuall going to do some work here...

    if (inMotion)
    {
     //put move commands here
      stepper.runToNewPosition(moveDegrees);
      stepper.setCurrentPosition(0);          //Reset stepper position
      stepper.disableOutputs();       //Turn coils off
      inMotion=false;
    }
/*/
//    Testing stepper motor speeds and steps per rotation
      stepper.setMaxSpeed(750.0);
//      stepper.setMaxSpeed(200.0);

      stepper.setAcceleration(200.0);
//      stepper.runToNewPosition(-21390);        //Full rotation
      stepper.runToNewPosition(-5347);        //90' rotation
      stepper.setCurrentPosition(0);          //Reset stepper position
//      stepper.setSpeed(1000);
//      delay(1000);
*/
}

