/**************************************************************************/
/*!
    @file     Arduino_Bluetooth_RC_Car.ino
    @author   A. Khan
    @section LICENSE
    Copyright (c) 2015, Art of Circuits (artofcircuits.com)
    All rights reserved.

    This example code utilizes Art of Circuits L298N library (1)
    and uses Android Bluetooth App called: Bluetooth_RC(2) to build a 2 wheel
    and 4 wheel Bluetooth Controlable Robot Car.
    
    External Libraries:
    1. L298N library : https://github.com/artofcircuits/ArtofCircuits_L298n_Library
    
    External Bluetooth Android App:
    2. Bluetooth RC Controller: https://play.google.com/store/apps/details?id=braulio.calle.bluetoothRCcontroller
    (Credit for this app goes to Andi.Co)

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
/**************************************************************************/

#include <artofcircuits_l298n.h>

// one L298N module can drive 2x left side motors in 4WD robot
#define    DCM1_ENABLE    13    // enable input of motor driver 
#define    DCM1_IN1       11     // input-1 of motor driver
#define    DCM1_IN2       10     // input-2 of motor driver

// one L298N module can drive 2x right side motors in 4WD robot
#define    DCM2_ENABLE    12    // enable input of motor driver 
#define    DCM2_IN1       9     // input-1 of motor driver
#define    DCM2_IN2       8     // input-2 of motor driver

#define HORN              7    // Horn output pin

// initialize L298n motor driver instances
L298n dcm1 (DCM1_ENABLE, DCM1_IN1, DCM1_IN2);    // L298n (enable,in1,in2) - left side motors
L298n dcm2 (DCM2_ENABLE, DCM2_IN1, DCM2_IN2);    // L298n (enable,in1,in2) - right side motors

int incomingByte = 0;   // for incoming serial data

void setup()
{
  Serial.begin(9600);
  Serial.println ("Art of Circuits - L298n - dual motor test example");
  pinMode(HORN, OUTPUT);  // inititalizes HORN pin as output  
 
  cmd_horn();      // turns on horn at power up
  delay (100);     // keeps on for 100 msec
  cmd_nohorn ();   // turns off horn
}

void loop ()
{
// send data only when you receive data:
  if (Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = Serial.read();
  
    // say what you got:
    Serial.print("I received: ");
    Serial.write(incomingByte);
    Serial.println();
    
    command_processor (incomingByte);   

  }   
}

///////////////////////////////////////////////////////////////////////////////
// function name: cmd_forward()
// inputs: none
// outputs: none
// Description: move forward
///////////////////////////////////////////////////////////////////////////////
void cmd_forward() {
  dcm1.forward(dcm1.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.forward(dcm2.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255  
}

///////////////////////////////////////////////////////////////////////////////
// function name: cmd_forward_left()
// inputs: none
// outputs: none
// Description: move forward left
///////////////////////////////////////////////////////////////////////////////
void cmd_forward_left() {
  dcm1.forward(dcm1.getSpeed()/4);    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.forward(dcm2.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255  
}



///////////////////////////////////////////////////////////////////////////////
// function name: cmd_forward_right()
// inputs: none
// outputs: none
// Description: move forward left
///////////////////////////////////////////////////////////////////////////////
void cmd_forward_right() {
  dcm1.forward(dcm1.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.forward(dcm2.getSpeed()/4);    // cc/forward motion possible value of pwm can be 0 - 255  
}


///////////////////////////////////////////////////////////////////////////////
// function name: cmd_backward_left()
// inputs: none
// outputs: none
// Description: move forward left
///////////////////////////////////////////////////////////////////////////////
void cmd_backward_left() {
  dcm1.backward(dcm1.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.backward(dcm2.getSpeed()/4);    // cc/forward motion possible value of pwm can be 0 - 255
}


///////////////////////////////////////////////////////////////////////////////
// function name: cmd_backward_right()
// inputs: none
// outputs: none
// Description: move backward left
///////////////////////////////////////////////////////////////////////////////
void cmd_backward_right() {
  dcm1.backward(dcm1.getSpeed()/4);    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.backward(dcm2.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255
}

///////////////////////////////////////////////////////////////////////////////
// function name: cmd_backward()
// inputs: none
// outputs: none
// Description: move backward
///////////////////////////////////////////////////////////////////////////////
void cmd_backward() {
  dcm1.backward(dcm1.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.backward(dcm2.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255  
}

///////////////////////////////////////////////////////////////////////////////
// function name: cmd_left()
// inputs: none
// outputs: none
// Description: turn left
///////////////////////////////////////////////////////////////////////////////
void cmd_left() {
  dcm1.backward(dcm1.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.forward(dcm2.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255  
}


///////////////////////////////////////////////////////////////////////////////
// function name: cmd_right()
// inputs: none
// outputs: none
// Description: turn right
///////////////////////////////////////////////////////////////////////////////
void cmd_right() {
  dcm1.forward(dcm1.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255
  dcm2.backward(dcm2.getSpeed());    // cc/forward motion possible value of pwm can be 0 - 255  
}


///////////////////////////////////////////////////////////////////////////////
// function name: cmd_stop()
// inputs: none
// outputs: none
// Description: Stops motion
///////////////////////////////////////////////////////////////////////////////
void cmd_stop() {
  dcm1.disable();
  dcm2.disable();
}


/////////////////////////////////////////////////////////////////////////////////
//// function name: cmd_speed()
//// inputs: none
//// outputs: none
//// Description: Implements set speed command.
/////////////////////////////////////////////////////////////////////////////////
void cmd_speed (unsigned char speed) 
{
  char *arg;
  dcm1.setSpeed(speed);
  dcm2.setSpeed(speed);  
}


///////////////////////////////////////////////////////////////////////////////
// function name: command_processor()
// inputs: 
// outputs: none
// Description: 
///////////////////////////////////////////////////////////////////////////////
void command_processor (int incomingByte)
{
    unsigned char s = L298N_MAX_SPEED-L298N_MIN_SPEED;
    s = s/10;
    
    switch (incomingByte)
    {
       case 'F':
       cmd_forward();
       break; 
       
       case 'B':
       cmd_backward();
       break; 

       case 'L':
       cmd_left();
       break; 
       
       case 'R':
       cmd_right();
       break; 

       case 'G':      // forward left
       cmd_forward_left();
       break; 

       case 'I':      // forward right
       cmd_forward_right();
       break; 

       case 'H':      // back left
       cmd_backward_left();
       break; 

       case 'J':      // back right
       cmd_backward_right();
       break; 

       case 'S':
       cmd_stop();
       break; 
       
       case '0':
       cmd_speed(0);
       break; 

       case '1':
       cmd_speed(s*1+L298N_MIN_SPEED);
       break; 

       case '2':
       cmd_speed(s*2+L298N_MIN_SPEED);
       break; 

       case '3':
       cmd_speed(s*3+L298N_MIN_SPEED);
       break; 

       case '4':
       cmd_speed(s*4+L298N_MIN_SPEED);
       break; 

       case '5':
       cmd_speed(s*5+L298N_MIN_SPEED);
       break; 

       case '6':
       cmd_speed(s*6+L298N_MIN_SPEED);
       break; 

       case '7':
       cmd_speed(s*7+L298N_MIN_SPEED);
       break; 


       case '8':
       cmd_speed(s*8+L298N_MIN_SPEED);
       break; 

       case '9':
       cmd_speed(s*9+L298N_MIN_SPEED);
       break; 


       case 'q':
       cmd_speed(s*10+L298N_MIN_SPEED);
       break; 

       case 'V':    //horn on
       cmd_horn();
       break; 

       case 'v':    // horn off
       cmd_nohorn();
       break; 
       
       default:
       
       break;
       
    }
}

/////////////////////////////////////////////////////////////////////////////////
//// function name: cmd_horn()
//// inputs: none
//// outputs: none
//// Description: turns on horn
/////////////////////////////////////////////////////////////////////////////////

void cmd_horn (void)
{
  Serial.println ("cmd_horn()");  
  analogWrite (HORN, 127);    // turn ons horn at 50% duty cycle
}

/////////////////////////////////////////////////////////////////////////////////
//// function name: cmd_nohorn()
//// inputs: none
//// outputs: none
//// Description: turns off horn
/////////////////////////////////////////////////////////////////////////////////

void cmd_nohorn ()
{
  Serial.println ("cmd_nohorn()");
  analogWrite (HORN, 0);
  digitalWrite(HORN, LOW);
}

