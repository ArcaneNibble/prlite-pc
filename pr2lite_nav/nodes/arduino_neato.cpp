/*
Arduino Neato XV-11 Laser Distance Scanner
Motor control board v0.2 by Cheng-Lung Lee

XV-11 LDS adapter reads LDS data from RX3 then relay to TX. Also extract the spe
ed
data from the data stream to do speed control on LDS motor. Everythin can power
from USB no extra power required.

Change log:
V0.2 Add simple speed control code update PWM 3 times per rev.
V0.1 Opend loop control version.

This code is tested on Arduino Mega 1280
I/O:
Motor drive by low side driver IPS041L connect to PWM Pin4, Motor power from 5V
Neato XV-11 LDS Vcc(red) : 5V
Neato XV-11 LDS TX(Orange) : RX3
*/

#include <SoftwareSerial.h>

const int DesiredRPM=300; // Setting Desired RPM Here.
const int MotorPWMPin=4;
int inByte = 0; // incoming serial byte
unsigned char Data_status=0;
unsigned char Data_4deg_index=0;
unsigned char Data_loop_index=0;
unsigned char SpeedRPHhighbyte=0; //
unsigned char SpeedRPHLowbyte=0;

int SpeedRPH=0;
const unsigned char PWM4dutyMax=255;
const unsigned char PWM4dutyMin=100;
//unsigned char PWM4duty=PWM4dutyMin; // have to set a default value make motor start spining
unsigned char PWM4duty=140;

// ARD: http://www.arduino.cc/en/Reference/SoftwareSerial
SoftwareSerial Serial3(10, 11); // RX, TX

 void setup() {
    pinMode(MotorPWMPin, OUTPUT);
    // ARD: 57600?
    Serial.begin(115200); // USB serial
    // ARD
    Serial3.begin(115200); // XV-11 LDS data

  // prints title with ending line break
  Serial.println("Arduino Neato XV-11 Motor control board v0.1 by Cheng-Lung Lee");
  
  // Pick your magic number and drive your motor , 178 is 178/255*5V=3.49V
    analogWrite(MotorPWMPin, PWM4duty );
}

void loop() {
    // if we get a valid byte from LDS, read it and send it to USB-serial
  if (Serial3.available() > 0) {
    // get incoming byte:
    inByte = Serial3.read();
    Serial.write(inByte);
    decodeData(inByte);
  }

}

void decodeData(unsigned char inByte){
        # `5A A5 00 C0 XX XX <data>``

  #    0D 0A 00 80 A9 C0
  #    0D 0A 00 80 A9 C0
  # 2A 00 80 00 00 A9 C0
  # 02 92 01 00 80 A9 C0
  #    A9 18 00 00 52 C0
  #    80 52 C0
  #    80 52 C0
  if (inByte==0xC0)
  {
    Data_loop_index=1;
  } else {
    readData(inByte);
    Data_loop_index++;
  }
}
void readData(unsigned char inByte){
  switch (Data_loop_index){
    case 1: // 4 degree index
      // Serial.print(Data_4deg_index, HEX);
      // Serial.print(": ");
      SpeedRPHLowbyte=inByte;
      break;
    case 2: // Speed in RPH low byte
      SpeedRPHhighbyte=inByte;
      SpeedRPH=(SpeedRPHhighbyte<<8)|SpeedRPHLowbyte;
    
      SpeedControl ( DesiredRPM ) ; //
      // Serial.print(SpeedRPHhighbyte, HEX);
      // Serial.println(SpeedRPHLowbyte, HEX);
      break;
    default: // other data
      break;
  }
}


// Very simple speed control
void SpeedControl ( int RPMinput)
{
 if (Data_4deg_index%30==0) { // I only do 3 updat I feel it is good enough for now
  if (SpeedRPH<RPMinput*60)
     if (PWM4duty<PWM4dutyMax) PWM4duty++; // limit the max PWM make sure it don't overflow and make LDS stop working
  if (SpeedRPH>RPMinput*60)
     if(PWM4duty>PWM4dutyMin) PWM4duty--; //Have to limit the lowest pwm keep motor running
  }
 analogWrite(MotorPWMPin, PWM4duty ); // update value
}
