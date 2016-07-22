//**************************************************
// Autotuner with Arduino version 2
// Maria Christopoulou
//
// Function:
// The Arduino measures the SWR, while controlling
// a variable capacitor with a servo motor.
// Then, it selects the capacitor position,
// with the lowest measured SWR.
// 
//
// Reference:
// http://www.pa3hcm.nl/?p=336
//
//
//*************************************************

#include <Servo.h>

#define FORWARD_VOLTAGE_THRESH 100
Servo servotest;

//#define adc_count 0.0048828125

//float V_fwd=0, V_ref=0, V_SWR;
int V_fwd=0;
int V_ref = 0;
float V_SWR;
int pos=0;
int bestCapPos=0;
float bestSWR=1023.000000;
const int capPin=9;
const int buttonPin= 8;
int buttonState;
int sensorPin0=A0;
int sensorPin1=A1;

void setup() 
{
    servotest.attach(capPin);
    pinMode(capPin, OUTPUT);
   // pinMode(buttonPin, INPUT);
   // digitalWrite(buttonPin, LOW);
   //Serial.begin(9600);
     buttonState = digitalRead(buttonPin);
 
  // if (buttonState == HIGH)
  // {
      bestCapPos = 0;
      
      bestSWR = 1023.000000;
      servotest.write(0); // Turn capacitor to start position
      delay(2000); 
  
      for (pos=0; pos <= 180; pos+=3)
      {
        servotest.write(pos);
        analogRead(sensorPin0);
        delay(10);
        V_fwd=analogRead(sensorPin0);
        delay(20);
        analogRead(sensorPin1);
        delay(10);
        V_ref=analogRead(sensorPin1);

        V_SWR = (float(V_fwd + V_ref)/float(V_fwd - V_ref));
        if ( V_SWR < bestSWR)
        {
           bestSWR = V_SWR;
           bestCapPos = pos;
           //delay(15);
           
        }
 
      }
      servotest.write(bestCapPos);
      
     
}

void loop()
{


    //  while (digitalRead(buttonPin) == HIGH);
   //}
  // while(1);
}

  
  
  
  
  
  
  
