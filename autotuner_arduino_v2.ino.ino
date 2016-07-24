//*****************************************************************
// Antenna Autotuner with Arduino Mega
// Maria Christopoulou
//
// Function:
// The Arduino measures the SWR, while controlling
// a variable capacitor with a servo motor.
// Then, it selects the capacitor position,
// with the lowest measured SWR.
// 
// Nokia 5110 LCD display pin connections with Arduino Mega 2560:
// CLK - Pin 6
// DIN (MOSI) - Pin 7
// DC - Pin 5
// RST - Pin 3
// CE (Chip Select) - Pin 4
// 
// TODO:
// 1) Add inductor control.
// 2) Check the accuracy of ADC readings. Problem with multiple analog readings?
// 3) Add button function to activate the autotuner, only when needed.
//    For the moment the system is activated with the RESET button.
// 
//
// References:
// 1) Arduino for Ham Radio, Glen Popiel, KW5GP, ARRL, 2014
// 1) http://www.pa3hcm.nl/?p=336
//
//
//*****************************************************************

#include <Servo.h>
#include <LCD5110_Basic.h>

#define adc_count 0.0048828125

Servo servotest; // Define the Servo object

LCD5110 glcd(6,7,5,3,4);  // Define the LCD object

extern uint8_t SmallFont[];  // Define the LCD Font

// Define Variables
float V_fwd=0, V_ref=0, V_SWR;
int pos=0;
int bestCapPos=0;
float bestSWR=1023.000000 * adc_count;
const int capPin=9; // PWM Pin 9 to control the servo
const int buttonPin= 8; //
int buttonState;
int sensorPin0=A0; // Read V_fwd
int sensorPin1=A1; // Read V_ref

void setup() 
{
  glcd.InitLCD(70);  // Initialize the Nokia 5110 Display - set the contrast to 60
  glcd.setFont(SmallFont);

  // Display the Startup screen
  glcd.clrScr();
  glcd.print("SV1PLE", CENTER,0);
  glcd.print("Antenna", CENTER,8);
  glcd.print("autotuner", CENTER, 20);
  delay(3000);
  glcd.clrScr();
  
  servotest.attach(capPin);
  pinMode(capPin, OUTPUT);
      
      servotest.write(0); // Turn capacitor to start position
      delay(2000); 
      // Display Fwd and Ref screen
      glcd.print("Fwd:", 0, 0);
      glcd.print("Ref:", 0, 8);
      
      // The servo loops through multiple positions, while measuring V_fwd and V_ref.
      for (pos=0; pos <= 180; pos+=3)
      {
        servotest.write(pos);
        //
        // When measuring from multiple analog sensors, read them twice.
        // 
        // The ADC multiplexer needs switching time and the voltage need time to stabilize after switching.
        // The first analogRead call causes the multiplexer to switch,
        // the delay gives the voltage time to stabilize,
        // so the second analogRead value is more accurate with less jitter.
        // Reference: http://forum.arduino.cc/index.php?topic=54976.msg428738#msg428738
                
        analogRead(sensorPin0);
        delay(10);
        V_fwd=analogRead(sensorPin0) * adc_count;
        analogRead(sensorPin1);
        delay(10);
        V_ref=analogRead(sensorPin1) * adc_count;

        
        // Measure and print V_SWR
        if ( V_Fwd > V_ref)
        {
          V_SWR = (V_fwd + V_ref)/(V_fwd - V_ref);
          glcd.printNumF(V_fwd, 7, 30, 0);
          glcd.printNumF(V_ref, 7, 30, 8);
          glcd.print("SWR: ", 0, 24);
          glcd.print("         ", 30, 24);
          glcd.printNumF(V_SWR, 1, 30, 24);
          glcd.print(" : 1", 56, 24);
            // If the V_SWR is less than the best V_SWR, assign the new value to best_SWR.
            if ( V_SWR < bestSWR)
            {
               bestSWR = V_SWR;
               bestCapPos = pos;
               servotest.write(bestCapPos);
                      
            }
        }
        
        else
        {
          V_SWR = 0;
        }
        

 
      }
     
      
     
}

void loop()
{


    //  while (digitalRead(buttonPin) == HIGH);
   //}
  // while(1);
}

  
  
  
  
  
  
  
