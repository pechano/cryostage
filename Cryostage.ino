//Define PID variables
  #include <PID_v1.h>
  double Setpoint, Input, Output;
  PID myPID(&Input, &Output, &Setpoint,5,1,1, REVERSE);

//Define constants and variables for temperature calculation, following Steinhart-Hart.
  #include <math.h>
  float T0 = 298.15;
  float B = 3950;
  float R0 = 100000;
  float Ti = 0;
  float T = 0;
  float R = 0;
  float ratio = 0;

//Define variables for click state and optional LED
  #include <Bounce.h>
  int button = 13;
  int ledPin = 10;
  int ledPWM = 0;
  boolean ledState = HIGH;

//Define 16x2 Grove RGB LCD v2.0
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

//Define Adafruit MAX31855 Thermocouple Breakout
  #include "Adafruit_MAX31855.h"
  #include <SPI.h>
  #define DO   6
  #define CS   7
  #define CLK  8
    Adafruit_MAX31855 thermocouple(CLK, CS, DO);
  double TC = thermocouple.readCelsius();
  double oldTC;

//Define MOSFET pin
  int IRL540 = 9;

//Define PWM variable and give initial value
  double PWM = 10;

//Incoming serial data
  int incoming = 0;

//Voltage divider input on analog pin 0
  int signal;
  float vout;
  int vin = 5;

//Define variables used in timekeeping.
  unsigned long previousTimeLCD = 0;
  unsigned long currentTimeLCD = 0;
  int intervalLCD= 500;

  unsigned long previousTimeSerial = 0;
  unsigned long currentTimeSerial = 0;
  int intervalSerial = 5000;
  
  unsigned long previousTimeTC = 0;
  unsigned long currentTimeTC = 0;
  int intervalTC = 200;

  //The bouncer refractory period is also deifned in here
    Bounce bouncer = Bounce( button, 5 );

/*
**************************SETUP SETUP SETUP SETUP SETUP****************************
*/

void setup(){;

  //Define pinmodes for MOSFET, button and optional LED pin
    pinMode(IRL540,OUTPUT);
    pinMode(button,INPUT);
    pinMode(ledPin,OUTPUT);
    Serial.begin(9600);
    Serial.println("Test");
  
    //Supply temperature readout with an arbitrary initial reading to avoid NaN errors
    TC = 10;
    oldTC = TC;

  //Init PID control
    Input = TC;
    Setpoint = 10;
  //Turn on PID
    myPID.SetMode(AUTOMATIC);
  
  //Start the LCD, say hello and clear
    lcd.begin(16, 2);
    lcd.setRGB(colorR, colorG, colorB);
    lcd.print("Temperature LCD");
    delay(1000);
    lcd.clear();



};

/*
**************************LEWP LEWP LEWP LEWP LEWP LEWP****************************
*/

void loop()
{;

  //Buttonpin controls ledPin through ledPWM.
  
    if ( bouncer.update() ) {
    if ( bouncer.read() == HIGH) {
        ledState = !ledState;
      digitalWrite( ledPin, ledState );
    }
}


  //PID Control with quality control. NaN will be rejected.
    currentTimeTC = millis();
    if ( currentTimeTC - previousTimeTC > intervalTC) 
    {
      TC = thermocouple.readCelsius();
        if (isnan(TC)) 
        {
          TC = oldTC;
        };
      Input = TC;
      myPID.Compute();
      analogWrite(IRL540,Output);
      //Set oldTC as a real reading for next loop, in case the TC outputs NaN
        oldTC = TC;
    }

  //Receive setpoint from serial, if available

   if (Serial.available() > 0 )
    {
		  incoming = Serial.parseInt();
		  Setpoint = incoming;
    }

  /*
  //Calculate temperature from thermistor connected to A0

    signal = analogRead(A0);
    vout = signal;
    vout = (vout/1023)*vin;
    R = R0*(vin/vout)-R0;
   
    Ti = (1/T0)+(1/B)*log(R/R0);
    T = 1/Ti; 
    T = T - 273.15;
*/

  //Update the LCD if the intervalLCD time has passed
    currentTimeLCD = millis();
    if ( currentTimeLCD - previousTimeLCD > intervalLCD) 
    {
    
      //Cursor is set for first line and TC temperature is printed
        lcd.setCursor(0,0);
        lcd.print("TC:");
        lcd.setCursor(3,0);
        lcd.print(TC);
        /*  lcd.setCursor(8,0);
        lcd.print("TR:");
        lcd.setCursor(11,0);
        lcd.print(T);
        */  

      //Cursor is set for the second line and setpoint + PWM are printed   
        lcd.setCursor(0,1);
        lcd.print("SP:");
        lcd.setCursor(3,1);
        lcd.print(Setpoint);
        lcd.setCursor(6,1);
        lcd.print("C");
        lcd.setCursor(8, 1);
        lcd.print("PWM:");
        lcd.setCursor(12,1);
        lcd.print(Output);

      //One cycle of LCD update has been executed and the time is set
        previousTimeLCD = currentTimeLCD ;  

    }
  //Serial communication is on a different timer and is checked like the LCD timer
    if ( currentTimeSerial - previousTimeSerial > intervalSerial ) 
    {
      //Print the incoming data as a confirmation

		    Serial.print("Received setpoint: ");
		    Serial.print(Setpoint);
		    Serial.println(" C");

      //Print temperature and duty cycle
        Serial.print("Duty cycle: ");
   	    Serial.println(PWM);
    		Serial.print("Temperature from 100k NTC ");
    		Serial.print(T);
  		  Serial.println(" degrees C");   
        
      //Set the Serial time for next cycle
        previousTimeSerial = currentTimeSerial;  
   }
};
