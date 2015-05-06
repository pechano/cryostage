#include <PID_v1.h>

double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint,1,1,1, REVERSE);

//Input duty cycle percentage:
int powerlevel = 0;

//Incoming serial data
int incoming = 0;

//voltage divider input on analog pin 0
int signal;
float vout;
int vin = 5;

//define constants and variables for temperature calculation, 
//following Steinhart-Hart.

float T0 = 298.15;
float B = 3950;
float R0 = 100000;
float Ti = 0;
float T = 0;
float R = 0;
float ratio = 0;


//for click state

byte state;
int PWM = 0;
int button = 13;
int ledPin = 10;

boolean ledState = LOW;

//initiate pins
int IRL540 = 9;

#include <Bounce.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#include <math.h>

#define DO   6
#define CS   7
#define CLK  8
Adafruit_MAX31855 thermocouple(CLK, CS, DO);

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long previousTimeSerial = 0;
unsigned long currentTimeSerial = 0;
int interval = 500;
int intervalSerial = 5000;
Bounce bouncer = Bounce( button, 5 );


void setup(){;
  pinMode(IRL540,OUTPUT);
  pinMode(button,INPUT);
  pinMode(ledPin,OUTPUT);
  Serial.begin(9600);
  Serial.println("Test");
  
  //Init PID control
  Input = thermocouple.readCelsius();
  Setpoint = 2;
  //Turn on PID
  myPID.SetMode(AUTOMATIC);
  
  lcd.begin(16, 2);
lcd.print("Temperature LCD");
delay(1000);

lcd.clear();

};

void loop(){;

//PID Control

  Input = thermocouple.readCelsius();
  myPID.Compute();
  analogWrite(IRL540,Output);
  


//Receive powerlevel from serial

if (Serial.available() > 0 ){
		incoming = Serial.parseInt();
		Setpoint = incoming;
          //      PWM = map(powerlevel,0,100,0,255);
          //      lcd.clear();
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

currentTime = millis();
currentTimeSerial = millis();
  



if ( currentTime - previousTime > interval ) {
  
//Cursor is set for first line
      
      
      lcd.setCursor(0,0);
      lcd.print("TC:");
      lcd.setCursor(3,0);
      lcd.print(thermocouple.readCelsius());
      lcd.setCursor(8,0);
      lcd.print("TR:");
      lcd.setCursor(11,0);
      lcd.print(T);
      

       
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

previousTime = currentTime;  

}
if ( currentTimeSerial - previousTimeSerial > intervalSerial ) {

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
      
       

previousTimeSerial = currentTimeSerial;  
}
};
