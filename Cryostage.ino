//Input duty cycle percentage:
int powerlevel = 0;

//Incoming serial data
int incoming = 0;

//PWM on digital pin3
int IRL540 = 9;
int pwm = 0;


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

#include <math.h>;

void setup(){;
  pinMode(IRL540,OUTPUT);
  Serial.begin(9600);
  Serial.println("Resistor");
delay(1000);

};

void loop(){;

//Receive powerlevel from serial

if (Serial.available() > 0 ){
		incoming = Serial.parseInt();
		powerlevel = incoming;
                pwm = map(powerlevel,0,100,0,255);
}

//Convert input to powerlevel


//pwm = map(incoming,0,100,0,255);			

//Use PWM to run the Peltier


  
analogWrite(IRL540,pwm);

//Calculate temperature from thermistor connected to A0

    signal = analogRead(A0);
    vout = signal;
    vout = (vout/1023)*vin;
    R = R0*(vin/vout)-R0;
   
   Ti = (1/T0)+(1/B)*log(R/R0);
   T = 1/Ti; 
   T = T - 273.15;

//Print the incoming data as a confirmation

		Serial.print("Received powerlevel: ");
		Serial.print(powerlevel);
		Serial.println("%");

//Print temperature and duty cycle
          Serial.print("Duty cycle: ");
 	  Serial.println(pwm);

		Serial.print("Temperature from 100k NTC ");
		Serial.print(T);
		Serial.println(" degrees C");

delay(500);

};
