/*

***************************************************************
* Cryostage control - Version 1.0.0
* by Johannes Lørup Buch <jloerup@gmail.com>
*
* This sketch is licensed under a GPLv3 License and is purely for academic purposes.
***************************************************************

*/


//****************************LIBRARIES****************************


    #include "SPI.h" //SPI Library supplied with the Arduino. A dependency of the Adafruit MAX31855 library.
    #include <PID_v1.h> //The Arduino PID algorithm by Brett Beauregaard, used to control temperature in setpoint mode. https://github.com/br3ttb/Arduino-PID-Library
    #include <math.h> //Math functions used in thermistor calculations.
    #include <Wire.h> //I2C library used for communicating with the LCD.
    #include "rgb_lcd.h" //Grove LCD RGB Backlight library. https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight


//****************************PINS*********************************

//Define MOSFET pin. PWM output from the PID control will go through this pin
    int IRL540 = 6;

//Initiate pins used for buttons on the board and on the box
    int boardButton = 2;
    int freezeButton = 7;
    int PIDButton = 8;
    int loadButton = 9;
    int topButton = 10;
    
//Initiate pins used for status LED and microscope light    
    int freezeLED =13;
    int PIDLED = 12;
    int microscopeLED = 11;
   
 

//************************VARIABLES******************************


//Define PID variables
    double Setpoint, Input, Output;
    PID myPID(&Input, &Output, &Setpoint,5,1,1, REVERSE);

//Define constants and variables for temperature calculation, following Steinhart-Hart.
    float T0 = 298.15;
    float B = 3950;//B-value from the datasheet of the 100k NTC.
    float R0 = 75400;//Measured on 100k resistor with multimeter.
    float Ti = 0;
    float T = 0;
    float R = 0;

//Variables needed for voltage divider at the 100k NTC on A2.
    int signal;
    float vout;
    float vin = 3.30;//Measured between 3.3v(aref) and GND on the Arduino, using a multimeter.

//Initiate the tempstate variable which decides the mode of action for temperature control. The initial value "0" corresponds to load (PWM=0).    
    int tempstate = 0;

//Initiate 16x2 Grove RGB LCD v2.0. The device uses I2C, so SDA is connected to A4 and SCL connected to A5.
    rgb_lcd lcd;
    int colorR = 255;
    int colorG = 0;
    int colorB = 0;

//Variable used to store incoming serial data. The stored value can be used as setpoint for the PID control
    float incoming = 2;

//Voltage divider input on analog pin 2. 
    float potSignal;
    int light;

//Define variables used in timekeeping.
    unsigned long previousTimeLCD = 0;
    unsigned long currentTimeLCD = 0;
    int intervalLCD= 500;

    unsigned long previousTimeSerial = 0;
    unsigned long currentTimeSerial = 0;
    int intervalSerial = 1000;

//**************************SETUP SETUP SETUP SETUP SETUP****************************

void setup(){

    //3.3V (AREF) is used in place of the 5V rail to reduce noise in thermistor readings
        analogReference(EXTERNAL);

    //Define pinmodes for MOSFET, button and optional LED pin
        pinMode(boardButton,INPUT);
        pinMode(freezeButton,INPUT);
        pinMode(PIDButton,INPUT);
        pinMode(loadButton,INPUT);
        pinMode(topButton,INPUT);

        pinMode(freezeLED,OUTPUT);
        pinMode(PIDLED,OUTPUT);
        pinMode(microscopeLED,OUTPUT);

        pinMode(IRL540,OUTPUT);

    //Init PID control
        Input = T;
        Setpoint = 2;

    //Turn on PID
        myPID.SetMode(AUTOMATIC);
  
    //Initiate serial connection and print a test message
        Serial.begin(9600);
        Serial.println("Setup successful");
        delay(500);
        Serial.println("Going to Load");
        delay(500);

    //Start the LCD, say hello and clear
        lcd.begin(16, 2);
        lcd.setRGB(colorR, colorG, colorB);
        lcd.print("Temperature LCD");
        delay(1000);
        lcd.clear();
}

/*
**************************LEWP LEWP LEWP LEWP LEWP LEWP****************************
*/

void loop(){
    
     //Calculate temperature from thermistor connected to A0

        signal = analogRead(A0);
        vout = signal;
        vout = (vout/1023)*vin;
        R = (R0*vout)/(vin-vout);
   
        Ti = (1/T0)+(1/B)*log(R/R0);
        T = 1/Ti; 
        T = T - 273.15;
        

    //BoardButton press sets the temperature state to 3(Potentiometer PID with microscope light)

        if  (digitalRead(boardButton)==HIGH) {
            tempstate = 3;
        }
    
    //LoadButton press sets the tempstate to 0 (Load: PWM=0)

        if  (digitalRead(loadButton)==HIGH) {
            tempstate = 0; 
        }

    //PidButton press sets the tempstate to 1, which is PID control from serial setpoint.

        if  (digitalRead(PIDButton)==HIGH) {
            tempstate = 1;
             
        }

    //Freezebutton press sets the tempstate to 2, which is deep freeze. PWM is 255

        if  (digitalRead(freezeButton)==HIGH) {
            tempstate = 2;
        }
    
    //Topbutton has the same function as boardbutton, it is just at a more convenient position for the user.

        if  (digitalRead(topButton)==HIGH) {
            tempstate = 3;
        }
  

    //Depending on the current tempstate(0-3) the cryostage will do several specific actions.

        switch(tempstate){

            case 0://Tempstate is at the load position, low lights and low cooling
                Output = 20;
                analogWrite(IRL540,Output);
                digitalWrite(PIDLED, LOW);
                digitalWrite(freezeLED, LOW);
                analogWrite(microscopeLED,30);
                colorR = 0;colorG=255;colorB=0;
                lcd.setRGB(colorR, colorG, colorB);
            break;

            case 1://Tempstate is at the PID position, meaning the PID algorithm will run and lights will be turned on
                Setpoint = incoming;
                light = 255;
                digitalWrite(PIDLED,HIGH);
                digitalWrite(freezeLED,LOW);
                analogWrite(microscopeLED,light);
                colorR=255-int(Output);colorG=0;colorB=int(Output);//LCD backlight is set according to cooling power. 255=blue, 0=red.
                lcd.setRGB(colorR, colorG, colorB);
            
                //PID Control with quality control, using the TC value from the beginning of the loop.
                    Input = T;
                    myPID.Compute();
                    analogWrite(IRL540,Output);
            break;

            case 2://Tempstate is deep freeze, meaning the Peltier PWM will be set to 255
                Output = 255;
                analogWrite(IRL540,Output);
                digitalWrite(freezeLED,HIGH);
                digitalWrite(PIDLED,LOW);
                digitalWrite(microscopeLED,HIGH);

                colorR = 0;colorG=0;colorB=255;
                lcd.setRGB(colorR, colorG, colorB);

            break;

            case 3://Tempstate 3 is the same as PID, but setpoint is determined by the potentiometer on A2. The LCD backlight is also set to 30, same as PID mode.
                potSignal = analogRead(A2);   
                potSignal = potSignal - 511;
                potSignal = potSignal/50;
                Setpoint = potSignal;
                Input = T;
                myPID.Compute();
                
                analogWrite(IRL540,Output);
                digitalWrite(PIDLED,LOW);
                digitalWrite(freezeLED, LOW);
                digitalWrite(microscopeLED,HIGH);

                colorR = 255;colorG=255;colorB=255;
                lcd.setRGB(colorR, colorG, colorB);

            break;

    }


    //Receive setpoint from serial, if available. 

        if  (Serial.available() > 0 ){
	        incoming = Serial.parseFloat();//The setpoint is stored in the "incoming" variable. It is only used in tempstate=1
                
        }



 

    //Update the LCD if the intervalLCD time has passed
        
        currentTimeLCD = millis();
        if  ( currentTimeLCD - previousTimeLCD > intervalLCD) {
    
            //Cursor is set for first line and TC temperature is printed
                lcd.setCursor(0,0);
                lcd.print("TR:");
                lcd.setCursor(3,0);
                lcd.print(T);
                lcd.setCursor(8,0);
                lcd.print("State: ");
                lcd.setCursor(15,0);
                lcd.print(tempstate);
         

            //Cursor is set for the second line and setpoint + PWM are printed   
                lcd.setCursor(0,1);
                lcd.print("SP:");
                lcd.setCursor(3,1);
                lcd.print(Setpoint);
                lcd.setCursor(8, 1);
                lcd.print("PWM:");
                lcd.setCursor(12,1);
                lcd.print(Output);

            //One cycle of LCD update has been executed and the timer is reset
                previousTimeLCD = currentTimeLCD ;  

        }

    //Serial communication is on a different timer and is checked like the LCD timer
        currentTimeSerial = millis();
        if  ( currentTimeSerial - previousTimeSerial > intervalSerial ) {

            //Print the incoming data as a confirmation
    	   	    Serial.print("Received setpoint: ");
    	   	    Serial.print(Setpoint);
    	   	    Serial.println(" C");
                    Serial.println(light);
    
            //Print temperature and duty cycle

                Serial.print("100kNTC : ");
                Serial.println(T);
                
            //Reset the Serial timer for next cycle
                previousTimeSerial = currentTimeSerial;  
        }
}

