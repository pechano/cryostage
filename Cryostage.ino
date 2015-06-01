//****************************LIBRARIES****************************


    #include "SPI.h" //SPI Library supplied with the Arduino. A dependency of the Adafruit MAX31855 library.
    #include "MAX31855.h" //Library by Rob Tillaart. Used for communicating with the MAX31855 chip. https://github.com/RobTillaart/Arduino/tree/master/libraries/MAX31855
    #include <PID_v1.h> //The Arduino PID algorithm by Brett Beauregaard, used to control temperature in setpoint mode. https://github.com/br3ttb/Arduino-PID-Library
    #include <math.h> //Math functions used in thermistor calculations.
    #include <Wire.h> //I2C library used for communicating with the LCD.
    #include "rgb_lcd.h" //Grove LCD RGB Backlight library. https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight


//****************************PINS*********************************


//Initiate Adafruit MAX31855 Thermocouple Breakout pins. SPI Device.
    int thermoCLK = 5;
    int thermoCS = 4;
    int thermoDO = 3;

//Define MOSFET pin. PWM output from the PID control will go through this pin
    int IRL540 = 6;

//Initiate pins used for buttons on the board and on the box
    int boardButton = 2;
    int freezeButton = 7;
    int PIDButton = 8;
    int loadButton = 9;
    int topButton = 10;
    
//Initiate pins used for status LED and microscope light    
    int freezeLED =11;
    int PIDLED = 12;
    int microscopeLED = 13;


// Initialize the Thermocouple
    MAX31855 tc(thermoCLK, thermoCS, thermoDO);

 

//************************VARIABLES******************************


//Define PID variables
    double Setpoint, Input, Output;
    PID myPID(&Input, &Output, &Setpoint,5,1,1, REVERSE);

//Define constants and variables for temperature calculation, following Steinhart-Hart.
    float T0 = 298.15;
    float B = 3950;//B-value from the datasheet of the 100k NTC.
    float R0 = 101050;//Measured on 100k resistor with multimeter.
    float Ti = 0;
    float T = 0;
    float R = 0;

//Variables needed for voltage divider at the 100k NTC on A2.
    int signal;
    float vout;
    float vin = 4.97;//Measured between 5v and GND on the Arduino, using a multimeter.

//Initiate the tempstate variable which decides the mode of action for temperature control. The initial value "0" corresponds to load (PWM=0).    
    int tempstate = 0;

//Initiate 16x2 Grove RGB LCD v2.0. The device uses I2C, so SDA is connected to A4 and SCL connected to A5.
    rgb_lcd lcd;
    int colorR = 255;
    int colorG = 0;
    int colorB = 0;

//Initiate the variables that will be used for storing temperature readouts. oldTC is the latest numeric readout, should the thermocouple return NaN.
    float TC = tc.getTemperature();

//Supply temperature readout with an arbitrary initial reading to avoid NaN errors
    float oldTC = 10.0;

//Variable used to store incoming serial data. The stored value can be used as setpoint for the PID control
    float incoming = 10.0;

//Voltage divider input on analog pin 2. 
    int potSignal;

//Define variables used in timekeeping.
    unsigned long previousTimeLCD = 0;
    unsigned long currentTimeLCD = 0;
    int intervalLCD= 500;

    unsigned long previousTimeSerial = 0;
    unsigned long currentTimeSerial = 0;
    int intervalSerial = 1000;
  
    unsigned long previousTimeTC = 0;
    unsigned long currentTimeTC = 0;
    int intervalTC = 300;

//**************************SETUP SETUP SETUP SETUP SETUP****************************

void setup(){

    //Start the setup by getting a real thermocouple reading. There is one fallback reading, but a NaN will at that point set TC=oldTC.
        if  (isnan(TC)) {
            tc.read();
            TC = tc.getTemperature();
            if (isnan(TC)) {
                TC = oldTC;
            }
                        
        }

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
        Input = TC;
        Setpoint = 10;

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
};

/*
**************************LEWP LEWP LEWP LEWP LEWP LEWP****************************
*/

void loop(){
    
     //Calculate temperature from thermistor connected to A0

        signal = analogRead(A0);
        vout = signal;
        vout = (vout/1023)*vin;
        R = R0*(vin/vout)-R0;
   
        Ti = (1/T0)+(1/B)*log(R/R0);
        T = 1/Ti; 
        T = T - 273.15;
        
      
    //If intervalTC time has passed, a readout will be stored in the TC variable. NaN will result in an extra reading before finally using oldTC if the problem persists

        currentTimeTC = millis();
        if  (currentTimeTC - previousTimeTC > intervalTC) {
            tc.read();
            TC = tc.getTemperature();
            if  (isnan(TC)) {
                tc.read();
                TC = tc.getTemperature();
                if (isnan(TC)) {
                    TC = oldTC;
                }
                
            }
            oldTC = TC;
            previousTimeTC = currentTimeTC;
        }


    //BoardButton press sets the temperature state to 3(Load with microscope light)

        if  (digitalRead(boardButton)==HIGH) {
            tempstate = 3;
        }
    
    //LoadButton press sets the tempstate to 0 (Load: PWM=0)

        if  (digitalRead(loadButton)==HIGH) {
            tempstate = 0; 
        }

    //PidButton press sets the tempstate to 1, which is PID control from setpoint.

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

            case 0://Tempstate is at the load position, no lights and no cooling
                Output = 0;
                analogWrite(IRL540,Output);
                digitalWrite(PIDLED, LOW);
                digitalWrite(freezeLED, LOW);
                digitalWrite(microscopeLED,LOW);
                colorR = 0;colorG=255;colorB=0;
                lcd.setRGB(colorR, colorG, colorB);
            break;

            case 1://Tempsate is at the PID position, meaning the PID algorithm will run and lights will be turned on
                digitalWrite(PIDLED,HIGH);
                digitalWrite(freezeLED,LOW);
                digitalWrite(microscopeLED,HIGH);
                colorR=255-int(Output);colorG=0;colorB=int(Output);//LCD backlight is set according to cooling power. 255=blue, 0=red.
                lcd.setRGB(colorR, colorG, colorB);
            
                //PID Control with quality control, using the TC value from the beginning of the loop.
                    Input = TC;
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

            case 3://Tempstate 3 is the same as load, but with the microscope LED also turned on. The LCD backlight is also set to full white, so this loading mode provides some work light.
                potSignal = analogRead(A2);
                potSignal = map(potSignal,0,925,0,255);//925 was chosed instead of the usual 1023 because the voltage on the 5v rail was 4.5v in this particular case.
                potSignal = constrain(potSignal,0,255);        
                Output = potSignal;
                analogWrite(IRL540,Output);
                digitalWrite(PIDLED,LOW);
                digitalWrite(freezeLED, LOW);
                digitalWrite(microscopeLED,HIGH);

                colorR = 255;colorG=255;colorB=255;
                lcd.setRGB(colorR, colorG, colorB);

            break;

    }


    //Receive setpoint from serial, if available

        if  (Serial.available() > 0 ){
	        incoming = Serial.parseInt();
        }



 

    //Update the LCD if the intervalLCD time has passed
        
        currentTimeLCD = millis();
        if  ( currentTimeLCD - previousTimeLCD > intervalLCD) {
    
            //Cursor is set for first line and TC temperature is printed
                lcd.setCursor(0,0);
                lcd.print("TC:");
                lcd.setCursor(3,0);
                lcd.print(TC);
                lcd.setCursor(8,0);
                lcd.print("State: ");
                lcd.setCursor(15,0);
                lcd.print(tempstate);
         

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
    
            //Print temperature and duty cycle
                Serial.print("Duty cycle: ");
       	        Serial.println(Output);
      	 	    Serial.println(" C");   
                Serial.println(TC);
                Serial.print("100kNTC : ");
                Serial.println(T);
                
            //Reset the Serial timer for next cycle
                previousTimeSerial = currentTimeSerial;  
        }
}

