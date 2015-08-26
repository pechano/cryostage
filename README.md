# cryostage

Contained within the /Cryostage repository is all the resources needed to use the Arduino controlled cryostage by Johannes Lørup Buch.

This includes:
	The Arduino sketch
	EAGLE files for schematic and PCB (ressources)
	Schematics of any lasercut objects used in the project
	The scientific poster used to present the project

The project was a part of my PhD thesis and anything contained here has been used purely for academic purposes. 

###########
Arduino libraries used:
###########

	MAX31855.h //Library by Rob Tillaart. Used for communicating with the MAX31855 chip. https://github.com/RobTillaart/Arduino/tree/master/libraries/MAX31855
  	PID_v1.h //The Arduino PID algorithm by Brett Beauregaard, used to control temperature in setpoint mode. https://github.com/br3ttb/Arduino-PID-Library
  	rgb_lcd.h //Grove LCD RGB Backlight library. https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight    
	math.h //Math functions used in thermistor calculations.
    	Wire.h //I2C library used for communicating with the LCD.

