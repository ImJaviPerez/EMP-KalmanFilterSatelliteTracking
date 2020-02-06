#include <Arduino.h>

#include <QMC5883L_ST.h>
#include <Wire.h>

QMC5883L_ST compass(180);

void setup()
{
	// Initiate the Wire library
	Wire.begin();

	compass.init();
	compass.setSamplingRate(50);

	Serial.begin(115200);
	Serial.println("QMC5883L_ST Compass Demo");
	Serial.println("Turn compass in all directions to calibrate....");

	// PORT: Time Period After VDD and VDDIO at Operating Voltage to Ready for I2C Commend and Analogy Measurement = 350 us
	// Power on Interval. PINT. Time Period Required for Voltage Lower Than SDV to Enable Next POR = 100 us
	delay(500);



	int heading = 0;
	delay(10);
	while(heading==0) {
		// Still calibrating, so measure but don't print
		Serial.print("+");
		delay(10);
		heading = compass.readHeading();
	}

}

void loop()
{
	int heading = compass.readHeading();

	Serial.println(heading);
/*	
	if(heading==0) {
		// Still calibrating, so measure but don't print
    Serial.print("+");
    delay(10);
	} else {
				
		Serial.println(heading);
	}
*/	
	
}
