#include <Arduino.h>

#include <QMC5883L.h>
#include <Wire.h>

QMC5883L compass;

int16_t xCompass, yCompass, zCompass, tCompass;

void setup()
{
	Wire.begin();

	compass.init();
	compass.setSamplingRate(50);

	Serial.begin(9600);
	Serial.println("QMC5883L Compass Demo");
	Serial.println("Turn compass in all directions to calibrate....");
}

void loop()
{
	int heading = compass.readHeading();
    int result = compass.readRaw(&xCompass, &yCompass, &zCompass, &tCompass);
	if(heading==0) {
		/* Still calibrating, so measure but don't print */
	} else {
		//Serial.println(heading);
        Serial.print(heading); Serial.print(", ");
		Serial.print(result); Serial.print(", ");
		Serial.print(xCompass); Serial.print(", ");
		Serial.print(yCompass); Serial.print(", ");
		Serial.print(zCompass); Serial.print(", ");
		Serial.println(tCompass);
        
	}
    delay(100);
}
