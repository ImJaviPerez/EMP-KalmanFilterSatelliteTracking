#include <Arduino.h>

//// #include "P13.h"
#include <Plan13.h>

bool runOnce = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}


void loop() {
  // put your main code here, to run repeatedly:
  double azimut, elevacion ;
  char buf[20] ;
  Satellite sat ;
  DateTime  fecha ;

  //   Definimos un satélite a observar

  /*
  sat = Satellite("UK-DMC 2                ",
	"1 35683U 09041C   12289.23158813  .00000484  00000-0  89219-4 0  5863",
	"2 35683  98.0221 185.3682 0001499 100.5295 259.6088 14.69819587172294") ;
  */
 /*
  sat = Satellite("MOLNIYA 1-36                ",
	"1 09880U 77021A   19310.41076981  .00001413  00000-0  22811-3 0  9998",
	"2 09880  62.2133  59.2477 7471150 281.4413  10.3732  2.00844742210264") ;
*/
/*
  sat = Satellite("FUNCUBE-1 (AO-73)           ",
	"1 39444U 13066AE  19311.49643895  .00000247  00000-0  36839-4 0  9999",
	"2 39444  97.5590 319.1036 0056564 282.6309  76.8584 14.82004767320934") ;
*/
  sat = Satellite("RADIO ROSTO (RS-15)     ",
	"1 23439U 94085A   19311.42076594 -.00000039  00000-0  83545-4 0  9991",
	"2 23439  64.8160 246.9267 0145361  85.5007 276.2506 11.27568548 23874") ;



  //   Ubicación del observador (coord geográficas y altitud snm en metros)

  // const Observer &observationPlace = Observer("Derio", 43.3, -2.2, 172.0) ;
  const Observer &observationPlace = Observer("Leioa", 43.328955, -2.966181, 40.0) ;
  // const Observer &observationPlace = Observer("Copenhague", 55.6167, 12.65, 5.0) ;


  if (runOnce)
  {
    Serial.print(observationPlace.name);
    Serial.println("\t\tElevacion\tAzimut") ;
  
    runOnce = false;
    for (int i = 0; i < 24; i++) {
        // fecha.settime(2012, 10, 15, 5, 33, i) ;        // genera momento para la predicción
        
        fecha.settime(2019, 11, 8, i, 0, 0) ;        // genera momento para la predicción
        sat.predict(fecha) ;                           // genera predicción
        fecha.ascii(buf) ;                             // fecha en ASCII para imprimir más abajo
        sat.altaz(observationPlace, elevacion, azimut) ;          // elevación y azimut desde ubicación proporcionada
        
        Serial.print(i) ;                            // imprime resultados
        Serial.print("  ") ;
        Serial.print(buf) ;                            // imprime resultados
        Serial.print("\t") ;
        Serial.print(elevacion) ;                      
        Serial.print("\t\t") ;
        Serial.println(azimut) ;
      }
      Serial.flush() ;
  }
  
 }