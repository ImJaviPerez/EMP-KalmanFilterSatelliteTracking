#include <Arduino.h>

//// #include "P13.h"
#include <Plan13.h>

bool runAgain = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}


void loop() {
  // put your main code here, to run repeatedly:
  double azimut, elevacion ;
  char buf[20] ;
  Satellite sat, moon ;
  DateTime  fecha ;


  //   Definimos un satélite a observar

  sat = Satellite("BEESAT-3                ",
  "1 39135U 13015F   20149.80919899  .00000684  00000-0  42744-4 0  9993",
  "2 39135  64.8659   4.8562 0037220 251.2881 108.4205 15.18143310392626");

  sat = Satellite("SUN                     ",
  "1 00001U 00  0  0 95080.09236111  .00000000  00000-0  00000-0 0  0017",
  "2 00001 023.4400 000.0000 0000000 282.8700 075.2803 00.00273790930019");

  sat = Satellite("SUN95                   ",
  "1 00001U 00000    95080.09236111  .00000000  00000-0  10000-3 0 99999",
  "2 00001 023.4400 000.0000 0000001 282.8700 075.2803 00.00273790000018");

  moon = Satellite("MOON                    ",
  "1 00000U 00000    18159.72676026  .00000000  00000-0  10000-3 0 99993",
  "2 00000 020.3672 011.6574 0628000 102.1918 264.5285 00.03660099000009);



  //   Ubicación del observador (coord geográficas y altitud snm en metros)
  // const Observer &observationPlace = Observer("Derio", 43.3, -2.2, 172.0) ;
  // const Observer &observationPlace = Observer("Leioa", 43.328955, -2.966181, 40.0) ;
  const Observer &observationPlace = Observer("Bilbao", 43.30000, -2.9333, 37.0) ;
  // const Observer &observationPlace = Observer("Copenhague", 55.6167, 12.65, 5.0) ;


  if (runAgain)
  {
    runAgain = false;

    // Serial.print(observationPlace.name);
    Serial.println("DateTime\tElevation\tAzimut") ;
    for (int hh = 7; hh < 8; hh++)
    {
      for (int mm = 0; mm < 21; mm++)
      {   
        for (int ss = 0; ss < 60; ss+=20)
        {
          fecha.settime(2020, 07, 16, hh, mm, ss) ;        // genera momento para la predicción
          sat.predict(fecha) ;                           // genera predicción
          fecha.ascii(buf) ;                             // fecha en ASCII para imprimir más abajo
          sat.altaz(observationPlace, elevacion, azimut) ;          // elevación y azimut desde ubicación proporcionada
          
          //if ((azimut > 0) && (azimut < 90))
          //{
            Serial.print(buf) ;                            // imprime resultados
            // Serial.print("\t") ;
            //Serial.print( sat.name ) ;                      
            Serial.print("\t") ;
            Serial.print(elevacion,4) ;                      
            Serial.print("\t") ;
            Serial.println(azimut,4) ;
          //}
        }
      }
    }
    

      Serial.flush() ;
  }
  
 }