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
  Satellite sat ;
  DateTime  fecha ;

  //   Definimos un satélite a observar

  sat = Satellite("BEESAT-3                ",
  "1 39135U 13015F   20149.80919899  .00000684  00000-0  42744-4 0  9993",
  "2 39135  64.8659   4.8562 0037220 251.2881 108.4205 15.18143310392626");



  //   Ubicación del observador (coord geográficas y altitud snm en metros)
  // const Observer &observationPlace = Observer("Derio", 43.3, -2.2, 172.0) ;
  // const Observer &observationPlace = Observer("Leioa", 43.328955, -2.966181, 40.0) ;
  const Observer &observationPlace = Observer("Bilbao", 43.30000, -2.9333, 37.0) ;
  // const Observer &observationPlace = Observer("Copenhague", 55.6167, 12.65, 5.0) ;


  if (runAgain)
  {
    runAgain = false;

    // Serial.print(observationPlace.name);
    Serial.println("\tDateTime\tSatName\tAzimut\tElevation") ;
    for (int hh = 9; hh < 10; hh++)
    {
      for (int mm = 0; mm < 9; mm++)
      {   
        for (int ss = 0; ss < 60; ss++)
        {
          fecha.settime(2020, 06, 01, hh, mm, ss) ;        // genera momento para la predicción
          sat.predict(fecha) ;                           // genera predicción
          fecha.ascii(buf) ;                             // fecha en ASCII para imprimir más abajo
          sat.altaz(observationPlace, elevacion, azimut) ;          // elevación y azimut desde ubicación proporcionada
          
          if ((azimut > 0) && (azimut < 90))
          {
            Serial.print(buf) ;                            // imprime resultados
            Serial.print("\t") ;
            Serial.print( sat.name ) ;                      
            Serial.print("\t") ;
            Serial.print(azimut) ;
            Serial.print("\t") ;
            Serial.println(elevacion) ;                      
          }
        }
      }
    }
    

      Serial.flush() ;
  }
  
 }