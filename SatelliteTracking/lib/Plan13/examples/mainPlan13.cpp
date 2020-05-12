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
  Satellite satList[10] ;
  DateTime  fecha ;

  //   Definimos un satélite a observar
  sat = Satellite("RADIO ROSTO (RS-15)     ",
	"1 23439U 94085A   19311.42076594 -.00000039  00000-0  83545-4 0  9991",
	"2 23439  64.8160 246.9267 0145361  85.5007 276.2506 11.27568548 23874");


  satList[0] = Satellite("OSCAR 7 (AO-7)          ",
  "1 07530U 74089B   20112.89879479 -.00000047  00000-0 -97882-5 0  9992",
  "2 07530 101.7949  82.9674 0012534 112.0012 304.6453 12.53643112 78903");

  satList[1] = Satellite("YUBILEINY (RS-30)       ",
  "1 32953U 08025A   20112.89743657  .00000037  00000-0  19437-3 0  9999",
  "2 32953  82.5007 313.0876 0019076 174.9536 185.1735 12.43095092540602");

  satList[2] = Satellite("KAITUO 1B               ",
  "1 40912U 15049P   20112.82508424  .00001064  00000-0  52166-4 0  9999",
  "2 40912  97.4505 121.2876 0014850 169.8527 314.6989 15.20221599254110");

  satList[3] = Satellite("TIANWANG 1C (TW-1C)     ",
  "1 40926U 15051B   20112.72295608  .00018557  00000-0  19969-3 0  9991",
  "2 40926  97.0909 164.1878 0004523 309.1202 171.3304 15.63761442258152");

  satList[4] = Satellite("BEESAT-4                ",
  "1 41619U 16040W   20112.82549648  .00001425  00000-0  61869-4 0  9990",
  "2 41619  97.3247 173.6871 0011747  27.8343 332.3520 15.23871937200775");

  satList[5] = Satellite("NAYIF-1 (EO-88)         ",
  "1 42017U 17008BX  20112.84437678  .00001595  00000-0  64152-4 0  9995",
  "2 42017  97.3638 181.2250 0007057 149.3141 210.8512 15.26341856176917");

  satList[6] = Satellite("ES'HAIL 2               ",
  "1 43700U 18090A   20112.77947424  .00000142  00000-0  00000-0 0  9999",
  "2 43700   0.0124 256.7313 0002562 168.3381  91.6664  1.00274241  5148");

  satList[7] = Satellite("PSAT2 (NO-104)          ",
  "1 44354U 19036R   20112.88696932  .00017117  00000-0  25410-3 0  9998",
  "2 44354  28.5305  31.8899 0368042  67.2522 296.6529 15.02593063 45305");

  satList[8] = Satellite("BRICSAT2 (NO-103)       ",
  "1 44355U 19036S   20112.88555242  .00032222  00000-0  46116-3 0  9997",
  "2 44355  28.5305  26.0313 0349900  76.7648 287.1725 15.07174000 45369");

  satList[9] = Satellite("CAS-2T & KS-1Q          ",
  "1 41847U 16066G   20112.84939343  .00000057  00000-0  18640-4 0  9997",
  "2 41847  98.6606 162.7710 0366397  37.7521 324.8775 14.38057603181287");


  //   Ubicación del observador (coord geográficas y altitud snm en metros)
  // const Observer &observationPlace = Observer("Derio", 43.3, -2.2, 172.0) ;
  // const Observer &observationPlace = Observer("Leioa", 43.328955, -2.966181, 40.0) ;
  const Observer &observationPlace = Observer("Bilbao", 43.30000, -2.9333, 37.0) ;
  // const Observer &observationPlace = Observer("Copenhague", 55.6167, 12.65, 5.0) ;


  if (runAgain)
  {
    runAgain = false;

    Serial.print(observationPlace.name);
    Serial.println("\tDateTime\tSatName\tAzimut\tElevacion") ;
    for (size_t i = 0; i < 1; i++)
    {
      sat = satList[i];
    
      for (int hh = 12; hh < 24; hh++)
      {
        for (int mm = 0; mm < 60; mm+=5) {        
          fecha.settime(2020, 04, 22, hh, mm, 0) ;        // genera momento para la predicción
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