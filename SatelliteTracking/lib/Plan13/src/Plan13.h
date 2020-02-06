//
// Plan13.cpp
//
// An implementation of Plan13 in C++ by Mark VandeWettering
//
// Plan13 is an algorithm for satellite orbit prediction first formulated
// by James Miller G3RUH.  I learned about it when I saw it was the basis
// of the PIC based antenna rotator project designed by G6LVB.
//
// http://www.g6lvb.com/Articles/LVBTracker2/index.htm
//
// I ported the algorithm to Python, and it was my primary means of orbit
// prediction for a couple of years while I operated the "Easy Sats" with
// a dual band hand held and an Arrow antenna.
//
// I've long wanted to redo the work in C++ so that I could port the code
// to smaller processors including the Atmel AVR chips.  Bruce Robertson,
// VE9QRP started the qrpTracker project to fufill many of the same goals,
// but I thought that the code could be made more compact and more modular,
// and could serve not just the embedded targets but could be of more
// use for more general applications.  And, I like the BSD License a bit
// better too.
//
// So, here it is!
//

//----------------------------------------------------------------------

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


double RADIANS(double deg) ;

double DEGREES(double rad) ;


// here are a bunch of constants that will be used throughout the
// code, but which will probably not be helpful outside.

static const double RE = 6378.137f ;
static const double FL = 1.f/298.257224f ;
static const double GM = 3.986E5f ;
static const double J2 = 1.08263E-3f ;
static const double YM = 365.25f ;
static const double YT = 365.2421874f ;
static const double WW = 2.f*M_PI/YT ;
static const double WE = 2.f*M_PI+ WW ;
static const double W0 = WE/86400.f ;

// BEGIN CHANGE ImJaviPerez ----------------------------
// Thanks to Thorsten Godau: https://github.com/dl9sec
// Discussion at: https://github.com/brainwagon/angst/issues/4
//
// Original source code: https://www.amsat.org/articles/g3ruh/111.html
// *** Astronomical data for 2014 onwards
// 
// 1620 REM Sidereal and Solar data. Valid to year ~2030
// 1630 YG = 2014: G0 = 99.5828:  REM GHAA, Year YG, Jan 0.0
// 1640 MAS0 = 356.4105: MASD = 0.98560028: REM MA Sun and rate, deg, deg/day
// 1650 INS = RAD(23.4375): CNS = COS(INS): SNS = SIN(INS): REM Sun's inclination
// 1660 EQC1=0.03340: EQC2=0.00035:   REM Sun's Equation of centre terms
// 
/* 
Old source code:
REM Sidereal and Solar data. Rarely needs changing. Valid to year ~2015
static const double YG = 2000.f ;
static const double G0 = 98.9821f ;
static const double MAS0 = 356.0507f ;
static const double MASD = 0.98560028f ;
static const double EQC1 = 0.03342 ;
static const double EQC2 = 0.00035 ;
static const double INS = RADIANS(23.4393f) ;
*/
static const double YG = 2014.f;
static const double G0 = 99.5828f;
static const double MAS0 = 356.4105f;
static const double MASD = 0.98560028f;
static const double EQC1 = 0.03340;
static const double EQC2 = 0.00035;
static const double INS = RADIANS(23.4375f);
//
// END CHANGE ImJaviPerez ----------------------------
static const double CNS = cos(INS) ;
static const double SNS = sin(INS) ;

//----------------------------------------------------------------------

// the original BASIC code used three variables (e.g. Ox, Oy, Oz) to
// represent a vector quantity.  I think that makes for slightly more
// obtuse code, so I going to collapse them into a single variable
// which is an array of three elements

typedef double Vec3[3] ;

//----------------------------------------------------------------------

class DateTime {
public:
	long DN ;
	double TN ;
	DateTime(int year, int month, int day, int h, int m, int s) ;
	DateTime(const DateTime &) ;
	DateTime() ;
	~DateTime() { }
	void add(double) ;
	void settime(int year, int month, int day, int h, int m, int s) ;
	void gettime(int& year, int& mon, int& day, int& h, int& m, int& s) ;
	void ascii(char *) ;
	void roundup(double) ;
} ;

//----------------------------------------------------------------------

class Observer {
public:
    const char *name ;
    double LA ;
    double LO ;
    double HT ;
    Vec3 U, E, N, O, V  ;

    Observer(const char *, double, double, double) ;
    ~Observer() { } ;
} ;

//----------------------------------------------------------------------

// class Satellite with explanations
// SATELLITE EPHEMERIS
class Satellite {
	long N ;
	long YE ;	// Epoch Year    year
	long DE ;
	double TE ;	// Epoch time    days
	double IN ;	//Inclination   deg
	double RA ;	// REM R.A.A.N.      deg
	double EC ;	// Eccentricity
	double WP ;	// Arg perigee   deg
	double MA ;	// Mean anomaly  deg
	double MM ;	// Mean motion   rev/d
	double M2 ;	// Decay Rate    rev/d/d
	double RV ;	// Orbit number
	double ALON ;	// Sat attitude, deg. 180 = nominal ) See bulletins
	double ALAT ;	// Sat attitude, deg.   0 = nominal ) for latest


	// these values are stored, but could be calculated on the fly
	// during calls to predict()
	// classic space/time tradeoff

	// N0 = MM/86400:      REM Mean motion rad/s
 	// A0 = (GM/N0/N0)^(1/3):  REM Semi major axis km
 	// B0 = A0*SQR(1-EC*EC):   REM Semi minor axis km
	double N0, A_0, B_0 ;
	double PC ;	// Precession const, rad/Day
	// QD = -PC*CI:            REM Node precession rate, rad/day
	// WD =  PC*(5*CI*CI-1)/2: REM Perigee precession rate, rad/day
	// DC = -2*M2/MM/3: REM Drag coeff. (Angular momentum rate)/(Ang mom)  s^-1
	double QD, WD, DC ;
	// RS = A*DNOM: REM Distances
	double RS ;

public:
	const char *name ;
	Vec3 SAT, VEL ;		// celestial coordinates
	Vec3 S, V ;		// geocentric coordinates

	Satellite() { } ;
	Satellite(const char *name, const char *l1, const char *l2) ;
	~Satellite() ;
	void tle(const char *name, const char *l1, const char *l2) ;
	void predict(const DateTime &dt) ;
	void LL(double &lat, double &lng) ;
	void altaz(const Observer &obs, double &alt, double &az) ;
} ;

class Sun {
public:
	Vec3 SUN, H ;
	Sun() ;
	~Sun() { } ;
	void predict(const DateTime &dt) ;
	void LL(double &lat, double &lng) ;
	void altaz(const Observer &obs, double &alt, double &az) ;
} ;
