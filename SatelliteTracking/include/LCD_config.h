#ifndef LCD_CONFIG_h
#define LCD_CONFIG_h

/*
#define BarGauge_PAGE				1
#define CircleGauge_PAGE			2
#define Keyboard_PAGE				3
//#define Main_PAGE					4
#define Setting_PAGE				5
#define Slider_PAGE					6
#define Temperture_PAGE				7
#define Text_PAGE					8
#define Waveform_PAGE				9
*/
#define MAIN_PAGE               2
// #define MAIN_BUTTON_ATTITUDE    2
// #define MAIN_BUTTON_SATELLITES  3
// #define MAIN_LABEL_TITLE_SAT_NAME 4
#define MAIN_LABEL_SAT_NAME     5
#define MAIN_NUMBER_HH           8
#define MAIN_NUMBER_MM           9
#define MAIN_NUMBER_SS           10

#define ATTITUDE_PAGE                   1
// #define ATT_BUTTON_HOME                 20
#define ATT_COMPASS                     21
#define ATT_ROLL_DEVICE                 22
#define ATT_ROLL_SATELLITE              23
#define ATT_LABEL_TEST_01               24
#define ATT_LABEL_TEST_02               25
#define ATT_LABEL_TEST_03               26
#define ATT_LABEL_TEST_04               27
#define ATT_HEADING                     28
#define ATT_NUMBER_AZIMUT_SATELLITE     44
#define ATT_NUMBER_AZIMUT_ANTENNA       45
#define ATT_NUMBER_ASCENSION_SATELLITE  46
#define ATT_NUMBER_ASCENSION_ANTENNA    47
#define ATT_LABEL_SAT_NAME              41
#define ATT_NUMBER_HH                   48
#define ATT_NUMBER_MM                   49
#define ATT_NUMBER_SS                   50
#define ATT_RECTANGLE_GPS_ON            51

#define SATELLITES_PAGE     3
#define SAT_BUTTON_HOME     51
#define SAT_BUTTON_OK       52
#define SAT_BUTTON_UP       53
#define SAT_BUTTON_DOWN     54
#define SAT_LABEL_PAGE_NUMB 55
#define SAT_CHECK_SAT_01    61
#define SAT_CHECK_SAT_02    62
#define SAT_CHECK_SAT_03    63

#endif // LCD_CONFIG_h