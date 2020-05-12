/**
 * @file SatTrack.cpp 
 * @author ImJaviPerez
 * @brief 
 * @version 0.1.004
 * @date 2019-12-20 18:21
 * 
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// RobotDyn MEGA-WiFi R3 configuration:
// 
// Board Generic MEGA 2560
// https://docs.platformio.org/en/latest/boards/espressif8266/esp12e.html
//
// While uploading
// DIM switches:
// 1  ON
// 2  ON
// 3  ON
// 4  ON
// 5  OFF
// 6  OFF
// 7  OFF
// 8  OFF
//
//
// When uploading is finished
// DIM switches:
// 1  ON
// 2  ON
// 3  ON
// 4  ON
// 5  OFF
// 6  OFF
// 7  OFF
// 8  OFF
//
#include <STSerialMsg.h>
#include <STDebug.h>
#include <TinyGPS.h>  // GPS NEO V2
#include <QMC5883L_ST.h> // Digital compass
#include <Wire.h>     // Digital compass I2C communication
#include <I2Cdev.h>   // MPU6050 accelerometer Gyroscope communication
#include <Kalman_AGC.h> // Accelerometer, Gyroscope, Compass
#include <LiquidCrystalDisplay.h> // H35B-IC touch screen display

#include "SatTrack.h" // SatTrack.cpp header files
#include "STConfig.h" // App configuration
#include "LCD_config.h" // H35B-IC touch screen display

// =====================================================
// Configuring user messages in an appropriate language
#ifdef LANG_ENG
    #include "STUserMessagesENG.h"
#else
  #ifdef LANG_ESP
    #include "STUserMessagesESP.h"
  #endif
#endif
// =======================================================

#include <STDebug.h>

// Plan13 calculates satellite attitude
#include <Plan13.h>

#include <Arduino.h>


using namespace SatTrack;

#define MEGA2560_TRACE (MEGA2560_TRACE_TEST | MEGA2560_TRACE_INFO | MEGA2560_TRACE_WARNING | MEGA2560_TRACE_ERROR)



const double kalAngleY_array[] = {0.03374983, 0.05396196, 0.07551581, 0.07207574, 0.07289729, 0.05148751, 0.04134257, 0.04614449, 0.04562800, 0.02332562, 0.02203336, 0.02549419, 0.02567886, 0.03099540, 0.02360774, 0.02344486, 0.01744263, 0.01977973, 0.02490207, 0.02112831, 0.03495816, 0.03961930, 0.02872736, 0.02834678, 0.02846124, 0.02913422, 0.03421975, 0.03080722, 0.03754004, 0.04055694, 0.03785620, 0.03653201, 0.03633381, 0.03312325, 0.02074987, 0.04813487, 0.05449000, 0.07234949, 0.08855302, 0.10778797, 0.10223716, 0.10764644, 0.10543644, 0.11343760, 0.11549623, 0.10900562, 0.08553758, 0.08331526, 0.09517132, 0.10442203, 0.10944059, 0.11065963, 0.10958513, 0.09579839, 0.08899766, 0.09517085, 0.08769997, 0.08972409, 0.10417684, 0.12537678, 0.11947022, 0.13475619, 0.14457071, 0.16963577, 0.16700754, 0.17462783, 0.18002670, 0.16515172, 0.16194537, 0.16762248, 0.16231856, 0.15186486, 0.15710828, 0.12510461, 0.12438343, 0.10888059, 0.07736905, 0.08007952, 0.03076786, 0.02908052, 0.03297940, 0.05246203, 0.07099172, 0.07277409, 0.07358602, 0.08617160, 0.09609146, 0.11935416, 0.13217618, 0.16741320, 0.16620283, 0.19748414, 0.20133018, 0.25442487, 0.27915096, 0.28672400, 0.28126737, 0.27722204, 0.25991219, 0.25987026, 0.27454752, 0.27674606, 0.26324466, 0.28231317, 0.25518185, 0.25335455, 0.24898700, 0.23245558, 0.22748724, 0.23217292, 0.23788591, 0.23494996, 0.22713675, 0.23506542, 0.18939722, 0.19644114, 0.19934756, 0.20862432, 0.20867383, 0.20332268, 0.21445075, 0.20036967, 0.21887794, 0.22102921, 0.19355136, 0.21601747, 0.20238873, 0.21953724, 0.20101017, 0.22529259, 0.21370250, 0.23027612, 0.21095710, 0.16007012, 0.20943648, 0.19457918, 0.21685736, 0.19180886, 0.21680337, 0.21463186, 0.21398306, 0.23524323, 0.22035682, 0.22750297, 0.20815161, 0.23403464, 0.21453512, 0.24379468, 0.21392106, 0.22603095, 0.23634842, 0.20997185, 0.23959270, 0.21178895, 0.21424559, 0.21066426, 0.22949934, 0.22087909, 0.24413206, 0.21894021, 0.22019373, 0.20891854, 0.21017054, 0.18101561, 0.23252413, 0.21142992, 0.19517539, 0.20369548, 0.23892692, 0.21193962, 0.24109821, 0.17043005, 0.24370876, 0.22310975, 0.18298714, 0.24143909, 0.17577809, 0.23678324, 0.24834204, 0.29527462, 0.17311102, 0.21777073, 0.31078824, 0.17127641, 0.19539186, 0.20587149, 0.22221354, 0.23077081, 0.20055874, 0.23483354, 0.19847630, 0.23502061, 0.23858333, 0.25363421, 0.25842103, 0.23867033, 0.23538212, 0.29064599, 0.24302207, 0.27656281, 0.25629649, 0.33452103, 0.30681494, 0.31085324, 0.36331069, 0.37271845, 0.33300567, 0.40083691, 0.35847175, 0.38516900, 0.42505550, 0.45841160, 0.47903624, 0.42145878, 0.42867321, 0.45045829, 0.39531472, 0.39760312, 0.43786991, 0.35403463, 0.38144213, 0.40246570, 0.35470343, 0.37810570, 0.43093452, 0.38275138, 0.44713688, 0.43938547, 0.42186323, 0.45350745, 0.45610291, 0.45927310, 0.43917209, 0.47383934, 0.42231128, 0.46421918, 0.41628790, 0.45497066, 0.44175649, 0.42559922, 0.45472264, 0.41723704, 0.45605522, 0.40587595, 0.43138471, 0.45561928, 0.40577292, 0.45650548, 0.47894639, 0.45534748, 0.50269216, 0.52474332, 0.49263382, 0.51440078, 0.53861344, 0.56937766, 0.46773297, 0.69533288, 0.53030467, 0.49052736, 0.38927159, 0.38675499, 0.42019331, 0.33521187, 0.35820678, 0.39362693, 0.38949811, 0.40121591, 0.37800798, 0.36350980, 0.39162123, 0.36313933, 0.31318754, 0.35282931, 0.31375489, 0.34686998, 0.32594150, 0.30826288, 0.33192503, 0.31499296, 0.30483493, 0.31250402, 0.30384105, 0.31994024, 0.30966756, 0.32549348, 0.27463537, 0.31168514, 0.31866789, 0.31706101, 0.29488149, 0.35585153, 0.28948164, 0.32958978, 0.32045132, 0.28427821, 0.28078768, 0.33348495, 0.24286166, 0.29454160, 0.27450386, 0.28472596, 0.28437266, 0.27912313, 0.28716290, 0.34638512, 0.31718355, 0.34076160, 0.31583264, 0.32359803, 0.28195840, 0.35685831, 0.29898191, 0.29180712, 0.29227209, 0.29097313, 0.27721399, 0.27554652, 0.28131062, 0.26880777, 0.28495654, 0.25916028, 0.27675316, 0.28810954, 0.26967093, 0.26343909, 0.29788858, 0.27528515, 0.26950586, 0.28173774, 0.28470230, 0.31769919, 0.27196717, 0.29762873, 0.28493059, 0.30702609, 0.30328923, 0.30121151, 0.31384936, 0.29628545, 0.28888187, 0.30898359, 0.28554150, 0.32211566, 0.29094678, 0.30642134, 0.29492807, 0.31682113, 0.30096820, 0.29714987, 0.29100123, 0.31416947, 0.29866737, 0.29926118, 0.31311202, 0.27190304, 0.31354177, 0.29098791, 0.31130648, 0.28876224, 0.31731513, 0.27712858, 0.30199701, 0.31330141, 0.27645123, 0.33120334, 0.27964967, 0.31740367, 0.29178336, 0.31430435, 0.30261382, 0.27907082, 0.31191880, 0.29644057, 0.30631381, 0.31526840, 0.29512897, 0.30472103, 0.29547530, 0.29936373, 0.30410832, 0.30084684, 0.28892878, 0.31211525, 0.29850796, 0.29301068, 0.30563712, 0.30114424, 0.30483848, 0.29797202, 0.30057320, 0.30444548, 0.30029234, 0.29145059, 0.31750661, 0.29578373, 0.30487987, 0.30133522, 0.30372220, 0.31106234, 0.28651303, 0.29829681, 0.33404687, 0.32436854, 0.30567303, 0.30912280, 0.30943131, 0.32641959, 0.30704597, 0.31909296, 0.30829257, 0.32596257, 0.30011451, 0.32298297, 0.30423301, 0.32342976, 0.31461760, 0.31431827, 0.30440074, 0.28489858, 0.32846308, 0.30050510, 0.32726175, 0.29082558, 0.32894212, 0.29755175, 0.30192444, 0.33083063, 0.30244014, 0.33225858, 0.29418650, 0.31243527, 0.30897802, 0.30893421, 0.30352366, 0.32649359, 0.30470496, 0.30754018, 0.30874980, 0.30174205, 0.30778843, 0.31136826, 0.30244955, 0.30726945, 0.32158723, 0.29832581, 0.30778557, 0.31073368, 0.31554025, 0.31351030, 0.30018020, 0.30924082, 0.30190128, 0.30419135, 0.30669966, 0.30269638, 0.32478628, 0.30881229, 0.31911951, 0.30844751, 0.30840108, 0.31496030, 0.30243441, 0.31289989, 0.31402481, 0.30291846, 0.30073512, 0.30457413, 0.31003949, 0.30897984, 0.30211723, 0.30697554, 0.31209540, 0.31333548, 0.31249946, 0.31400323, 0.31374952, 0.31664664, 0.31754750, 0.31519997, 0.32065925, 0.31073454, 0.32343936, 0.31621844, 0.33464721, 0.30389249, 0.32149222, 0.30858809, 0.30599868, 0.32907486, 0.32127672, 0.31207412, 0.31199133, 0.31410685, 0.30386254, 0.33397472, 0.30737078, 0.31886214, 0.30987555, 0.32091525, 0.31704837, 0.31752867, 0.32532153, 0.31965691, 0.31371957, 0.31745550, 0.34375092, 0.31235287, 0.32235098, 0.31070197, 0.34408551, 0.30465063, 0.32514140, 0.31590807, 0.32089156, 0.32093695, 0.32283637, 0.31747720, 0.32031053, 0.32801065, 0.30972892, 0.32771674, 0.31060204, 0.31188899, 0.31080985, 0.31758019, 0.31145155, 0.31999215, 0.31647089, 0.31112516, 0.32217452, 0.32512680, 0.32489279, 0.31548071, 0.32556647, 0.30962130, 0.32836363, 0.32842049, 0.31309903, 0.32160142, 0.32661378, 0.30487373, 0.32155579, 0.31326771, 0.32744822, 0.33004826, 0.32872018, 0.31906253, 0.32647905};
const float horizontalHeadingAngle_array[] = {6.13261750, 6.17740060, 6.18012910, 6.17283870, 6.17193370, 6.16894910, 6.16695170, 6.17624280, 6.16405150, 6.15879440, 6.16018150, 6.15115790, 6.14969920, 6.15564300, 6.16752580, 6.17342660, 6.17571070, 6.17713400, 6.17695090, 6.15206000, 6.15593390, 6.17820550, 6.16617820, 6.17185690, 6.17119170, 6.14742090, 6.13165330, 6.18078420, 6.14401960, 6.13672830, 6.14753630, 6.12949370, 6.15223460, 6.16573720, 6.22789670, 6.15841440, 6.21069720, 6.18056920, 6.13667580, 6.16271070, 6.10932350, 6.10995530, 6.15362070, 6.18326140, 6.16799020, 6.14890580, 6.16216900, 6.11784360, 6.13985250, 6.15629290, 6.13453010, 6.15200900, 6.10635520, 6.12117430, 6.26465700, 6.19324970, 6.17789170, 6.18661640, 6.20799110, 0.03486336, 0.02302901, 0.03109941, 0.04758435, 0.03608374, 0.04774952, 0.03109228, 0.00291161, 0.05280032, 0.00417127, 0.00958779, 0.08754368, 0.02810852, 0.03867527, 0.06724005, 0.04556736, 0.05564900, 0.03947682, 0.10563622, 0.10002898, 0.20130645, 0.16541223, 0.16413662, 0.12969428, 0.08609713, 0.01876073, 6.27786110, 0.01405992, 6.23203850, 6.25256110, 6.22291090, 6.18068170, 6.15937570, 6.11649270, 6.08787680, 6.13271570, 6.03077790, 6.05193380, 6.03366140, 6.01093240, 5.97480680, 5.93064310, 5.90649370, 5.87626890, 5.85238410, 5.87127690, 5.80791040, 5.88715650, 5.84359460, 5.75959540, 5.71859500, 5.68311690, 5.73238330, 5.68332960, 5.66300680, 5.60670420, 5.55853460, 5.50406650, 5.48056890, 5.44281200, 5.39522550, 5.43962810, 5.39467380, 5.30447390, 5.32012610, 5.29234700, 5.23146580, 5.22689340, 5.15812640, 5.10237550, 5.04107760, 5.09857230, 5.04772190, 5.00756840, 4.98567770, 4.93004890, 4.94187640, 4.88068390, 4.81305460, 4.78337910, 4.79042290, 4.75587270, 4.73765560, 4.68183570, 4.68318130, 4.65892890, 4.59754850, 4.53207970, 4.53525920, 4.51615720, 4.52367120, 4.48080160, 4.51194240, 4.42874340, 4.48729040, 4.41433380, 4.39887330, 4.38489820, 4.34370900, 4.36562160, 4.30393360, 4.25567010, 4.29398920, 4.22388940, 4.19098760, 4.13191180, 4.21884200, 4.13428310, 3.95420810, 3.93376710, 3.82303950, 3.65617350, 3.52898860, 3.54890250, 3.71597340, 3.42240260, 3.09874220, 3.25660350, 3.14446740, 3.36803720, 3.56267620, 3.26467900, 3.29313850, 3.48183060, 3.49127480, 3.43890930, 3.50405960, 3.54311420, 3.60121800, 3.70531820, 3.70522620, 3.83172680, 3.74670270, 3.81395360, 3.80484100, 4.02924060, 4.04298970, 4.07877400, 4.16323800, 4.23269460, 4.10869220, 4.28277730, 4.12931590, 3.99829440, 3.95125680, 4.00560620, 4.10285140, 4.04996440, 4.01848030, 4.12385180, 3.96312500, 3.96936610, 3.94310240, 3.93357660, 3.90151240, 3.73702500, 3.70481920, 3.71139600, 3.62832900, 3.72784710, 3.58232020, 3.50868230, 3.57719180, 3.61604190, 3.52010490, 3.56543680, 3.53892210, 3.48550940, 3.59625960, 3.58622550, 3.68259930, 3.62493060, 3.67654800, 3.68952610, 3.64961580, 3.73940830, 3.74585200, 3.87933090, 3.79797460, 3.89155340, 3.81846520, 3.91116810, 3.96717550, 3.90462260, 3.88301420, 3.90955380, 3.99265840, 4.00024130, 3.97993490, 4.02826930, 3.96648140, 3.97659560, 4.05913160, 4.00105380, 3.96824260, 3.95895050, 4.03033640, 4.03875260, 4.26534270, 4.31766510, 3.93233440, 3.95119640, 3.91517830, 3.90078830, 3.91608500, 3.82409570, 3.86010670, 3.87144140, 3.88272640, 3.85326150, 3.85006930, 3.85487510, 3.82869200, 3.82698920, 3.79719590, 3.86723070, 3.69987200, 3.82658150, 3.70337890, 3.66696210, 3.70237450, 3.67550300, 3.61717890, 3.70090580, 3.66397210, 3.67197470, 3.65394590, 3.65219830, 3.66574840, 3.65273740, 3.61121030, 3.59206820, 3.58644530, 3.57724480, 3.57123140, 3.62429210, 3.58601980, 3.63411810, 3.59728740, 3.66913290, 3.61679940, 3.65887070, 3.58635280, 3.57423660, 3.59370660, 3.59239320, 3.60965590, 3.57485150, 3.56668830, 3.66367480, 3.64031000, 3.60369780, 3.57333400, 3.67377310, 3.69167660, 3.61827180, 3.58883880, 3.63991260, 3.65065930, 3.62863780, 3.60570550, 3.64853570, 3.59801720, 3.68248650, 3.66431860, 3.65527560, 3.69071510, 3.69228910, 3.71919680, 3.76826760, 3.78090290, 3.79029940, 3.89070750, 3.82620430, 3.84391880, 3.80979850, 3.90885500, 3.91532110, 3.89725850, 3.77487800, 3.80823420, 3.88780930, 3.83075190, 3.92438550, 3.79470250, 3.88085030, 3.83101490, 3.94371180, 3.80227760, 3.84828470, 3.79883810, 3.77024790, 3.73339610, 3.74255420, 3.67029120, 3.71365710, 3.63762520, 3.58091830, 3.64197250, 3.62727190, 3.58156300, 3.57240460, 3.55929140, 3.49952700, 3.52832250, 3.49184420, 3.43680410, 3.47606850, 3.53344010, 3.53285620, 3.45594290, 3.45747470, 3.50480180, 3.45550200, 3.53071810, 3.48807500, 3.50916430, 3.58784870, 3.48363230, 3.49758670, 3.51656910, 3.48991610, 3.49645280, 3.51909020, 3.57513140, 3.50844430, 3.59388540, 3.53710700, 3.55034920, 3.56104900, 3.56276200, 3.57897260, 3.58983350, 3.58966300, 3.55737690, 3.57799720, 3.56138180, 3.53208300, 3.57198550, 3.58209180, 3.63091250, 3.63520070, 3.59849410, 3.63220790, 3.63540080, 3.59681820, 3.65813830, 3.70288010, 3.64573240, 3.67950300, 3.68869920, 3.66748450, 3.69685440, 3.74239160, 3.71461940, 3.77727080, 3.69242840, 3.72856810, 3.71804550, 3.79489990, 3.76102710, 3.73324750, 3.72224660, 3.75649190, 3.77869530, 3.71166630, 3.73553490, 3.77407190, 3.75665880, 3.73866530, 3.83323220, 3.70982930, 3.64857910, 3.69025040, 3.62554650, 3.64655160, 3.61280850, 3.60637020, 3.61549140, 3.58623340, 3.60448380, 3.58196450, 3.54929640, 3.54960510, 3.59265040, 3.54913380, 3.51533060, 3.56865000, 3.46958640, 3.47920560, 3.47752400, 3.47391010, 3.45026490, 3.54603510, 3.52796720, 3.51762010, 3.53296660, 3.52036600, 3.52643730, 3.57115910, 3.53234910, 3.48690270, 3.56026630, 3.58942410, 3.56509180, 3.61729570, 3.56768920, 3.61816170, 3.61672570, 3.58711960, 3.59136180, 3.63705830, 3.64198540, 3.61993960, 3.66713760, 3.62890600, 3.65742750, 3.65094800, 3.74009250, 3.73593470, 3.69359280, 3.70067880, 3.75128600, 3.68691850, 3.72459130, 3.70046330, 3.74086760, 3.63519930, 3.68499920, 3.66149430, 3.58446550, 3.61181240, 3.60159990, 3.59092880, 3.59167340, 3.58134370, 3.59416370, 3.54415990, 3.55740930, 3.54207560, 3.48990540, 3.50261040, 3.49483040, 3.50619600, 3.52280210, 3.50740930, 3.55533360, 3.54138520, 3.54514170, 3.46982770, 3.53231790, 3.50672010, 3.47941830, 3.47793390, 3.52653030, 3.45352840, 3.55044840, 3.52249430, 3.55848290, 3.53274750, 3.55585150, 3.58032460, 3.55870270, 3.54228880, 3.49454550, 3.57375760, 3.51083330, 3.54319620, 3.55153890, 3.62143540, 3.54710960, 3.54195900, 3.54210590, 3.56950330, 3.60326650, 3.57282920, 3.56130000, 3.61549740, 3.57840920, 3.61111550, 3.60449810, 3.58727930, 3.61801480, 3.57865410, 3.58896060, 3.63047910, 3.62969330, 3.62170550, 3.59227300, 3.64464710};
int nextN = 0;

// =======================================================
// Global variables --------------------------------------

// Variables to sen and receive messages -----------------
#if MEGA2560_TRACE
  // Debug trace messages
  STDebug serialMsgDBG;
  // Declare auxiliar variable
  String debugMessage = "";
#endif
// Messages between MEGA2560 and ESP8266
STSerialMsg serialMsgESP(STSerialMsg::PORT_THREE);

// Variables to smart-delay messages ---------------------
#if MEGA2560_TRACE_INFO
  unsigned long lastTimeTrace = millis();
  // TRACE_TIME_PERIOD must be greater or equal than ANGLES_TIME_PERIOD
  const unsigned long TRACE_TIME_PERIOD = ANGLES_TIME_PERIOD; // milliseconds
#endif

// Global varibles ---------------------------------------
String strSatelliteListNames = "";
char *psSatelliteTLE[] = {"OSCAR 7 (AO-7)          ",
"1 07530U 74089B   20112.89879479 -.00000047  00000-0 -97882-5 0  9992",
"2 07530 101.7949  82.9674 0012534 112.0012 304.6453 12.53643112 78903"};

//// bool newSatelliteName = true;

String askTLEsatelliteName = "OSCAR 7 (AO-7)";

//// String msgInfoRead = "";
////  char msgCommandRead = ' ';

// GPS variables -----------------------------------------
const int DEFAULT_YEAR = 2020;
const byte DEFAULT_MONTH = 04;
const byte DEFAULT_DAY = 22;
const byte DEFAULT_HOUR = 12; //12; //11;
const byte DEFAULT_MINUTE = 50; //54; //0;
const byte DEFAULT_SECOND = 0;

// FIX-ME : Delete variable bool gpsReceiving
bool gpsReceiving = false;
uint32_t uGpsReceiving = 0x00;

TinyGPS gps;
long lat, lon;
float flat, flon;
unsigned long age = 0, date = 0, time = 0, chars = 0;
int year = DEFAULT_YEAR;
byte month = DEFAULT_MONTH, day = DEFAULT_DAY;
byte hour = DEFAULT_HOUR, minute = DEFAULT_MINUTE, second = DEFAULT_SECOND, hundredths = 0;

// Digital compass variables -----------------------------
// compass variances of roll X angle, pitch Y angle and yaw Z angle 
float sigma2compassX = 2*M_PI*2*M_PI, sigma2compassY = 2*M_PI*2*M_PI, sigma2compassZ = 2*M_PI*2*M_PI;
QMC5883L_ST compass(COMPASS_X_AXIS_POSITION, LOCAL_DECLINATION, LOCAL_INCLINATION, COMPASS_SD_X, COMPASS_SD_Y, COMPASS_SD_Z);
///QMC5883L_ST compass(COMPASS_X_AXIS_POSITION, LOCAL_DECLINATION, 0, COMPASS_SD_X, COMPASS_SD_Y, COMPASS_SD_Z);

// After the device is powered on, some time periods are 
// required for the device fully functional. 
// The external power supply requires a time period for 
// voltage to ramp up (PSUP), it is typically 
// 50 milli-second. However it isn’t controlled by the device.
// The power on/off time related to the device are:
// POR Completion Time: Time Period After VDD and VDDIO at Operating Voltage to Ready for I2C Commend and Analogy Measurement
// // const unsigned long PORCT = 1; // 350 micro seconds
// PINT: Power on Interval. Time Period Required for Voltage Lower Than SDV (0.2 volts) to Enable Next PORCT
// // const unsigned long PINT = 1; // 100 micro seconds
// Compass heading
float heading, horizontalHeadingAngle;
float rollXcompass = 0.0F, pitchYcompass = 0.0F, yawZcompass = 0.0F;
float rollXcompassInit = 0.0F, pitchYcompassInit = 0.0F, yawZcompassInit = 0.0F;
float rollXCompassOffset, pitchYCompassOffset;


// MPU6050 accelerometer Gyroscope -----------------------
// IMU Data
// Accelerometers: raw measurements values
double accX, accY, accZ;
// Gyroscopes: raw angle rates
double gyroX, gyroY, gyroZ;
// accelerometer gyroscope variances of roll X angle, pitch Y angle and yaw Z angle 
double sigma2accX , sigma2accY, sigma2accZ;
double sigma2gyroX, sigma2gyroY, sigma2gyroZ;
int16_t tempRaw;
// rollAccX: angle around X axis.
// pitchAccY: angle around Y axis.
// yawAccZ: angle around Z axis.
double rollAccX = 0.0F, pitchAccY = 0.0F, yawAccZ = 0.0F;
float yawAccZOffset;
double rollAccXInit, pitchAccYInit, yawAccZInit;
double gyroXrate = 0.0F, gyroYrate = 0.0F, gyroZrate = 0.0F;
#if MEGA2560_TRACE_TEST
  // Angle calculated using the gyro only
  double gyroXangle, gyroYangle, gyroZangle;
#endif
// Create the Kalman instance
Kalman_AGC kalmanX, kalmanY, kalmanZ;

// Create covariance matrices: H_k, Q_k
float H_diagonalX[3] = {M_PI*M_PI, GYRO_SD_X*GYRO_SD_X, M_PI*M_PI};
float H_diagonalY[3] = {M_PI*M_PI, GYRO_SD_Y*GYRO_SD_Y, M_PI*M_PI};
float H_diagonalZ[3] = {M_PI*M_PI, GYRO_SD_Z*GYRO_SD_Z, M_PI*M_PI};
float Q_diagonalX[2] = {ANGLE_SD_X*ANGLE_SD_X, ANGLE_RATE_SD_X*ANGLE_RATE_SD_X};
float Q_diagonalY[2] = {ANGLE_SD_Y*ANGLE_SD_Y, ANGLE_RATE_SD_Y*ANGLE_RATE_SD_Y};
float Q_diagonalZ[2] = {ANGLE_SD_Z*ANGLE_SD_Z, ANGLE_RATE_SD_Z*ANGLE_RATE_SD_Z};

// Define Kalman filtered state vector: [ANGLE, ANGLE_RATE]
// float auxKalmanState[2];
float auxKalmanState;

// Calculated angle using a Kalman filter
double kalAngleX, kalAngleY, kalAngleZ; 

uint32_t timer;
#if MEGA2560_TRACE_INFO
  uint32_t lastTimeRefreshTrace = 0;
#endif
double dt;
// Buffer for I2C data
uint8_t i2cDataMPU6050[14];
// MPU6050 Communication channel
// imjaviperez
I2Cdev i2cDevice;

// TODO: Make MPU6050  calibration routine


// Satellite attitude variables. Plan13 ------------------
double satAzimuth, satElevation;
#if MEGA2560_TRACE_INFO
  char asciiDateTime[20] ;
#endif
Satellite sat = Satellite("OSCAR 7 (AO-7)          ",
"1 07530U 74089B   20112.89879479 -.00000047  00000-0 -97882-5 0  9992",
"2 07530 101.7949  82.9674 0012534 112.0012 304.6453 12.53643112 78903");
DateTime  myDateTime ;

char DEFAULT_PLACE[] = "Bilbao"; // "Leioa";
const double DEFAULT_LATITUDE = 43.3000; // 43.328955;
const double DEFAULT_LONGITUDE = 2.9333; // -2.966181;
const double DEFAULT_ALTITUDE = 37.0; // 40.0; // meters

//   Observer postition. Plan13
char *observerPlace = DEFAULT_PLACE;
double observerLat = DEFAULT_LATITUDE;
double observerLong = DEFAULT_LONGITUDE;
double observerAlt = DEFAULT_ALTITUDE;
const Observer &observationPlace = Observer(observerPlace, observerLat, observerLong, observerAlt) ;


// Touch screen display variables (H35B-IC) ---------------
const uint8_t strMsgButtonUp[]="Button PREVIOUS!";
const uint8_t strMsgButtonDown[]="Button NEXT!";
const uint8_t strMsgFinishedSetup[]="Finished setup";

uint8_t  strSatName[25] = "QWERTY :(";
uint8_t  strPagenumber[] = "pp/ff";

uint8_t  checkSat01 = 0;
uint8_t  checkSat02 = 0;
uint8_t  checkSat03 = 0;

// LCD variables
uint8_t   identifier_lcd,cnt_lcd;
uint8_t   cmd_buffer_lcd[CMD_MAX_SIZE];
uint8_t   data_size_lcd;
uint8_t   update_lcd;
uint8_t   command_cmd_lcd;
uint8_t   command_status_lcd;
uint8_t   command_length_lcd;
uint8_t   page_id_lcd = 0;
uint8_t   targe_Id = 0;

LiquidCrystal TFTlcd(13);//RST pin13

// -------------------------------------------------------


// Status for asking arguments ---------------------------
bool askSatelliteNames = false;
bool askOneSatelliteTLE = false;

// Variables for smart delay sampling ====================
// Period of time to get GPS Date and Time (in millisecons)
unsigned long lastTimeGetGpsDT = millis();
unsigned long lastTimeGps = millis();
unsigned long lastTimeCompass = millis();
unsigned long lastTimeAngles = millis();
unsigned long lastTimeSatAttitude = millis();
unsigned long lastTimeLcdRefresh = millis();


// Variables to ask only one device per time, to do one task per time
class CircularArray
{
  public:
    CircularArray();
    ~CircularArray();
    enum devicesToComunicateWith {GPS_DATE_TIME, GPS_POSITION, COMPASS, ACC_GYRO, LCD};
    uint8_t next();
    uint8_t current();
  private:
    uint8_t m_currentDevice;
    // Create ordered list
    devicesToComunicateWith m_listOfDevices[5] = {GPS_DATE_TIME, GPS_POSITION, COMPASS, ACC_GYRO, LCD};
};


CircularArray::CircularArray()
{
  // First device to comunicate with
  m_currentDevice = devicesToComunicateWith::GPS_DATE_TIME;
}

CircularArray::~CircularArray()
{
}

uint8_t CircularArray::next()
{
  if ( (m_currentDevice+1) >= ((uint8_t)(sizeof(m_listOfDevices)/sizeof(m_listOfDevices[0]))) )
  {
    m_currentDevice = 0;
  }
  else
  {
    m_currentDevice++;
  }
  
  return m_listOfDevices[m_currentDevice];
}

uint8_t CircularArray::current()
{
  return m_listOfDevices[m_currentDevice];
}

CircularArray deviceToComunicateWith;


// Only for testing purpouses
bool anotherPeriodOfTime = true;
// End of Global variables -------------------------------
// =======================================================


void setup() {
  // =====================================================
  // Initialize Serial ports -----------------------------
  #if MEGA2560_TRACE
    serialMsgDBG.init();
  #endif  
  serialMsgESP.init();
  // Serial2 GPS
  Serial2.begin(STSerialMsg::GPS_BAUD_RATE);
  delay(1000);

  // Wait for Serial ports to be connetted ---------------
  #if MEGA2560_TRACE
  // Waiting for Serial3 and Serial avalability
  // ESP8266 sends TLE throw Serial3 port
    while(!serialMsgDBG.ready()){};
  #endif
  // ESP8266 Serial port
  while(!serialMsgESP.ready()){
      #if MEGA2560_TRACE_TEST
        if (millis() - lastTimeTrace >= TRACE_TIME_PERIOD){
          lastTimeTrace = millis();
          debugMessage = String(DBG_MSG_ESP_STARTING_UP);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, ORIGIN_MEGA2560, debugMessage);
        }
      #endif
  };
  // GPS Serial port
  while (!(Serial2)) {}
  // =====================================================

  // =====================================================
  // I2C communication setup
   // Compass and MPU6050 (accelerometer gyroscope) communication
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  // MPU6050 (accelerometer gyroscope) configuration
  i2cDataMPU6050[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cDataMPU6050[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cDataMPU6050[2] = GYRO_FULL_SCALE_RANGE_0250; // Set Gyro Full Scale Range to ±250deg/s
  i2cDataMPU6050[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  //// while (i2cDevice.i2cWrite(0x19, i2cDataMPU6050, 4, false)); // Write to all four registers at once
  while (!i2cDevice.writeBytes(IMU_MPU6050_ADDRS, 0x19, 4,i2cDataMPU6050)); // Write to all four registers at once

  //// while (i2cDevice.i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (!i2cDevice.writeByte(IMU_MPU6050_ADDRS,0x6B, 0x01)); // PLL with X axis gyroscope reference and disable sleep mode

  //// while (i2cDevice.i2cRead(0x75, i2cDataMPU6050, 1));
  while (!i2cDevice.readByte(IMU_MPU6050_ADDRS, 0x75, i2cDataMPU6050));
  if (i2cDataMPU6050[0] != IMU_MPU6050_ADDRS) { // Read "WHO_AM_I" register
    #if MEGA2560_TRACE_ERROR
      debugMessage = String("Error reading MPU6050 sensor");
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    //  TO-DO
    //    Change this error behaviour
    while (true);
  }

  // Time for VDD to rise from 10% to 90% of its final value
  delay(100); // Wait for sensor to stabilize
  // =====================================================


  // =====================================================
  // Initialize acelerometer gyroscope:
  getAccGyroDeviceInfo();
  rollAccXInit = atan2(accZ, accY) + M_PI_2;
  rollAccXInit = positiveAngle(rollAccXInit);
  pitchAccYInit = atan2(accX,accZ) - M_PI;
  pitchAccYInit = positiveAngle(pitchAccYInit);
  yawAccZInit = atan2(accY, accX);
  yawAccZInit = positiveAngle(yawAccZInit);


  transformAccelGyroInfo2Angles();

  

  timer = micros();
  // =====================================================
  
  // =====================================================
  #if MEGA2560_TRACE_TEST
    #ifdef ARDUINO_AVR_MEGA2560
        debugMessage = "DEFINED ARDUINO_AVR_MEGA2560";
        serialMsgDBG.printDebug(STDebug::STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #else
        debugMessage = "NOT DEFINED ARDUINO_AVR_MEGA2560";
        serialMsgDBG.printDebug(STDebug::STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
  #endif

  // =====================================================
  // Initialize Compass ----------------------------------
  // Compass communication
	compass.init();
	compass.setSamplingRate(COMPASS_SAMPLING_RATE);

  // We can get initial angles after compass.init()
  while (!compass.ready()){}
  // Set starting angle after initialize compass
  compass.readRotatedAngles(rollXcompass, pitchYcompass, yawZcompass);
  // Transform into positive angles
  rollXcompass = positiveAngle(rollXcompass);
  pitchYcompass = positiveAngle(pitchYcompass);
  yawZcompass = positiveAngle(yawZcompass);
  // Save starting offset angles
  rollXcompassInit = rollXcompass;
  pitchYcompassInit = pitchYcompass;
  yawZcompassInit = yawZcompass;
  // Save initial offsets
  rollXCompassOffset = (float) (rollAccXInit - rollXcompassInit);
  rollXCompassOffset = getPositiveAngle(rollXCompassOffset);
  pitchYCompassOffset = (float) (pitchAccYInit - pitchYcompassInit);
  pitchYCompassOffset = getPositiveAngle(pitchYCompassOffset);

  yawAccZOffset = (yawZcompassInit - yawAccZInit);
  yawAccZOffset = getPositiveAngle(yawAccZOffset);

  #if MEGA2560_TRACE_TEST
    debugMessage = String(rollXcompassInit*RAD_TO_DEG) + String("\t") + String(pitchYcompassInit*RAD_TO_DEG) + String("\t") + String(yawZcompassInit*RAD_TO_DEG);
    float locAngX, locAngY, locAngZ;
    compass.getLocalAngles(locAngX, locAngY, locAngZ);
    debugMessage += String("\t") + String(locAngX*RAD_TO_DEG) + String("\t") + String(locAngY*RAD_TO_DEG) + String("\t") + String(locAngZ*RAD_TO_DEG);
    debugMessage += String("\t") + String(rollAccXInit*RAD_TO_DEG) + String("\t") + String(pitchAccYInit*RAD_TO_DEG) + String("\t") + String(yawAccZInit*RAD_TO_DEG);
    debugMessage += String("\t") + String(rollXCompassOffset*RAD_TO_DEG) + String("\t") + String(pitchYCompassOffset*RAD_TO_DEG);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
  
  #if MEGA2560_TRACE_TEST
    gyroXangle = rollAccX;
    gyroYangle = pitchAccY;
    gyroZangle = yawZcompass;
  #endif
  // =====================================================


  // =====================================================
  // Initialize LCD ----------------------------------
   update_lcd = 0;
   data_size_lcd = 0;
   TFTlcd.queue_reset();

   // TO-DO :
   //  In order to manage many pages and buttons, create an interruption like this:
   //   attachInterrupt(digitalPinToInterrupt(compassInterruptionPin),LcdIICInterrupt,FALLING);//Interrupt 0 is D2 PIN

   TFTlcd.SetPage(ATTITUDE_PAGE);//main page id
  // =====================================================


  // =====================================================
  // Force reading from ESP8266 Serial3 port before loop() statement
  mySerialEvent3();
    
  #if MEGA2560_TRACE_TEST
    debugMessage = "End of setup()";
    serialMsgDBG.printDebug(STDebug::STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
  // =====================================================

  // =====================================================
  // Kalman filters initialization variables
  // a_1 = {ANGLE, ANGLE_RATE}
  float a1X[2] = {rollAccX,0.0F};
  float a1Y[2] = {pitchAccY,0.0F};
  float a1Z[2] = {yawZcompass,0.0F};
  // P_1 {Variance(angle), Variance(AngleRate)}
  float P1_diagonalX[2] = {M_PI*M_PI, 2.0F*M_PI*2.0F*M_PI};
  float P1_diagonalY[2] = {M_PI*M_PI, 2.0F*M_PI*2.0F*M_PI};
  float P1_diagonalZ[2] = {M_PI*M_PI, 2.0F*M_PI*2.0F*M_PI};

  // Calculate Accelerometer variance
  getAccVarAngles();
  // Gyroscope variance is constant: sigma2gyroX, sigma2gyroY, sigma2gyroZ
  getGyroVariance();
  // Calculate Compass variance
  compass.getVarianceAngle(sigma2compassX, sigma2compassY, sigma2compassZ);

  // H: observation error density covariance matrices
  H_diagonalX[Kalman_AGC::ACCELER] = sigma2accX;
  H_diagonalX[Kalman_AGC::GYROSPE] = sigma2gyroX;
  H_diagonalX[Kalman_AGC::COMPASS] = sigma2compassX;
  //
  H_diagonalY[Kalman_AGC::ACCELER] = sigma2accY;
  H_diagonalY[Kalman_AGC::GYROSPE] = sigma2gyroY;
  H_diagonalY[Kalman_AGC::COMPASS] = sigma2compassY;
  //
  H_diagonalZ[Kalman_AGC::ACCELER] = sigma2accZ;
  H_diagonalZ[Kalman_AGC::GYROSPE] = sigma2gyroZ;
  H_diagonalZ[Kalman_AGC::COMPASS] = sigma2compassZ;
  //
  // Q: state error density covariance matrices
  Q_diagonalX[Kalman_AGC::ANGLE] = ANGLE_SD_X*ANGLE_SD_X;
  Q_diagonalX[Kalman_AGC::ANGLE_RATE] = ANGLE_RATE_SD_X*ANGLE_RATE_SD_X;
  //
  Q_diagonalY[Kalman_AGC::ANGLE] = ANGLE_SD_Y*ANGLE_SD_Y;
  Q_diagonalY[Kalman_AGC::ANGLE_RATE] = ANGLE_RATE_SD_Y*ANGLE_RATE_SD_Y;
  //
  Q_diagonalZ[Kalman_AGC::ANGLE] = ANGLE_SD_Z*ANGLE_SD_Z;
  Q_diagonalZ[Kalman_AGC::ANGLE_RATE] = ANGLE_RATE_SD_Z*ANGLE_RATE_SD_Z;
  
  // pass pointer to the array as an argument.
  kalmanX.initialize(H_diagonalX, Q_diagonalX, a1X, P1_diagonalX);
  kalmanY.initialize(H_diagonalY, Q_diagonalY, a1Y, P1_diagonalY);
  kalmanZ.initialize(H_diagonalZ, Q_diagonalZ, a1Z, P1_diagonalZ);
  // =====================================================


  // Show Message strMsgFinishedSetup
  TFTlcd.Display_Message(0X18,3,(unsigned char *)strMsgFinishedSetup);
} // End of setup()


void loop() {
  // Change status periodically ==========================
  // TO-DO
  //    Change askSatelliteNames and askOneSatelliteTLE from LCD touch screen action

  // Only for testing purpouses --------------------------
  anotherPeriodOfTime = true;
  // Change status periodically
  #if MEGA2560_TRACE_INFO
    if (millis() - lastTimeTrace >= TRACE_TIME_PERIOD){
      lastTimeTrace = millis();
      // debugMessage = String("anotherPeriodOfTime=") + String(anotherPeriodOfTime);
      // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    }else{
      anotherPeriodOfTime = false;
    }
  #endif
  if (anotherPeriodOfTime)
  {
      askSatelliteNames = false;
      askOneSatelliteTLE = false;
  }
  // End of Only for testing purpouses -------------------
  // End of Change status periodically ===================

  
  // =====================================================
  // Smart delays and get information from devices =======
  // If there is no LCD interruption we comunicate next device
  if (update_lcd == 0)
  {
    deviceToComunicateWith.next();
  }
  #if MEGA2560_TRACE_TEST
    debugMessage = String("deviceToComunicateWith.current()=") + String(deviceToComunicateWith.current())
    + String(".update_lcd=") + String(update_lcd);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif

  // GPS smart delay -------------------------------------
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::GPS_DATE_TIME) &&
    (millis() - lastTimeGps >= GPS_TIME_PERIOD)){
    lastTimeGps = millis();
    // Read info from GPS: date, time and observer position
    getGPSInfo();
    observerLat = (double) flat;
    observerLong = (double) flon;
    observerAlt = (double) gps.f_altitude();
    //// askGPSDateTime = true;
  }
  //Compass smart delay ----------------------------------
  // Get device compass orientation every while
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::COMPASS) &&
    (millis() - lastTimeCompass >= COMPASS_TIME_PERIOD)){
    lastTimeCompass = millis();
    // Calculate device North-South orientation ----------
    // Get compass value
    compass.readRotatedAngles(rollXcompass, pitchYcompass, yawZcompass);

    // Add offset to these angles
    rollXcompass += rollXCompassOffset;
    pitchYcompass += pitchYCompassOffset;
    
    // Transform into positive angles
    rollXcompass = positiveAngle(rollXcompass);
    pitchYcompass = positiveAngle(pitchYcompass);
    yawZcompass = positiveAngle(yawZcompass);
    /// heading = compass.readHeading();
    heading = yawZcompass;
    
    //// // Add local declination correction
    //// heading += (int16_t) LOCAL_DECLINATION;

    // The member function readHeading() gets North heading in the device XY plane, 
    // but we need a projection of heading in the ground horizontal plane.
    // horizontalHeading() creates a projection of this vector in the ground plane
    horizontalHeading();
  }

  // Accelerometer-Gyroscope smart delay -----------------
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::ACC_GYRO) &&
    (millis() - lastTimeAngles >= ANGLES_TIME_PERIOD)){
    lastTimeAngles = millis();
    // Get device angles attitude
    getAccGyroDeviceInfo();
    // Calculate angles, gyro rate and kalman filter
    calculateRotationAngles();
    //// askDeviceAngles = true;
  }

  // Calculate satellite attitude smart delay ------------
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::GPS_POSITION) &&
    (millis() - lastTimeSatAttitude >= SAT_ATTITUDE_TIME_PERIOD))
  {
    lastTimeSatAttitude = millis();
    getSatelliteAttitude();     
    //// calculateSatelliteAttitude = false;
  }
  // Refresh LCD Smart delay -----------------------------
  if ((update_lcd == 1) || 
    ((deviceToComunicateWith.current() == CircularArray::LCD) && (millis() - lastTimeLcdRefresh >= SAT_LCD_TIME_PERIOD)))
  {
    lastTimeLcdRefresh = millis();
    update_lcd = 0;

    // TO-DO :
    // Create three pages: main, atttude, select_satellites
    // Update only attitude page
    page_id_lcd = ATTITUDE_PAGE;
    UpdateUI();
  }
  // End of Smart delays =================================

  // Ask ESP8266 for information =========================
  // Request Satellite Names -----------------------------
  // TO-DO
  //    Change value of askSatelliteNames = true from LCD
  if (askSatelliteNames)
  {
    bool result = serialMsgESP.requestSatelliteNames();
    if (result)
    {
      askSatelliteNames = false;
    }else{
      #if MEGA2560_TRACE_WARNING
        debugMessage = String("ERROR rSN=");
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
    }
  }

  // Request one Satellite TLE ---------------------------
  // TO-DO
  //    Change value of askOneSatelliteTLE = true from LCD
  if (askOneSatelliteTLE)
  {
    if (!serialMsgESP.requestSatelliteTLE(askTLEsatelliteName))
    {
      #if MEGA2560_TRACE_WARNING
        debugMessage = String("ERROR rSTLE Name=") + askTLEsatelliteName;
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
    }else{
      askOneSatelliteTLE = false;
    }
  }
  // End of Ask ESP8266 for information ==================


  // =====================================================
  // Find LCD commands: touch button, change screen, etc
  data_size_lcd = TFTlcd.queue_find_cmd(cmd_buffer_lcd,CMD_MAX_SIZE);
  if(data_size_lcd>0)//receive command
  {
    // Serial.println(data_size_lcd, HEX);
    // Serial.println(F("ProcessMessage"));
    ProcessMessage((PCTRL_MSG)cmd_buffer_lcd, data_size_lcd);//command process
  } 

  // =====================================================


  // Force reading from Serial3 port during loop() statement
  //// serialEvent3();
  mySerialEvent3();

  // 
  #if MEGA2560_TRACE_INFO
    if (millis() - lastTimeRefreshTrace >= TRACE_TIME_PERIOD) // TRACE_TIME_PERIOD
    {
      showTraceInfo();
      lastTimeRefreshTrace = millis();
    }
  #endif

 } // end: loop()

// =======================================================
// Functions =============================================

// Checking an event on the Serial3 port
// SerialEvent occurs whenever a new data comes in the hardware serial RX. 
// This routine is run between each time loop() runs
//// void serialEvent3() 
void mySerialEvent3() 
{ 
  const int TLE_LINE_0_LENGTH = 24;
  const int TLE_LINE_1_LENGTH = 69;
  const int TLE_LINE_2_LENGTH = 69;

  #if MEGA2560_TRACE_TEST
    // Show data on serial monitor
    if (serialMsgESP.available() > 0) 
    {
      debugMessage = String("serialMsgESP.available()=") + String(serialMsgESP.available());
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    }
  #endif
  //// serialMsgESP.readSerialMsg(msgCommandRead, msgInfoRead);
  serialMsgESP.readSerialMsg();
  if (serialMsgESP.newCommandRead())
  {
    String tmpCommandInfo = serialMsgESP.commandInfoRead();
    switch (serialMsgESP.commandRead())
    {
      case STSerialMsg::PUT_SAT_NAMES:
        // The ESP8266 has PUT_SAT_NAMES. So let's read them
        //// strSatelliteListNames = serialMsgESP.commandInfo();
        strSatelliteListNames = tmpCommandInfo; //serialMsgESP.commandInfo();
        #if MEGA2560_TRACE_TEST // MEGA2560_TRACE_INFO
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_SAT_NAMES) + strSatelliteListNames;
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::PUT_TLE0_SATELLITE:
        tmpCommandInfo = serialMsgESP.commandInfoRead();
        tmpCommandInfo.toCharArray(psSatelliteTLE[0], TLE_LINE_0_LENGTH);
        #if MEGA2560_TRACE_TEST
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_TLE_LINE_0) + String(psSatelliteTLE[0]);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::PUT_TLE1_SATELLITE:
        tmpCommandInfo = serialMsgESP.commandInfoRead();
        tmpCommandInfo.toCharArray(psSatelliteTLE[1], TLE_LINE_1_LENGTH);
        #if MEGA2560_TRACE_TEST
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_TLE_LINE_1) + String(psSatelliteTLE[1]);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::PUT_TLE2_SATELLITE:
        tmpCommandInfo = serialMsgESP.commandInfoRead();
        tmpCommandInfo.toCharArray(psSatelliteTLE[2], TLE_LINE_2_LENGTH);
        #if MEGA2560_TRACE_TEST
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_TLE_LINE_2) + String(psSatelliteTLE[2]);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::DBG_MSG:
      // Do not do anything with DBG_MSG
      default:
        break;
    }

    // After reading and saving message, make a reset serialMsgESP message
    // to let Serial read more information next loop.
    serialMsgESP.resetCommandRead();

  }else{
      #if MEGA2560_TRACE_TEST
        // Show data on serial monitor
        debugMessage = "No message read";
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif

  }
}

inline void showTraceInfo()
{
  // Number of decimals
  const byte N_DEC = 8;
  #if MEGA2560_TRACE_INFO
  
    debugMessage = String(timer) + String("\t") + String(dt) 
    + String("\t") + String(rollAccX, N_DEC) + String("\t") + String(pitchAccY, N_DEC) + String("\t") + String(yawAccZ, N_DEC)
    #if MEGA2560_TRACE_TEST
    + String("\t")  + String(gyroXangle) + String("\t") + String(gyroYangle)
    #endif
    + String("\t")  + String(gyroXrate, N_DEC) + String("\t") + String(gyroYrate, N_DEC) + String("\t") + String(gyroZrate, N_DEC)
    + String("\t")  + String(rollXcompass, N_DEC)+ String("\t")  + String(pitchYcompass, N_DEC) + String("\t") + String(yawZcompass, N_DEC)
    + String("\t") + String(kalAngleX, N_DEC) + String("\t") + String(kalAngleY, N_DEC) + String("\t") + String(kalAngleZ, N_DEC)
    + String("\t") + String(sigma2accX, N_DEC) + String("\t") + String(sigma2accY, N_DEC) + String("\t") + String(sigma2accZ, N_DEC)
    + String("\t") + String(sigma2gyroX, N_DEC) + String("\t") + String(sigma2gyroY, N_DEC) + String("\t") + String(sigma2gyroZ, N_DEC)
    + String("\t") + String(sigma2compassX, N_DEC) + String("\t") + String(sigma2compassY, N_DEC) + String("\t") + String(sigma2compassZ, N_DEC)
    + String("\t") + String(heading, N_DEC)  
    + String("\t") + String(horizontalHeadingAngle, N_DEC);

    // debugMessage += String("\t") + String(accX) + String("\t") +  String(accY) + String("\t") +  String(accZ);
    debugMessage += String("\t") + String(compass.rawValueX()) + String("\t") +  String(compass.rawValueY()) + String("\t") +  String(compass.rawValueZ());


    // // Compass Variance
    // debugMessage = String(sigma2compassX, N_DEC) + String("\t") + String(sigma2compassY, N_DEC) + String("\t") + String(sigma2compassZ, N_DEC); 

    // // GPS receiving
    // debugMessage = String(uGpsReceiving) 
    // + String("\t") + String(year) + String("\t") + String(month) + String("\t") + String(day)
    // + String("\t") + String(hour) + String("\t") + String(minute) + String("\t") + String(second);

    // // Kalman and sd Acc
    // debugMessage = String(kalAngleX*RAD_TO_DEG) + String("\t") + String(kalAngleY*RAD_TO_DEG) + String("\t") + String(kalAngleZ*RAD_TO_DEG);
    // debugMessage += String("\t") + String(sigma2accX) + String("\t") + String(sigma2accY) + String("\t") + String(sigma2accZ);

    // // Raw compass
    // int16_t compassX, compassY, compassZ;
    // compass.m_readRaw(&compassX, &compassY, &compassZ,false);
    // debugMessage = String(compassX) + String("\t") + String(compassY) + String("\t") + String(compassZ);

    // Raw info acc, gyro, compass. Measures with device in stable position
    // int16_t compassX, compassY, compassZ;
    // compass.m_readRaw(&compassX, &compassY, &compassZ,false);
    // debugMessage = String(accX) + String("\t") +  String(accY) + String("\t") +  String(accZ)
    //  + String("\t") +  String(gyroXrate, NUM_DECIMALS) + String("\t") + String(gyroYrate, NUM_DECIMALS) + String("\t") + String(gyroZrate, NUM_DECIMALS)
    //  + String("\t") + String(compassX) + String("\t") + String(compassY) + String("\t") + String(compassZ);

    // debugMessage = String(gyroXrate) + String("\t") + String(gyroYrate) + String("\t") + String(gyroZrate)
    //  + String("\t") + String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ);
    
    // debugMessage = String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ);
    // debugMessage +=  String("\t") + String(accX) + String("\t") + String(accY) + String("\t") + String(accZ);
    // debugMessage += String("\t") + String(gyroX) + String("\t") + String(gyroY) + String("\t") + String(gyroZ);
    #if MEGA2560_TRACE_TEST
      // debugMessage += String(gyroXangle) + String("\t") + String(gyroYangle) + String("\t") + String(gyroZangle);
    #endif
    
    // int16_t cx,cy,cz;
    // compass.m_readRaw(&cx,&cy,&cz,true);
    // debugMessage = String(cx) + String("\t") + String(cy) + String("\t") + String(cz)
    //  + String("\t") +   String(rollXcompass) + String("\t") + String(pitchYcompass) + String("\t") + String(yawZcompass);

    // debugMessage = String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ)
    //  + String("\t") +   String(yawZcompass) 
    //  + String("\t") + String(horizontalHeadingAngle)
    //  + String("\t") + String(kalAngleX) + String("\t") + String(kalAngleY) + String("\t") + String(kalAngleZ);

    // // horizontalHeadingAngle
    // debugMessage = String(heading*RAD_TO_DEG) + String("\t") + String(horizontalHeadingAngle*RAD_TO_DEG);
    // // Kalman
    // //debugMessage = String("\t") + String(kalAngleX*RAD_TO_DEG) + String("\t") + String(kalAngleY*RAD_TO_DEG) + String("\t") + String(kalAngleZ*RAD_TO_DEG);

    // 
    // int lcdAntennaAscension = (int) ((M_PI_2 + M_PI - heading) * RAD_TO_DEG);
    // lcdAntennaAscension = positiveAngleDegrees(lcdAntennaAscension);
    // debugMessage += String("\t") + String(lcdAntennaAscension);
    // 
    //  + String("\t") + String(kalAngleX) + String("\t") + String(kalAngleY) + String("\t") + String(kalAngleZ);
    // float locRoll, locPitch, locYaw;
    // compass.getLocalAngles(locRoll, locPitch, locYaw);
    // debugMessage += String("\t") + String(locRoll*RAD_TO_DEG) + String("\t") + String(locPitch*RAD_TO_DEG) + String("\t") + String(locYaw*RAD_TO_DEG);

    // Angles
    // debugMessage = String(timer) 
    // + String("\t") + String(rollAccX * RAD_TO_DEG) + String("\t") + String(pitchAccY * RAD_TO_DEG) + String("\t") + String(yawAccZ * RAD_TO_DEG);
    // + String("\t")  + String(gyroXrate, N_DEC) + String("\t") + String(gyroYrate, N_DEC) + String("\t") + String(gyroZrate, N_DEC)
    // debugMessage += String("\t")  + String(rollXcompass * RAD_TO_DEG)+ String("\t")  + String(pitchYcompass * RAD_TO_DEG) + String("\t") + String(yawZcompass * RAD_TO_DEG);
    // debugMessage += String("\t") + String(kalAngleX * RAD_TO_DEG) + String("\t") + String(kalAngleY * RAD_TO_DEG) + String("\t") + String(kalAngleZ * RAD_TO_DEG);
    // float locRoll, locPitch, locYaw;
    // compass.getLocalAngles(locRoll, locPitch, locYaw);
    // debugMessage += String("\t") + String(locRoll*RAD_TO_DEG) + String("\t") + String(locPitch*RAD_TO_DEG) + String("\t") + String(locYaw*RAD_TO_DEG);

    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
}

inline void getGPSInfo()
{
  // Every TIME_PERIOD_GPS_DT milliseconds we ask for an update
  if (millis() - lastTimeGetGpsDT <= TIME_PERIOD_GPS_DT)
  {
    bool newdata = false;
    // unsigned long start = millis();
    #if MEGA2560_TRACE_TEST
      debugMessage = String("S2.available()=") + String(Serial2.available());
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    while (Serial2.available()) 
    {
      char c = Serial2.read();
      #if MEGA2560_TRACE_TEST
        // To see raw GPS data
        // Serial.print(c);
        // debugMessage = String(c);
        // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      if (gps.encode(c)) 
      {
        newdata = true;
        #if MEGA2560_TRACE_TEST
          debugMessage = "NEW gps DATA";
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        // break; // uncomment to print new data immediately!
      }
    }

    if (newdata) 
    {
      gps.get_position(&lat, &lon, &age);
      // gps.f_get_position(&flat, &flon, &age);
      gps.f_get_position(&flat, &flon, &age);
      gps.get_datetime(&date, &time, &age);
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
      // gps.stats(&chars, &sentences, &failed);
      #if MEGA2560_TRACE_TEST
        debugMessage = String("GPS OK");
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
    }
    lastTimeGetGpsDT = millis();
    gpsReceiving = newdata;
    // Shift left 1 position
    uGpsReceiving = uGpsReceiving << 1;
    // Add new state at first position
    uGpsReceiving |= (uint16_t) newdata;

  }
}

/**
 * @brief Get the Acc Gyro Device Info object
 * X axis will be a virtual forward axis and
 * Y axis will be a virtual left axis.
 * Z axis is a real upward axis.
 * 
 * This device originally gets positive accZ values when 
 * it is stabilized on a desk, but we whish to obtain 
 * negative values in that position, so we change the
 * obtaind signs.
 * 
 */
inline void getAccGyroDeviceInfo()
{
  // Update all the values
  while (!i2cDevice.readBytes(IMU_MPU6050_ADDRS, 0x3B, 14, i2cDataMPU6050));

  // Get values and change sign
  //accX = (int16_t)((i2cDataMPU6050[0] << 8) | i2cDataMPU6050[1]);
  accX = -(int16_t)((i2cDataMPU6050[0] << 8) | i2cDataMPU6050[1]);
  //accY = (int16_t)((i2cDataMPU6050[2] << 8) | i2cDataMPU6050[3]);
  accY = -(int16_t)((i2cDataMPU6050[2] << 8) | i2cDataMPU6050[3]);
  // accZ = (int16_t)((i2cDataMPU6050[4] << 8) | i2cDataMPU6050[5]);
  accZ = -(int16_t)((i2cDataMPU6050[4] << 8) | i2cDataMPU6050[5]);
  tempRaw = (int16_t)((i2cDataMPU6050[6] << 8) | i2cDataMPU6050[7]);
  // gyroX = (int16_t)((i2cDataMPU6050[8] << 8) | i2cDataMPU6050[9]);
  gyroX = -(int16_t)((i2cDataMPU6050[8] << 8) | i2cDataMPU6050[9]);
  // gyroY = (int16_t)((i2cDataMPU6050[10] << 8) | i2cDataMPU6050[11]);
  gyroY = -(int16_t)((i2cDataMPU6050[10] << 8) | i2cDataMPU6050[11]);
  // gyroZ = (int16_t)((i2cDataMPU6050[12] << 8) | i2cDataMPU6050[13]);
  gyroZ = -(int16_t)((i2cDataMPU6050[12] << 8) | i2cDataMPU6050[13]);

  // Rotate accelerometer and gyroscope readings to get a coordinate system
  // with X forward.
  // Rotate X and Y angles
  if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_FORWARD) // 0
  {// X = X', Y = Y'
    // Do not do anything
  }
  else if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_LEFT) //90
  {
    int16_t tmpX = accX;
    accX = -accY; // X = - Y'
    accY = tmpX; // Y = X'

    tmpX = gyroX;
    gyroX = -gyroY;
    gyroY = tmpX;
  }
  else if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_BACKWARD) // 180
  {
    accX = -accX; // X = -X'
    accY = -accY; // Y = - Y'

    gyroX = -gyroX;
    gyroY = -gyroY;
  }
  else if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_RIGHT) // 270 
  {
    int16_t tmpX = accX;
    accX = accY; // X = Y'
    accY = -tmpX; // Y = -X'

    tmpX = gyroX;
    gyroX = gyroY;
    gyroY = -tmpX;
  }
}

/**
 * @brief Get the Acceelerometer Variance  Angles: 
 * rollX, pitchY, yawZ in (radians)^2
 * 
 */
inline void getAccVarAngles()
{
  // Calculate standard deviation
  double sigmaAccEast =  sdAngleAtanDiff(accZ, accY, ACC_SD_Z, ACC_SD_Y);
  double sigmaAccNorth = sdAngleAtanDiff(accX, accZ, ACC_SD_X, ACC_SD_Z);
  double sigmaAccZ =     sdAngleAtanDiff(accY, accX, ACC_SD_Y, ACC_SD_X);

  // SD angle is between [0, PI]
  sigmaAccEast = min(abs(sigmaAccEast), 2*M_PI);
  sigmaAccNorth = min(abs(sigmaAccNorth), 2*M_PI);
  sigmaAccZ = min(abs(sigmaAccZ), 2*M_PI);

  // Calculate variance
  sigma2accX =  sigmaAccEast * sigmaAccEast;
  sigma2accY =  sigmaAccNorth * sigmaAccNorth;
  sigma2accZ =  sigmaAccZ * sigmaAccZ;
}


/**
 * @brief Get the Gyro Variance 
 * It is easy to calculate gyroscope variance because it is constant
 * 
 */
inline void getGyroVariance()
{
  sigma2gyroX = GYRO_SD_X * GYRO_SD_X;
  sigma2gyroY = GYRO_SD_Y * GYRO_SD_Y;
  sigma2gyroZ = GYRO_SD_Z * GYRO_SD_Z;
}


inline void calculateRotationAngles()
{
  // Calculate delta time in seconds
  dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  transformAccelGyroInfo2Angles();

  // Calculate gyro X Y Z rates
  gyroXrate = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR_0250_RADS; // Convert to rad/s
  gyroYrate = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR_0250_RADS; // Convert to rad/s 
  gyroZrate = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR_0250_RADS; // Convert to rad/s 

  // Set values of H matrix: H_diagonalX[3] = {Variance(AccelerometerX), Variance(GyroscopeX), Variance(CompassX)}
  // 1. Calculate Accelerometer variance
  getAccVarAngles();
  // 2. Gyroscope variance is constant: sigma2gyroX, sigma2gyroY, sigma2gyroZ
  // 3. Calculate Compass variance
  compass.getVarianceAngle(sigma2compassX, sigma2compassY, sigma2compassZ);
  // 
  H_diagonalX[Kalman_AGC::ACCELER] = sigma2accX;
  H_diagonalX[Kalman_AGC::GYROSPE] = sigma2gyroX;
  H_diagonalX[Kalman_AGC::COMPASS] = sigma2compassX;

  H_diagonalY[Kalman_AGC::ACCELER] = sigma2accY;
  H_diagonalY[Kalman_AGC::GYROSPE] = sigma2gyroY;
  H_diagonalY[Kalman_AGC::COMPASS] = sigma2compassY;

  H_diagonalZ[Kalman_AGC::ACCELER] = sigma2accZ;
  H_diagonalZ[Kalman_AGC::GYROSPE] = sigma2gyroZ;
  H_diagonalZ[Kalman_AGC::COMPASS] = sigma2compassZ;

  // Calculate angles using Kalman filter
  kalmanX.filteredState(rollAccX, gyroXrate, rollXcompass, dt, H_diagonalX, auxKalmanState);
  /// kalAngleX = auxKalmanState[ANGLE];
  kalAngleX = auxKalmanState;
  kalmanY.filteredState(pitchAccY, gyroYrate, pitchYcompass, dt, H_diagonalY, auxKalmanState);
  /// kalAngleY = auxKalmanState[ANGLE];
  kalAngleY = auxKalmanState;
  kalmanZ.filteredState(yawAccZ, gyroZrate, yawZcompass, dt, H_diagonalZ, auxKalmanState);
  /// kalAngleZ = auxKalmanState[ANGLE];
  kalAngleZ = auxKalmanState;


  #if MEGA2560_TRACE_TEST
    // Calculate gyro angle without any filter
    gyroXangle += gyroXrate * dt;
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    // Reset the gyro angle when it has drifted too much
    gyroXangle = (float)angle_pm180Radians(gyroXangle);
    gyroYangle = (float)angle_pm180Radians(gyroYangle);
    gyroZangle = (float)angle_pm180Radians(gyroZangle);
  #endif
}

inline void getSatelliteAttitude()
{
  // TO-DO
  //    Usar string proveniente del ESP8266 para crear la variable sat
  /// char *tle0, *tle1, *tle2;
  /// psSatelliteTLE[0].toCharArray(tle0, psSatelliteTLE[0].length());
  /// psSatelliteTLE[1].toCharArray(tle1, psSatelliteTLE[1].length());
  /// psSatelliteTLE[2].toCharArray(tle2, psSatelliteTLE[2].length());

  #if false // MEGA2560_TRACE_TEST
      debugMessage = String("psSatelliteTLE[0]=") + String(psSatelliteTLE[0]) + 
                    String("\tpsSatelliteTLE[1]=") + String(psSatelliteTLE[1]) + 
                    String("\tpsSatelliteTLE[2]=") + String(psSatelliteTLE[2]);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif

  sat = Satellite(psSatelliteTLE[0], psSatelliteTLE[1], psSatelliteTLE[2]);

  // Prediction time
  myDateTime.settime(year, (int)month, (int)day, (int)hour, (int)minute, (int)second);
  // Create prediction
  sat.predict(myDateTime);
  // satElevation and satAzimuth from observer place
  sat.altaz(observationPlace, satElevation, satAzimuth);




  /*
  char buf[20] ;
  for (int hh = 12; hh < 24; hh++)
  {
    for (int mm = 0; mm < 60; mm+=5) 
    {        
      myDateTime.settime(2020, 04, 22, hh, mm, 0) ;        // genera momento para la predicción
      sat.predict(myDateTime) ;                           // genera predicción
      myDateTime.ascii(buf) ;                             // fecha en ASCII para imprimir más abajo
      sat.altaz(observationPlace, satElevation, satAzimuth) ;          // elevación y azimut desde ubicación proporcionada
      
      if ((satAzimuth > 0) && (satAzimuth < 90) && (satElevation > 0) && (satElevation < 360))
      {
        Serial.print(buf) ;                            // imprime resultados
        Serial.print("\t") ;
        Serial.print( sat.name ) ;                      
        Serial.print("\t") ;
        Serial.print(satAzimuth) ;
        Serial.print("\t") ;
        Serial.println(satElevation) ;                      
      }
    }
  }
  */



}

/**
 * @brief This function transforms accelerometer and 
 * gyroscope information into angles.
 * Every calculated angle is positive between [0,2*M_PI]
 * 
 */
inline void transformAccelGyroInfo2Angles()
{
  const float OFFSET_ROLL_X = -M_PI_2;
  const float OFFSET_PITCH_Y = M_PI;
  const float OFFSET_YAW_Z = 0.0F - yawAccZOffset ;
  
  rollAccX = atan2(accZ, accY) - OFFSET_ROLL_X;
  pitchAccY = atan2(accX,accZ) - OFFSET_PITCH_Y;
  yawAccZ = atan2(accY, accX) - OFFSET_YAW_Z;

  // atan2() returns angles between [-PI,PI] 
  // but we will use angles between [0,2*M_PI]
  rollAccX = positiveAngle(rollAccX);
  pitchAccY = positiveAngle(pitchAccY);
  yawAccZ = positiveAngle(yawAccZ);
}



/**
 * @brief Writes horizontalHeadingAngle variable.
 * The member function readHeading() gets North heading in
 * the device XY plane, but we need a projection of 
 * heading in the ground horizontal plane.
 * horizontalHeading() create a projection of this vector
 * in the ground plane writing the angle variable horizontalHeadingAngle.
 * It has been inspired by Bingaman 2010 [pag.74], Grygorenko 2014 and Abyarjoo 2015
 */
inline void horizontalHeading()
{
  //// const float OFFSET_ROLL_X = -M_PI_2;
  //// const float OFFSET_PITCH_Y = M_PI;
  //// const float OFFSET_YAW_Z = 0.0F - yawAccZOffset ;
  //// double aux_rollAccX = rollAccX + OFFSET_ROLL_X;
  //// double aux_pitchAccY = atan2(accX,accZ) + OFFSET_PITCH_Y;
  //// double aux_yawAccZ = atan2(accY, accX) + OFFSET_YAW_Z;

  // Read compass raw values
  double mx = compass.rawValueX();
  double my = compass.rawValueY();
  double mz = compass.rawValueZ();

  // Euler rotations
  // Accelerometer angles
  double xh = mx * cos(pitchAccY) - mz * sin(pitchAccY);
  double yh = mx * sin(pitchAccY)*sin(rollAccX) + my * cos(rollAccX) + mz * sin(rollAccX)*cos(pitchAccY);
  // double zh = mx * sin(pitchAccY)*cos(rollAccX) - my * sin(rollAccX) + mz * cos(pitchAccY)*cos(rollAccX);
  /// // Kalman angles
  /// double xh = mx * cos(kalAngleY) - mz * sin(kalAngleY);
  /// double yh = mx * sin(kalAngleY)*sin(kalAngleX) + my * cos(kalAngleX) + mz * sin(kalAngleX)*cos(kalAngleY);
  /// // double zh = mx * sin(kalAngleY)*cos(kalAngleX) - my * sin(kalAngleX) + mz * cos(kalAngleY)*cos(kalAngleX);

  // Calculate angle and add local declination correction
  horizontalHeadingAngle = atan2(yh,xh); // - (M_PI_2 + LOCAL_DECLINATION);
  // Allow only positive values
  horizontalHeadingAngle = positiveAngle(horizontalHeadingAngle);
}

/**
 * @brief Writes horizontalHeadingAngle variable.
 * The member function readHeading() gets North heading in
 * the device XY plane, but we need a projection of 
 * heading in the ground horizontal plane.
 * horizontalHeading() create a projection of this vector
 * in the ground plane writing the angle variable horizontalHeadingAngle.
 * It has been inspired by Bingaman 2010 [pag.74], Grygorenko 2014 and Abyarjoo 2015
 */
inline void horizontalHeading_OLD_BORRAME()
{
  int16_t rawX, rawY, rawZ;
  // Read compas values adding compass orientation offset.
  ////if (compass.readRawNormalizedRotated(mx,my,mz))
  if (compass.readRawRotated(rawX, rawY, rawZ))
  {
    double mx, my, mz;
    mx = (double) rawX;
    my = (double) rawY;
    mz = (double) rawZ;
    
    // Euler rotations
    // 2_1 Rx*Ry: horizontalHeadingAngle
    //// [cos(pitchAccY), 0, -sin(pitchAccY)], 
    //// [sin(pitchAccY)*sin(rollAccX), cos(rollAccX), sin(rollAccX)*cos(pitchAccY)], 
    //// [sin(pitchAccY)*cos(rollAccX), -sin(rollAccX), cos(pitchAccY)*cos(rollAccX)]
    double xh = mx * cos(pitchAccY) - mz * sin(pitchAccY);
    double yh = mx * sin(pitchAccY)*sin(rollAccX) + my * cos(rollAccX) + mz * sin(rollAccX)*cos(pitchAccY);
    // double zh = mx * sin(pitchAccY)*cos(rollAccX) - my * sin(rollAccX) + mz * cos(pitchAccY)*cos(rollAccX);
    // Calculate angle and add local declination correction
    horizontalHeadingAngle = atan2(yh,xh) - (M_PI_2 + LOCAL_DECLINATION);
    // Allow only positive values
    horizontalHeadingAngle = positiveAngle(horizontalHeadingAngle);
  }
}

void LcdIICInterrupt()
{
  command_cmd_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(command_cmd_lcd);
  for(cnt_lcd =0;cnt_lcd <2;cnt_lcd++)
  {
    identifier_lcd = TFTlcd.I2C_Read();
    TFTlcd.queue_push(identifier_lcd);
    // Serial.print(command_cmd_lcd, HEX);
    // Serial.print('\t');
    // Serial.println(identifier_lcd, HEX);
  }
  command_status_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(command_status_lcd);

  identifier_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(identifier_lcd);

  command_length_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(command_length_lcd);

  /* We do not use EDIT_VALUE
  if((command_cmd_lcd == GET_EDIT_VALUE && command_status_lcd == SUCCESS)||
      (command_cmd_lcd == GET_TOUCH_EDIT_VALUE && command_status_lcd == SUCCESS))
  {
      for(cnt_lcd =0;cnt_lcd <command_length_lcd;cnt_lcd++)
    {
      identifier_lcd = TFTlcd.I2C_Read();
      TFTlcd.queue_push(identifier_lcd);
      //Serial.println(identifier_lcd, HEX);
    }
  }
  */
}

void ProcessMessage( PCTRL_MSG msg, uint16_t dataSize )
{
    uint8_t cmd_type    = msg->cmd_type;
    uint8_t control_id  = msg->control_id;
    uint8_t page_id_lcd     = msg->page_id;
    uint8_t _status     = msg->status;
    uint8_t key_type    = msg->key_type;
    uint8_t key_value   = msg->key_value;
    #if MEGA2560_TRACE_INFO
      debugMessage = String("ProcessMessage()")
      + String("\t") + String(cmd_type, HEX) 
      + String("\t") + String(page_id_lcd) 
      + String("\t") + String(control_id) 
      + String("\t") + String(_status, HEX) 
      + String("\t") + String(key_type, HEX) 
      + String("\t") + String(key_value, HEX);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif

    switch(cmd_type)
    {
    case NOTIFY_TOUCH_BUTTON:
      NotifyTouchButton(page_id_lcd,control_id,_status,key_type,key_value);
      break;
  
    case NOTIFY_TOUCH_CHECKBOX:
      NotifyTouchCheckbox(page_id_lcd,control_id,_status,key_type,key_value);
      break;
  
    case NOTIFY_TOUCH_SLIDER:
      NotifyTouchSlider(page_id_lcd,control_id,_status,key_type,key_value);
      break;

    case NOTIFY_TOUCH_EDIT:
    NotifyTouchEdit(page_id_lcd,control_id,_status,key_type,key_value);
    break;
    
    case NOTIFY_GET_EDIT:
     NotifyGetEdit((PEDIT_MSG)cmd_buffer_lcd);
      break;

    case NOTIFY_GET_TOUCH_EDIT:
    NotifyGetTouchEdit((PEDIT_MSG)cmd_buffer_lcd);
    break;
  
    case NOTIFY_GET_PAGE:
      NotifyGetPage(page_id_lcd,_status);
      break;
  
    case NOTIFY_GET_CHECKBOX:
      NotifyGetCheckbox(page_id_lcd,control_id,_status,key_type,key_value);
      break;
  
    case NOTIFY_GET_SLIDER:
      NotifyGetSlider(page_id_lcd,control_id,_status,key_type,key_value);
      break;
      
    default:
      #if MEGA2560_TRACE_ERROR
        debugMessage = String("ProcessMessage()")
        + String("\t") + String(cmd_type, HEX) 
        + String("\t") + String(page_id_lcd) 
        + String("\t") + String(control_id) 
        + String("\t") + String(_status, HEX) 
        + String("\t") + String(key_type, HEX) 
        + String("\t") + String(key_value, HEX);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      break;
  }
}

/**
 * @brief Update LCD touch screen
 * 
 */
void UpdateUI()
{
  #if MEGA2560_TRACE_TEST
    debugMessage = String("UpdateUI()=") + String(page_id_lcd);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
  if (page_id_lcd == MAIN_PAGE)
  {
    #if MEGA2560_TRACE_TEST
      debugMessage = String("UpdateUI().MAIN_PAGE") + String(page_id_lcd);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    uint8_t newStrSatName[25] = "MI NUEVO SAT ;-))";
    strcpy((char *)strSatName, (char *) newStrSatName);
    TFTlcd.SetLableValue(page_id_lcd,MAIN_LABEL_SAT_NAME,strSatName);      
    ///TFTlcd.SetLableValue(page_id_lcd,MAIN_LABEL_SAT_NAME,(unsigned char *)strSatName);

    // Get GPS time and write variables
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_HH,(uint16_t)hour);
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_MM,(uint16_t)minute);
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_SS,(uint16_t)second);
    #if MEGA2560_TRACE_TEST
      debugMessage = String("newStrSatName=") + String((char *)newStrSatName);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
  }
  else if (page_id_lcd == ATTITUDE_PAGE)
  {
    #if MEGA2560_TRACE_INFO // For video recording purposes
      satAzimuth = 73;
      satElevation = 250;
      kalAngleY = kalAngleY_array[nextN++];
      horizontalHeadingAngle = horizontalHeadingAngle_array[nextN++];
      if (nextN > 550) nextN = 0;
    #endif
    // LCD local variables
    int16_t lcdSatelliteAzimut = (int16_t) satAzimuth; // degrees

    int16_t lcdAntennaAzimut = (int16_t) ((M_PI_2 - kalAngleY) * RAD_TO_DEG);
    lcdAntennaAzimut = positiveAngleDegrees(lcdAntennaAzimut);
    
    int16_t lcdSatelliteAscension = (int16_t) satElevation; // degrees
    lcdSatelliteAscension = positiveAngleDegrees(lcdSatelliteAscension);

    ///int16_t lcdAntennaAscension = (int16_t) ((horizontalHeadingAngle) * RAD_TO_DEG);
    //// int16_t lcdAntennaAscension = (int16_t) ((-horizontalHeadingAngle) * RAD_TO_DEG);
    int16_t lcdAntennaAscension = (int16_t) ((M_PI_2 - horizontalHeadingAngle) * RAD_TO_DEG);
    lcdAntennaAscension = positiveAngleDegrees(lcdAntennaAscension);

    // Set numbers
    // Info:
    //  The member function SetNumberValue(...) writes only positive numbers.
    //  The new member function SetLabelValueInt(...) writes positive and negative numbers,
    //  but overlaps other fields.
    // So, first I will paint the absolute value number and secondly its sign.

    // Size and color of anegative sign
    const uint16_t ATT_SIGN_WIDTH = 10;
    const uint16_t ATT_SIGN_HEIGTH = 3;
    // Paint number
    // TFTlcd.SetValueInt(page_id_lcd,74,44,lcdSatelliteAzimut);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_AZIMUT_SATELLITE,abs(lcdSatelliteAzimut));
    // Paint sign
    const uint16_t ATT_SIGN_AZIMUT_SATELLITE_X = 18;
    const uint16_t ATT_SIGN_AZIMUT_SATELLITE_Y = 53;
    uint16_t thisSign = (lcdSatelliteAzimut < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_AZIMUT_SATELLITE_X,ATT_SIGN_AZIMUT_SATELLITE_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    // TFTlcd.SetValueInt(page_id_lcd,75,45,lcdAntennaAzimut);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_AZIMUT_ANTENNA,abs(lcdAntennaAzimut));
    // Paint sign
    const uint16_t ATT_SIGN_AZIMUT_ANTENNA_X = 18;
    const uint16_t ATT_SIGN_AZIMUT_ANTENNA_Y = 90;
    thisSign = (lcdAntennaAzimut < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_AZIMUT_ANTENNA_X,ATT_SIGN_AZIMUT_ANTENNA_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    // TFTlcd.SetValueInt(page_id_lcd,76,46,lcdSatelliteAscension);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ASCENSION_SATELLITE,abs(lcdSatelliteAscension));
    // Paint sign
    const uint16_t ATT_SIGN_ASCENSION_SATELLITE_X = 200;
    const uint16_t ATT_SIGN_ASCENSION_SATELLITE_Y = 53;
    thisSign = (lcdSatelliteAscension < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_ASCENSION_SATELLITE_X,ATT_SIGN_ASCENSION_SATELLITE_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    //// TFTlcd.SetLabelValueInt(page_id_lcd,ATT_NUMBER_ASCENSION_ANTENNA,lcdAntennaAscension);
    // Set numbers
    // TFTlcd.SetValueInt(page_id_lcd,77,47,lcdAntennaAscension);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ASCENSION_ANTENNA,abs(lcdAntennaAscension));
    // lcdAntennaAscension is a positive value yet
    // TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ASCENSION_ANTENNA,lcdAntennaAscension);
    const uint16_t ATT_SIGN_ASCENSION_ANTENNA_X = 200;
    const uint16_t ATT_SIGN_ASCENSION_ANTENNA_Y = 90;
    thisSign = (lcdAntennaAscension < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_ASCENSION_ANTENNA_X,ATT_SIGN_ASCENSION_ANTENNA_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Heading info
    ///int16_t lcdHeading = (int16_t) ((horizontalHeadingAngle + M_PI_2) * RAD_TO_DEG);
    int16_t lcdHeading = (int16_t) ((horizontalHeadingAngle) * RAD_TO_DEG);
    lcdHeading = positiveAngleDegrees(lcdHeading);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_HEADING,lcdHeading);

    // Set figures -----------------------------
    // Set compass
    // Set sattellite ascension red line.
    // TFTlcd circle angles is left handed, but ascension is right handed from East position.
    // TFTlcd heading suplementary angle = 360 - heading
    // TFTlcd East = (360 - heading) + 90 = 90 - heading
    // TFTlcd circle angle for satellite = East - lcdSatelliteAscension
    // int lcdCircleGaugeSatAscension = (90 - heading) - lcdSatelliteAscension;
    //// int16_t lcdCircleGaugeSatAscension = (int16_t) round((- horizontalHeadingAngle) * RAD_TO_DEG) - lcdSatelliteAscension;
    int16_t lcdCircleGaugeSatAscension = (int16_t) round((M_PI_2 - horizontalHeadingAngle) * RAD_TO_DEG) - lcdSatelliteAscension;
    lcdCircleGaugeSatAscension = positiveAngleDegrees(lcdCircleGaugeSatAscension);
    TFTlcd.SetCircleGaugeValue(page_id_lcd,ATT_COMPASS,(uint16_t) lcdCircleGaugeSatAscension);

    // Azimut bar graph constants
    const uint16_t ATT_AZIMUT_BAR_GRAPH_WIDTH = 36;
    const uint16_t ATT_AZIMUT_BAR_GRAPH_HEIGHT = 238;
    // Set satelite azimut
    const uint16_t ATT_AZIMUT_SATELLITE_BAR_GRAPH_X = 9;
    const uint16_t ATT_AZIMUT_SATELLITE_BAR_GRAPH_Y = 156;
    // Set satellite azimut
    if (lcdSatelliteAzimut < 0)
    {
      lcdSatelliteAzimut = 0;
    }
    else if (lcdSatelliteAzimut > 90)
    {
      lcdSatelliteAzimut = 90;
    }
    // Escale lcdSatelliteAzimut to BarGraph from 0 to 100
    lcdSatelliteAzimut =(int16_t) round((lcdSatelliteAzimut*10)/9); // * 100/90
    TFTlcd.SetBarGraph(ATT_AZIMUT_SATELLITE_BAR_GRAPH_X,
                      ATT_AZIMUT_SATELLITE_BAR_GRAPH_Y,
                      ATT_AZIMUT_BAR_GRAPH_WIDTH/2,
                      ATT_AZIMUT_BAR_GRAPH_HEIGHT,
                      lcdSatelliteAzimut,COLOR_RED);
    // Set antenna azimut
    const uint16_t ATT_AZIMUT_ANTENNA_BAR_GRAPH_X = 86;
    const uint16_t ATT_AZIMUT_ANTENNA_BAR_GRAPH_Y = 156;
    int16_t lcdAntennaAzimut180 =  angle_pm180Degrees(lcdAntennaAzimut);
    if (lcdAntennaAzimut180 < 0)
    {
      lcdAntennaAzimut180 = 0;
    }
    else if (lcdAntennaAzimut180 > 90)
    {
      lcdAntennaAzimut180 = 90;
    }
    // Escale lcdAntennaAzimut180 to BarGraph from 0 to 100
    lcdAntennaAzimut180 =(int16_t) round((lcdAntennaAzimut180*10)/9); // * 100/90
    // Set smart color
    uint16_t antennaBargraphColor = COLOR_BLUE;
    if (((lcdSatelliteAzimut - 1) <= lcdAntennaAzimut180) && (lcdAntennaAzimut180 <= (lcdSatelliteAzimut + 1)))
      antennaBargraphColor = COLOR_GREEN;
    TFTlcd.SetBarGraph(ATT_AZIMUT_ANTENNA_BAR_GRAPH_X,
                      ATT_AZIMUT_ANTENNA_BAR_GRAPH_Y,
                      ATT_AZIMUT_BAR_GRAPH_WIDTH,
                      ATT_AZIMUT_BAR_GRAPH_HEIGHT,
                      lcdAntennaAzimut180,antennaBargraphColor);
    
    // Set satellite name
    strcpy((char *)strSatName, (char *) psSatelliteTLE[0]);    
    TFTlcd.SetLableValue(page_id_lcd,ATT_LABEL_SAT_NAME,strSatName);

    // Get GPS time and write variables
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_HH,(uint16_t)hour);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_MM,(uint16_t)minute);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_SS,(uint16_t)second);

    // Set GPS signal ON-OFF
    const uint16_t ATT_GPS_ON_X = 153;
    const uint16_t ATT_GPS_ON_Y = 388;
    const uint16_t ATT_GPS_WIDTH = 17;
    const uint16_t ATT_GPS_HEIGTH = 17;
    uint16_t gpsRectangleColor = (uGpsReceiving) ? COLOR_DARKGREEN : COLOR_GREY;
    TFTlcd.RectangleFill(ATT_GPS_ON_X,ATT_GPS_ON_Y,ATT_GPS_WIDTH,ATT_GPS_HEIGTH,gpsRectangleColor);
  }
  else  if(page_id_lcd == SATELLITES_PAGE)
  {
    // TO-DO
    // Show list of satellites

    #if MEGA2560_TRACE_TEST
      debugMessage = String("UpdateUI().SATELLITES_PAGE") + String(page_id_lcd);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    uint8_t newStrPagenumber[6] = "01/33";
    strcpy((char *)strPagenumber, (char *) newStrPagenumber);
    TFTlcd.SetLableValue(page_id_lcd,SAT_LABEL_PAGE_NUMB,strPagenumber);

    checkSat01 = 0;
    TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_01,checkSat01);
    checkSat02 = 1;
    TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_02,checkSat02);

    TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_03,1);
    ///strcpy(checkSat01, "Satelite num 01");
    //TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_01,(unsigned char *)checkSat01);
    ///strcpy(checkSat02, "Satelite num 02");
    //TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_02,(unsigned char *)checkSat02);
    ///strcpy(checkSat03, "Peperrrrllllll 02");
    //TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_03,(unsigned char *)checkSat03);
  }
}

void NotifyTouchButton(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(type == CHANGE_PAGE&& state == KEY_RELEASE)
  {
    page_id_lcd = value;
    update_lcd = 1;
    //UpdateUI();
  }
  else if(type == ENTER&& state == KEY_RELEASE)
  {
    targe_Id = value;//targe Edit Id
    TFTlcd.GetEditValue(page_id_lcd,targe_Id);
  }
  else if(type == CHAR)
  {
  }
  else if(type == UPLOAD_CONTROL_ID && state == KEY_RELEASE)
  {
    // BEGIN imjaviperez
    if (control_id == SAT_BUTTON_OK)
    {
      // TO-DO: Save selected satellite name
      page_id_lcd = MAIN_PAGE;
      TFTlcd.SetPage(MAIN_PAGE);
      update_lcd = 1;
    }  
    else if(control_id == SAT_BUTTON_UP)
    {
      // TO-DO: Show previous page
      TFTlcd.Display_Message(0X18,2,(unsigned char *)strMsgButtonUp);
      update_lcd = 1;
    }
    else if(control_id == SAT_BUTTON_DOWN)
    {
      // TO-DO: Show next page
      TFTlcd.Display_Message(0X18,2,(unsigned char *)strMsgButtonDown);
      update_lcd = 1;
    }
    // END imjaviperez
  }
  else if(type == CLEAR)
  {
  }

}

void NotifyTouchCheckbox(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:

  // BEGIN imjaviperez
  // TFTlcd.SetCheckboxValue(page_id_lcd,control_id, 0);
  // BEGIN imjaviperez


  if(state == SELECT)
    update_lcd = 1;

  //UpdateUI();
}

void NotifyTouchSlider(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  //if(update_lcd != 1)
  //  TFTlcd.SetNumberValue(page_id_lcd,28,(uint16_t)value);
  //UpdateUI();
}

void NotifyTouchEdit(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(update_lcd != 1)
    TFTlcd.GetTouchEditValue(page_id_lcd,control_id);
  
}

void NotifyGetEdit(PEDIT_MSG msg)
{
  uint8_t cmd_type    = msg->cmd_type;  //command
  uint8_t control_id  = msg->control_id;//object Id
  uint8_t page_id     = msg->page_id;   //page Id
  uint8_t _status     = msg->status;


  //The test passward number 1 2 3 4,ASCII code is 0x31 0x32 0x33 0x34
  if(msg->param[0] == 0x31 && msg->param[1] == 0x32 && msg->param[2] == 0x33 && msg->param[3] == 0x34)
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String01);  
  }
  else
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String02);  
  }
 
}

void NotifyGetTouchEdit(PEDIT_MSG msg)
{
  uint8_t cmd_type    = msg->cmd_type;  //command
  uint8_t control_id  = msg->control_id;//object Id
  uint8_t page_id     = msg->page_id;   //page Id
  uint8_t _status     = msg->status;


  //The test passward number 1 2 3 4,ASCII code is 0x31 0x32 0x33 0x34
  if(msg->param[0] == 0x31 && msg->param[1] == 0x32 && msg->param[2] == 0x33 && msg->param[3] == 0x34)
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String04);  
  }
  else
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String05);  
  }
 
}

void NotifyGetPage(uint8_t page_id,uint8_t status)
{
  //TODO:
  if(status == SUCCESS)
    page_id_lcd = page_id;
}


void NotifyGetCheckbox(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(state == SELECT)
  {
    update_lcd = 1;
  }
  //UpdateUI();
}

void NotifyGetSlider(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(state == SUCCESS)
  {
    //success get value
  }
  update_lcd = 1;
}

float getPositiveAngle(float angle)
{
  while (angle < 0)
  {
    angle += (float) 2*M_PI;
  }
  while (angle > 2*M_PI)
  {
    angle -= (float) 2*M_PI;
  }
  return angle;
}

