#include "STDebug.h"
//// #include "STSerialCommands.h"
//// #include "STWriteSerial.h"

#include <Arduino.h>

using namespace SatTrack;


STDebug::STDebug() : STSerialMsg(STSerialMsg::PORT_ZERO) 
{
    // init();
};

STDebug::~STDebug()
{
};


/**
 * @author Javi Perez
 * @brief Print information in the Serial Line.
 * @version 1.0.1
 * 
 * @param typeOfInfo numeric code showing the kind of info. Please @see STDebug.h.
 * @param hwOrigin numeric code showing the hardware origin of this info. Please @see STDebug.h.
 * @param degubInfo String pointer describing information.  Please @see STDebug.h.
*/
void STDebug::printDebug(const byte typeOfInfo, const byte hwOrigin, String &degubInfo)
{
    /*
    // Write the msg to the Serial Port. It will be sent to Serial port to be show by user
    String msg = String(STSerialMsg::MSG_HEADER_BEGIN) + String(STSerialMsg::DBG_MSG) +
            String(typeOfInfo) + 
            String("|") + String(hwOrigin) + 
            String("|") + degubInfo +
            String(STSerialMsg::MSG_TAIL_END);

    writeSerialMsg(msg);
    */
    printTrace(typeOfInfo, hwOrigin, degubInfo);
}

