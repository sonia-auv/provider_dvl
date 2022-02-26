
// Pris de https://stackoverflow.com/questions/11031062/c-preprocessor-avoid-code-repetition-of-member-variable-list/11744832#11744832

#ifndef PROVIDER_DVL_DVL_DATA_H
#define PROVIDER_DVL_DVL_DATA_H

#include "Reflectable.h"

struct DVLformat21_t
{
    //*****************
    // a delete
    DVLformat21_t(int dummy) 
    {
      pathfinderDataId = 0x76;
    }
    DVLformat21_t()
    : DVLformat21_t(9)
    {}
    //******************
private:
    REFLECTABLE
    (
        (uint8_t) pathfinderDataId,
        (uint8_t) dataStructure,
        (uint16_t) numberBytes,
        (uint8_t) systemConfig,
        (int16_t) xVelBtm,
        (uint16_t) yVelBtm,
        (uint16_t) zVelBtm,
        (uint16_t) eVelBtm,
        (uint16_t) rngToBottomBm1,
        (uint16_t) rngToBottomBm2,
        (uint16_t) rngToBottomBm3,
        (uint16_t) rngToBottomBm4,
        (uint8_t) bottomStatus,
        (uint16_t) velocity1,
        (uint16_t) velocity2,
        (uint16_t) velocity3,
        (uint16_t) velocity4,
        (uint16_t) refLayerStart,
        (uint16_t) refLayerEnd,
        (uint8_t) refLayerStatus,
        (uint8_t) hourFirstPing,
        (uint8_t) minuteFirstPing,
        (uint8_t) secondFirstPing,
        (uint8_t) hundredthFirstPing,
        (uint16_t) statusLeakSensors,
        (uint16_t) speedOfSound,
        (uint16_t) temperature
    )
};

#endif //PROVIDER_DVL_DVL_DATA_H