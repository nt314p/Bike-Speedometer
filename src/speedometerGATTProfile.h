#ifndef SPEEDOMETER_GATT_PROFILE_H
#define SPEEDOMETER_GATT_PROFILE_H

#include "hal_types.h"
#include "bcomdef.h"

#define SPEEDOMETER_PROFILE_SERVICE 0x00000001
#define SPEEDOMETER_PROFILE_SERVICE_UUID 0xFFF0
#define SPEEDOMETER_PROFILE_CHAR_BIKE_DATA_UUID 0xFFF1

#define SPEEDOMETER_PROFILE_CHAR_BIKE_DATA 0
#define BIKE_DATA_LEN 5

#define SERVAPP_NUM_ATTR_SUPPORTED 5

// Callback when a characteristic value has changed
typedef void (*speedometerProfileChange_t)(uint8 paramID);

typedef struct
{
    // Called when characteristic value changes
    speedometerProfileChange_t pfnSpeedometerProfileChange;
} speedometerProfileCBs_t;

bStatus_t SpeedometerProfile_AddService(uint32 services);
bStatus_t SpeedometerProfile_RegisterAppCBs(speedometerProfileCBs_t* appCallbacks);
bStatus_t SpeedometerProfile_SetParameter(uint8 param, uint8 len, void* value);
bStatus_t SpeedometerProfile_GetParameter(uint8 param, void* value);

#endif