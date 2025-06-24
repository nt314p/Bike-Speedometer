#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "speedometerGATTProfile.h"

// Speedometer profile service UUID: 0xFFF0
CONST uint8 speedometerProfileServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(SPEEDOMETER_PROFILE_SERVICE_UUID), HI_UINT16(SPEEDOMETER_PROFILE_SERVICE_UUID)
};

// Bike data characteristic UUID: 0xFFF1
CONST uint8 speedometerProfileCharBikeDataUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(SPEEDOMETER_PROFILE_CHAR_BIKE_DATA_UUID), HI_UINT16(SPEEDOMETER_PROFILE_CHAR_BIKE_DATA_UUID)
};

static speedometerProfileCBs_t* speedometerProfile_AppCBs = NULL;

static CONST gattAttrType_t speedometerProfileService = { ATT_BT_UUID_SIZE, speedometerProfileServiceUUID };

static uint8 bikeData[BIKE_DATA_LEN] = { 0, 0, 0, 0, 0 };
static uint8 bikeDataCharProps = GATT_PROP_NOTIFY;
static uint8 bikeDataCharDescription[11] = "Bike data";
static gattCharCfg_t* bikeDataCharConfig;

static gattAttribute_t speedometerProfileAttrTable[SERVAPP_NUM_ATTR_SUPPORTED] =
{
    // Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, // type
        GATT_PERMIT_READ, // permissions
        0, // handle
        (uint8*)&speedometerProfileService // pValue
    },

    // Bike data characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &bikeDataCharProps
    },

    // Bike data value
    {
        { ATT_BT_UUID_SIZE, speedometerProfileCharBikeDataUUID },
        GATT_PERMIT_READ, // changed this from 0
        0,
        bikeData
    },

    // Bike data configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8*)&bikeDataCharConfig
    },

    // Bike data description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        bikeDataCharDescription
    },
};

static bStatus_t SpeedometerProfile_ReadAttrCB(uint16 connHandle, gattAttribute_t* pAttr,
    uint8* pValue, uint8* pLen, uint16 offset,
    uint8 maxLen, uint8 method);
static bStatus_t SpeedometerProfile_WriteAttrCB(uint16 connHandle, gattAttribute_t* pAttr,
    uint8* pValue, uint8 len, uint16 offset,
    uint8 method);

// Speedometer Profile Service Callbacks
CONST gattServiceCBs_t speedometerProfileCBs =
{
  SpeedometerProfile_ReadAttrCB,  // Read callback function pointer
  SpeedometerProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

bStatus_t SpeedometerProfile_AddService(uint32 services)
{
    uint8 status;

    // Allocate Client Characteristic Configuration table
    bikeDataCharConfig = (gattCharCfg_t*)osal_mem_alloc(sizeof(gattCharCfg_t) *
        linkDBNumConns);

    if (bikeDataCharConfig == NULL)
    {
        return bleMemAllocError;
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, bikeDataCharConfig);

    if (services & SPEEDOMETER_PROFILE_SERVICE)
    {
        // Register GATT attribute list and CBs with GATT Server App
        status = GATTServApp_RegisterService(speedometerProfileAttrTable,
            GATT_NUM_ATTRS(speedometerProfileAttrTable),
            GATT_MAX_ENCRYPT_KEY_SIZE,
            &speedometerProfileCBs);
    }
    else
    {
        status = SUCCESS;
    }

    return status;
}

bStatus_t SpeedometerProfile_RegisterAppCBs(speedometerProfileCBs_t* appCallbacks)
{
    if (appCallbacks)
    {
        speedometerProfile_AppCBs = appCallbacks;
        return SUCCESS;
    }

    return bleAlreadyInRequestedMode;
}

bStatus_t SpeedometerProfile_SetParameter(uint8 param, uint8 len, void* value)
{
    bStatus_t ret = SUCCESS;
    switch (param)
    {
    case SPEEDOMETER_PROFILE_CHAR_BIKE_DATA:
    {
        if (len == BIKE_DATA_LEN)
        {
            memcpy(bikeData, value, BIKE_DATA_LEN);

            // See if Notification has been enabled
            ret = GATTServApp_ProcessCharCfg(bikeDataCharConfig, bikeData, FALSE,
                speedometerProfileAttrTable, GATT_NUM_ATTRS(speedometerProfileAttrTable),
                INVALID_TASK_ID, SpeedometerProfile_ReadAttrCB);
        }
        else
        {
            ret = bleInvalidRange;
        }
    } break;
    default:
    {
        ret = INVALIDPARAMETER;
    } break;
    }

    return ret;
}

bStatus_t SpeedometerProfile_GetParameter(uint8 param, void* value)
{
    bStatus_t ret = SUCCESS;
    switch (param)
    {

    case SPEEDOMETER_PROFILE_CHAR_BIKE_DATA:
    {
        VOID memcpy(value, bikeData, BIKE_DATA_LEN);
    } break;

    default:
    {
        ret = INVALIDPARAMETER;
    } break;
    }

    return ret;
}

static bStatus_t SpeedometerProfile_ReadAttrCB(uint16 connHandle, gattAttribute_t* pAttr,
    uint8* pValue, uint8* pLen, uint16 offset,
    uint8 maxLen, uint8 method)
{
    bStatus_t status = SUCCESS;

    // If attribute permissions require authorization to read, return error
    if (gattPermitAuthorRead(pAttr->permissions))
    {
        // Insufficient authorization
        return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if (offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch (uuid)
        {
            // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
            // gattserverapp handles those reads

            // Bike characteristic has no read permission but can be sent as a 
            // notification
        case SPEEDOMETER_PROFILE_CHAR_BIKE_DATA:
        {
            *pLen = BIKE_DATA_LEN;
            memcpy(pValue, pAttr->pValue, BIKE_DATA_LEN);
        } break;
        default:
        {
            *pLen = 0;
            status = ATT_ERR_ATTR_NOT_FOUND;
        } break;
        }
    }
    else
    {
        // 128-bit UUID
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    return status;
}

static bStatus_t SpeedometerProfile_WriteAttrCB(uint16 connHandle, gattAttribute_t* pAttr,
    uint8* pValue, uint8 len, uint16 offset,
    uint8 method)
{
    bStatus_t status = SUCCESS;
    uint8 notifyApp = 0xFF;

    // If attribute permissions require authorization to write, return error
    if (gattPermitAuthorWrite(pAttr->permissions))
    {
        // Insufficient authorization
        return (ATT_ERR_INSUFFICIENT_AUTHOR);
    }

    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch (uuid)
        {
            // No characteristics have write permissions...?
        case GATT_CLIENT_CHAR_CFG_UUID:
        {
            status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                offset, GATT_CLIENT_CFG_NOTIFY);
        } break;
        default:
        {
            status = ATT_ERR_ATTR_NOT_FOUND;
        } break;
        }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    // If a charactersitic value changed then callback function to notify application of change
    if ((notifyApp != 0xFF) && speedometerProfile_AppCBs && speedometerProfile_AppCBs->pfnSpeedometerProfileChange)
    {
        speedometerProfile_AppCBs->pfnSpeedometerProfileChange(notifyApp);
    }

    return status;
}