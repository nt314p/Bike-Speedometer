#include "speedometer.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "OnBoard.h"
#include "gap.h"
#include "peripheral.h"
#include "gatt.h"
#include "gatt_profile_uuid.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "gapbondmgr.h"
#include "battservice.h"
#include "hal_types.h"
#include "hal_uart.h"

#define DEVICE_INIT_EVENT 1 << 0
#define PERIODIC_EVENT 1 << 1

#define DEVICE_NAME_LEN 16
static uint8 deviceName[DEVICE_NAME_LEN] = "Bike Speedometer";

uint8 speedometerTaskId;

// https://jimmywongiot.com/2019/08/13/advertising-payload-format-on-ble/
// https://www.ti.com/lit/ug/swru271i/swru271i.pdf
// https://argenox.com/library/bluetooth-low-energy/ble-advertising-primer

// Central devices can request more information from an advertising device
// by issuing a scan request. The advertising device responds with a scan
// response. In this case, we respond with the device name.
// Should disable to avoid turning on RX radio to listen for scan requests
static uint8 scanResponse[] =
{
  17, // length of string (not including terminator) + 1
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
  'B', 'i', 'k', 'e', ' ', 'S', 'p', 'e', 'e', 'd', 'o', 'm', 'e', 't', 'e', 'r'
};

// https://www.bluetooth.com/wp-content/uploads/Files/Specification/Assigned_Numbers.html#bookmark49
#define GAP_APPEARE_GENERIC_CYCLING 0x0480

// Advertising devices send out advertising packets to Central devices.
// This packet contains flags, appearance, and service UUID data.
static uint8 advertisingData[] =
{
    // flags
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03,
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_GENERIC_HR_SENSOR),
    HI_UINT16(GAP_APPEARE_GENERIC_HR_SENSOR),

    // manufacturer's data
    0x08,
    GAP_ADTYPE_MANUFACTURER_SPECIFIC,
    0xFF, 0xFF, // Company id for testing

    // Distance (revolutions): 3 bytes little endian
    // Speed (ms / revolution): 2 bytes little endian
    0x01, 0x02, 0x03,
    0x04, 0x05
};

static gapRolesCBs_t speedometer_PeripheralCBs =
{
    NULL,  // Profile State Change Callbacks
    NULL  // When a valid RSSI is read from controller (not used by application)
};

void Speedometer_Init(uint8 task_id)
{
    speedometerTaskId = task_id;

    // HalUARTInit();
    // UartInit();


    // uint8 message[] = {'P', 'e', 'e', 'p', '\n'};
    // HalUARTWrite(HAL_UART_PORT_1, message, 5);

    // Setup GAP ()
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, 6); // Pause 6 seconds (?)

    // Setup GAP role peripheral broadcaster
    uint8 enableAdvertising = TRUE;
    uint16 advertOffTime = 0;

    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &enableAdvertising);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16), &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanResponse), scanResponse);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertisingData), advertisingData);

    uint8 enableUpdateRequest = TRUE;

    // The range of the connection interval
    uint16 minConnInterval = 80; // 10 ms
    uint16 maxConnInterval = 800; // 10 ms
    uint16 peripheralLatency = 0; // The number of connection events we can skip
    uint16 connTimeoutMultiplier = 1000; // Connection timeout in units of 10 ms

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8), &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &minConnInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &maxConnInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16), &peripheralLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16), &connTimeoutMultiplier);

    // https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-2-bluetooth-le-advertising/topic/advertising-types/
    // ADV_NONCONN_IND means that the device is not connectable and does not accept
    // scan requests.
    uint8 advertisingType = GAP_ADTYPE_ADV_NONCONN_IND;
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8), &advertisingType);

    // Set GATT server name attribute
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, DEVICE_NAME_LEN, deviceName);

    uint16 advInt = 160; // In units of 0.625 ms, 160 = 100 ms

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

    GGS_AddService(GATT_ALL_SERVICES);
    GATTServApp_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();

    osal_set_event(speedometerTaskId, DEVICE_INIT_EVENT);
}

uint8 counter = 0;

uint16 Speedometer_ProcessEvent(uint8 task_id, uint16 events)
{
    if (events & SYS_EVENT_MSG)
    {
        // We don't expect any messages

        return (events ^ SYS_EVENT_MSG);
    }

    if (events & DEVICE_INIT_EVENT)
    {

        // Called when device starts
        // puts("Device initialized\n", 19);
        GAPRole_StartDevice(&speedometer_PeripheralCBs);

        osal_start_reload_timer(speedometerTaskId, PERIODIC_EVENT, 1000);

        return (events ^ DEVICE_INIT_EVENT);
    }

    if (events & PERIODIC_EVENT)
    {
        counter++;
        advertisingData[11] = counter;
        GAP_UpdateAdvertisingData(speedometerTaskId, TRUE, sizeof(advertisingData), advertisingData);

        return (events ^ PERIODIC_EVENT);
    }

    return 0;
}