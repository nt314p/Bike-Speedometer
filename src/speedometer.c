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
#include "hal_sleep.h"

#define DEVICE_INIT_EVENT 1 << 0
#define PERIODIC_EVENT 1 << 1
#define DEVICE_TIMEOUT_EVENT 1 << 2

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

#define APP_DATA_INDEX 11

// Advertising devices send out advertising packets to Central devices.
// This packet contains flags, appearance, and service UUID data.
static uint8 advertisingData[] =
{
    // flags
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

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

// Updates the advertising data in memory ONLY
// Does not update the BLE stack advertisement
void UpdateAdvertisingData(uint32 revolutions, uint16 msPerRevolution)
{
    advertisingData[APP_DATA_INDEX] = revolutions & 0xFF;
    advertisingData[APP_DATA_INDEX + 1] = (revolutions >> 8) & 0xFF;
    advertisingData[APP_DATA_INDEX + 2] = (revolutions >> 16) & 0xFF;
    advertisingData[APP_DATA_INDEX + 3] = LO_UINT16(msPerRevolution);
    advertisingData[APP_DATA_INDEX + 4] = HI_UINT16(msPerRevolution);
}

static gapRolesCBs_t speedometer_PeripheralCBs =
{
    NULL,  // Profile State Change Callbacks
    NULL  // When a valid RSSI is read from controller (not used by application)
};

#define BUTTON_BIT BV(6)
#define REED_SW_BIT BV(7)

void Speedometer_Init(uint8 task_id)
{
    speedometerTaskId = task_id;

    // https://www.ti.com/lit/an/swra347a/swra347a.pdf
    // Setup pins for low power
    P0SEL = 0;
    P1SEL = 0;
    P2SEL = 0;

    P0DIR = 0x3F; // Set P0.6 and P0.7 as input
    P1DIR = 0xFF;
    P2DIR = 0x1F;

    P0 = 0;
    P1 = 0;
    P2 = 0;

    // Enable interrupts for P0.6 and P0.7
    // See hal_key.c for more details
    //PICTL &= ~BV(0);
    //PICTL |= BV(0); // Falling edge for all Port 0 pins give interrupt

    P0IEN |= REED_SW_BIT | BUTTON_BIT; // Enable interrupts for P0.7 and P0.6
    IEN1 |= BV(5); // Enable Port 0 interrupts
    P0IFG = ~(REED_SW_BIT | BUTTON_BIT); // Clear any pending interrupts for the pins

    P0INP = REED_SW_BIT | BUTTON_BIT; // Set P0.7 and P0.6 inputs to 3 state

    // Setup GAP role peripheral broadcaster
    uint8 enableAdvertising = TRUE;
    uint16 advertOffTime = 0;

    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &enableAdvertising);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16), &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanResponse), scanResponse);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertisingData), advertisingData);

    // https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-2-bluetooth-le-advertising/topic/advertising-types/
    // ADV_NONCONN_IND means that the device is not connectable and does not accept
    // scan requests.
    uint8 advertisingType = GAP_ADTYPE_ADV_NONCONN_IND;
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8), &advertisingType);

    // Set GATT server name attribute
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, DEVICE_NAME_LEN, deviceName);

    // 5 second advertising interval (just needs to be above 1 second)
    uint16 advInt = 8000;
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

    // Send out advertisements for only 1 second
    // Effectively sends a single packet
    uint16 advTimeout = 1;
    GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 1);

    GGS_AddService(GATT_ALL_SERVICES);
    GATTServApp_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();

    osal_set_event(speedometerTaskId, DEVICE_INIT_EVENT);
}

uint8 counter = 0;
uint32 previousSleepTimer = 0;

uint8 buttonClicks = 0;
uint8 reedClicks = 0;

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

        // Calling this will reset the timer countdown
        // This will be useful later to put the device into PM2
        // to count the sleep timer.

        // If this event triggers, the device has not been woken by an interrupt for 
        // 30 seconds, which likely indicates the bike has stopped. We should stop using
        // the sleep timer to compute velocity, so the device can enter PM3.
        osal_start_timerEx(speedometerTaskId, DEVICE_TIMEOUT_EVENT, 30 * 1000);

        osal_start_reload_timer(speedometerTaskId, PERIODIC_EVENT, 2000);

        return (events ^ DEVICE_INIT_EVENT);
    }

    if (events & PERIODIC_EVENT)
    {
        //counter++;
        //advertisingData[11] = counter;
        GAP_UpdateAdvertisingData(speedometerTaskId, TRUE, sizeof(advertisingData), advertisingData);

        uint8 enableAdvertising = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &enableAdvertising);

        return (events ^ PERIODIC_EVENT);
    }

    if (events & DEVICE_TIMEOUT_EVENT)
    {
        // TODO

        return (events ^ DEVICE_TIMEOUT_EVENT);
    }

    return 0;
}

// P0 interrupt handler for any port 0 interrupts
HAL_ISR_FUNCTION(P0INT_ISR, P0INT_VECTOR)
{
    HAL_ENTER_ISR();

    if (P0IFG & REED_SW_BIT)
    {
        P0IFG &= ~REED_SW_BIT; // Clear interrupt
        reedClicks++;
        // TODO: send advertisement
    }

    if (P0IFG & BUTTON_BIT)
    {
        P0IFG &= ~BUTTON_BIT; // Clear interrupt
        buttonClicks++;
        // TODO: begin checking if button is held long enough for distance reset
    }

    UpdateAdvertisingData(reedClicks, buttonClicks);

    P0IFG = 0; // Clear remaining interrupts
    P0IF = 0; // Clear CPU interrupt flag

    HAL_EXIT_ISR();
}