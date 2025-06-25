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
#include "hal_types.h"

#include "speedometerGATTProfile.h"

#define DEVICE_INIT_EVENT 1 << 0
#define PERIODIC_EVENT 1 << 1
#define DEVICE_TIMEOUT_EVENT 1 << 2
#define REED_TRIGGER_EVENT 1 << 3

#define DEVICE_NAME_LEN 16
static uint8 deviceName[DEVICE_NAME_LEN] = "Bike Speedometer";

#define REVOLUTION_TIME_THRESHOLD_MS 200
#define MAX_REVOLUTION_TIME 0xFFFF
#define TIMEOUT_THRESHOLD_MS 10000

#define BUTTON_BIT BV(6)
#define REED_SW_BIT BV(7)

static uint8 speedometerTaskId;

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
static void UpdateAdvertisingData(uint32 revolutions, uint16 msPerRevolution)
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

// GAP Bond Manager Callbacks
static gapBondCBs_t speedometer_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

static void SetupPins()
{
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
    P0IFG = (uint8)~(REED_SW_BIT | BUTTON_BIT); // Clear any pending interrupts for the pins

    P0INP = REED_SW_BIT | BUTTON_BIT; // Set P0.7 and P0.6 inputs to 3 state
}

void Speedometer_Init(uint8 task_id)
{
    speedometerTaskId = task_id;

    SetupPins();

    // Setup GAP role peripheral broadcaster
    uint8 enableAdvertising = TRUE;
    uint16 advertOffTime = 0;

    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &enableAdvertising);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16), &advertOffTime);

    //GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanResponse), scanResponse);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertisingData), advertisingData);

    // https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-2-bluetooth-le-advertising/topic/advertising-types/
    // ADV_NONCONN_IND means that the device is not connectable and does not accept
    // scan requests.
    uint8 advertisingType = GAP_ADTYPE_ADV_NONCONN_IND;
    advertisingType = GAP_ADTYPE_ADV_IND;
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8), &advertisingType);

    uint8 enableUpdateRequest = FALSE;

    // The range of the connection interval
    uint16 minConnInterval = 400; // 10 ms
    uint16 maxConnInterval = 400; // 10 ms
    uint16 peripheralLatency = 4; // The number of connection events we can skip
    uint16 connTimeoutMultiplier = 500; // Connection timeout in units of 10 ms

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8), &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &minConnInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &maxConnInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16), &peripheralLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16), &connTimeoutMultiplier);

    // Set GATT server name attribute
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, DEVICE_NAME_LEN, deviceName);

    // 5 second advertising interval (just needs to be above 1 second)
    uint16 advInt = 8000;

    advInt = 200;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

    // Setup the GAP Bond Manager
    uint32 passkey = 0;
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitmProtection = TRUE;
    uint8 ioCapabilities = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8 enableBonding = TRUE;
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8), &mitmProtection);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8), &ioCapabilities);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8), &enableBonding);

    // Send out advertisements for only 1 second
    // Effectively sends a single packet
    // uint16 advTimeout = 1;
    // GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 1);

    GGS_AddService(GATT_ALL_SERVICES);
    GATTServApp_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();

    SpeedometerProfile_AddService(GATT_ALL_SERVICES);  // Speedometer GATT Profile
    uint8 bikeData[BIKE_DATA_LEN] = { 0, 0, 0, 0, 0 };
    SpeedometerProfile_SetParameter(SPEEDOMETER_PROFILE_CHAR_BIKE_DATA, BIKE_DATA_LEN, bikeData);

    osal_set_event(speedometerTaskId, DEVICE_INIT_EVENT);
}

static uint32 GetSleepTimer()
{
    uint32 time = ST0; // ST0 must be read first to latch clock values
    time |= ((uint32)ST1) << 8;
    time |= ((uint32)ST2) << 16;

    return time;
}

static uint32 GetSleepTimerMs()
{
    return (GetSleepTimer() * 125U) / 4096U;
}

static uint32 prevRevolutionTimeMs = 0;
static uint32 revolutionCounter = 0;
static uint32 diffMs = 0;
static uint8 isPreviousTimeValid = FALSE;

uint16 Speedometer_ProcessEvent(uint8 task_id, uint16 events)
{
    if (events & SYS_EVENT_MSG)
    {
        // We don't expect any messages
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & DEVICE_INIT_EVENT)
    {
        revolutionCounter = 0;
        prevRevolutionTimeMs = 0;
        isPreviousTimeValid = FALSE;

        GAPRole_StartDevice(&speedometer_PeripheralCBs);

        GAPBondMgr_Register(&speedometer_BondMgrCBs);

        UpdateAdvertisingData(revolutionCounter, 0);
        GAP_UpdateAdvertisingData(speedometerTaskId, TRUE, sizeof(advertisingData), advertisingData);

        // Calling this will reset the timer countdown
        // This will be useful later to put the device into PM2
        // to count the sleep timer.

        // If this event triggers, the device has not been woken by an interrupt for 
        // 10 seconds, which likely indicates the bike has stopped. We should stop using
        // the sleep timer to compute velocity, so the device can enter PM3.
        osal_start_timerEx(speedometerTaskId, DEVICE_TIMEOUT_EVENT, TIMEOUT_THRESHOLD_MS);

        osal_start_reload_timer(speedometerTaskId, PERIODIC_EVENT, 100);

        return (events ^ DEVICE_INIT_EVENT);
    }

    if (events & PERIODIC_EVENT)
    {
        // uint8 enableAdvertising = TRUE;
        // GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &enableAdvertising);

        return (events ^ PERIODIC_EVENT);
    }

    if (events & DEVICE_TIMEOUT_EVENT)
    {
        isPreviousTimeValid = FALSE;

        UpdateAdvertisingData(revolutionCounter, 0);
        GAP_UpdateAdvertisingData(speedometerTaskId, TRUE, sizeof(advertisingData), advertisingData);

        return (events ^ DEVICE_TIMEOUT_EVENT);
    }

    if (events & REED_TRIGGER_EVENT)
    {
        // Reset the timeout timer
        osal_start_timerEx(speedometerTaskId, DEVICE_TIMEOUT_EVENT, TIMEOUT_THRESHOLD_MS);
        revolutionCounter++;

        if (!isPreviousTimeValid) diffMs = 0;
        if (diffMs > MAX_REVOLUTION_TIME) diffMs = MAX_REVOLUTION_TIME;

        isPreviousTimeValid = TRUE;

        UpdateAdvertisingData(revolutionCounter, diffMs);
        GAP_UpdateAdvertisingData(speedometerTaskId, TRUE, sizeof(advertisingData), advertisingData);

        uint8 bikeData[BIKE_DATA_LEN] = { 0 };

        bikeData[0] = revolutionCounter & 0xFF;
        bikeData[1] = (revolutionCounter >> 8) & 0xFF;
        bikeData[2] = (revolutionCounter >> 16) & 0xFF;
        bikeData[3] = LO_UINT16(diffMs);
        bikeData[4] = HI_UINT16(diffMs);

        bStatus_t status = SpeedometerProfile_SetParameter(SPEEDOMETER_PROFILE_CHAR_BIKE_DATA, BIKE_DATA_LEN, bikeData);

        return (events ^ REED_TRIGGER_EVENT);
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

        while (!(SLEEPSTA & 1)); // Wait for positive clock

        uint32 currMs = GetSleepTimerMs();
        diffMs = (currMs - prevRevolutionTimeMs) & 0xFFFFFF;

        // Filter out impossibly short times < 200 ms
        if (diffMs >= REVOLUTION_TIME_THRESHOLD_MS)
        {
            prevRevolutionTimeMs = currMs;
            osal_set_event(speedometerTaskId, REED_TRIGGER_EVENT);
        }
    }

    if (P0IFG & BUTTON_BIT)
    {
        P0IFG &= ~BUTTON_BIT; // Clear interrupt
        // TODO: begin checking if button is held long enough for distance reset
    }

    P0IFG = 0; // Clear remaining interrupts
    P0IF = 0; // Clear CPU interrupt flag

    HAL_EXIT_ISR();
}