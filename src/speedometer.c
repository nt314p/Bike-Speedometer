#include "speedometer.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "osal_snv.h"
#include "OnBoard.h"
#include "gap.h"
#include "peripheral.h"
#include "gatt.h"
#include "hci.h"
#include "gatt_profile_uuid.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "hal_types.h"

#include "speedometerGATTProfile.h"

// The symbols below are in a library header that cannot be parsed by vscode
// Defining them here allows intellisense to work
#ifdef VSCODE
extern volatile uint8 EA, P0, P1, P2, IEN1, P0IFG, P0IF;
#endif

#define DEVICE_INIT_EVENT 1 << 0
#define PERIODIC_EVENT 1 << 1
#define DEVICE_TIMEOUT_EVENT 1 << 2
#define SEND_DATA_EVENT 1 << 3
#define BUTTON_PRESS_EVENT 1 << 4

#define SNV_ID_REVOLUTION_COUNTER 0x80

#define DEVICE_NAME_LEN 16
static uint8 deviceName[DEVICE_NAME_LEN] = "Bike Speedometer";

#define REVOLUTION_DELTA_THRESHOLD_MS 180
#define MAX_REVOLUTION_DELTA_MS 0xFFFF
#define SEND_DELTA_THRESHOLD_MS 500
#define TIMEOUT_THRESHOLD_MS 10000

#define BUTTON_DEBOUNCE_MS 100
#define CONSECUTIVE_SPECIAL_PRESS_THRESHOLD 5

// Computes the difference between two 24 bit sleep timer tick values, considering
// wrap around. The time delta is returned in milliseconds.
#define NEW_OLD_24BIT_TIME_DELTA_MS(new, old) (((((new) - (old)) & 0xFFFFFF) * 125U) / 4096U)
#define TIMER_TICKS_TO_MS(ticks) (((ticks) * 125U) / 4096U)

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
// static uint8 scanResponse[] =
// {
//   17, // length of string (not including terminator) + 1
//   GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
//   'B', 'i', 'k', 'e', ' ', 'S', 'p', 'e', 'e', 'd', 'o', 'm', 'e', 't', 'e', 'r'
// };

// Advertising devices send out advertising packets to Central devices.
static uint8 advertisingData[] =
{
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

static uint8 bikeData[BIKE_DATA_LEN] = { 0, 0, 0, 0, 0 };

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

// Main distance and speed variables
static uint32 revolutionCounter = 0;
static uint32 revolutionDeltaMs = 0;

// Timer value when the last revolution was detected
static uint32 lastRevolutionTimeTicks = 0;
static uint32 lastSendTimeTicks = 0; // Timer value when data was last sent

static uint8 isPreviousTimeValid = FALSE;

// A special button press is a button press when the reed switch is also activated
static uint8 specialButtonPresses = 0;

// Timer value when the button was pressed last; used for debounce
static uint32 lastButtonPressTimeTicks = 0;

// Resets the revolution counter in RAM and NV memory
static void ResetRevolutionCounter()
{
    revolutionCounter = 0;
    osal_snv_write(SNV_ID_REVOLUTION_COUNTER, sizeof(revolutionCounter), &revolutionCounter);
}

static void UpdateBikeData(uint32 revolutions, uint16 msPerRevolution)
{
    bikeData[0] = revolutions & 0xFF;
    bikeData[1] = (revolutions >> 8) & 0xFF;
    bikeData[2] = (revolutions >> 16) & 0xFF;
    bikeData[3] = LO_UINT16(msPerRevolution);
    bikeData[4] = HI_UINT16(msPerRevolution);
}

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

    P0INP = REED_SW_BIT | BUTTON_BIT; // Set P0.7 and P0.6 inputs to 3 state

    // Enable interrupts for P0.6 and P0.7
    // See hal_key.c for more details
    PICTL = 0;
    //PICTL &= (uint8)(~BV(0)); // Rising edge for all Port 0 pins give interrupt
    //PICTL |= BV(0); // Falling edge for all Port 0 pins give interrupt

    P0IEN |= REED_SW_BIT | BUTTON_BIT; // Enable interrupts for P0.7 and P0.6
    IEN1 |= BV(5); // Enable Port 0 interrupts
    P0IFG = (uint8)~(REED_SW_BIT | BUTTON_BIT); // Clear any pending interrupts for the pins
}

void Speedometer_Init(uint8 task_id)
{
    speedometerTaskId = task_id;

    SetupPins();

    // Setup GAP role peripheral broadcaster
    uint8 enableAdvertising = TRUE;
    uint16 advertOffTime = 30000;

    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &enableAdvertising);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16), &advertOffTime);

    //GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanResponse), scanResponse);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertisingData), advertisingData);

    uint8 advertisingType = GAP_ADTYPE_ADV_IND;
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8), &advertisingType);

    uint8 enableUpdateRequest = TRUE;

    // The range of the connection interval
    uint16 minConnInterval = 120; // 150 ms
    uint16 maxConnInterval = 120; // 150 ms
    uint16 peripheralLatency = 20; // The number of connection events we can skip
    uint16 connTimeoutMultiplier = 1000; // Connection timeout in units of 10 ms

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8), &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &minConnInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &maxConnInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16), &peripheralLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16), &connTimeoutMultiplier);

    // Set GATT server name attribute
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, DEVICE_NAME_LEN, deviceName);

    uint16 advInt = 800; // 500 ms advertising interval

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

    HCI_EXT_ClkDivOnHaltCmd(HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT);
    HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_23_DBM);

    // Use peripheral latency
    HCI_EXT_SetSlaveLatencyOverrideCmd(HCI_EXT_ENABLE_SL_OVERRIDE);

    // Allow new TX data (like notifications) to override skipping
    // connection events until the maximum latency
    HCI_EXT_SetFastTxResponseTimeCmd(HCI_EXT_ENABLE_FAST_TX_RESP_TIME);

    GGS_AddService(GATT_ALL_SERVICES);
    GATTServApp_AddService(GATT_ALL_SERVICES);

    SpeedometerProfile_AddService(GATT_ALL_SERVICES);  // Speedometer GATT Profile
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

// Fetches the sleep timer value after waiting for a positive
// sleep timer clock transition, as specified in the docs.
static uint32 GetSleepTimerSafe()
{
    while (!(SLEEPSTA & 1));

    uint32 time = ST0;
    time |= ((uint32)ST1) << 8;
    time |= ((uint32)ST2) << 16;

    return time;
}

static uint32 GetSleepTimerMs()
{
    return (GetSleepTimer() * 125U) / 4096U;
}

uint16 Speedometer_ProcessEvent(uint8 task_id, uint16 events)
{
    if (events & SYS_EVENT_MSG)
    {
        return (events ^ SYS_EVENT_MSG); // We don't expect any messages
    }

    if (events & DEVICE_INIT_EVENT)
    {
        // Attempt to read saved revolution counter
        uint8 status = osal_snv_read(SNV_ID_REVOLUTION_COUNTER,
            sizeof(revolutionCounter), &revolutionCounter);

        if (status == NV_OPER_FAILED) // Item does not exist
        {
            ResetRevolutionCounter(); // This also initializes the counter
        }

        // TODO: should these ticks be initialized to the clock value?
        lastRevolutionTimeTicks = 0;
        lastSendTimeTicks = 0;
        lastButtonPressTimeTicks = 0;
        isPreviousTimeValid = FALSE;

        GAPRole_StartDevice(&speedometer_PeripheralCBs);
        GAPBondMgr_Register(&speedometer_BondMgrCBs);

        GAP_UpdateAdvertisingData(speedometerTaskId, TRUE, sizeof(advertisingData), advertisingData);

        osal_start_timerEx(speedometerTaskId, DEVICE_TIMEOUT_EVENT, TIMEOUT_THRESHOLD_MS);

        return (events ^ DEVICE_INIT_EVENT);
    }

    if (events & DEVICE_TIMEOUT_EVENT)
    {
        isPreviousTimeValid = FALSE;

        UpdateBikeData(revolutionCounter, 0);
        SpeedometerProfile_SetParameter(SPEEDOMETER_PROFILE_CHAR_BIKE_DATA, BIKE_DATA_LEN, bikeData);

        return (events ^ DEVICE_TIMEOUT_EVENT);
    }

    if (events & SEND_DATA_EVENT)
    {
        // Reset the timeout timer
        osal_start_timerEx(speedometerTaskId, DEVICE_TIMEOUT_EVENT, TIMEOUT_THRESHOLD_MS);

        if (!isPreviousTimeValid)
        {
            revolutionDeltaMs = 0; // We cannot compute a delta without valid previous time
        }

        if (revolutionDeltaMs > MAX_REVOLUTION_DELTA_MS)
        {
            revolutionDeltaMs = MAX_REVOLUTION_DELTA_MS;
        }

        UpdateBikeData(revolutionCounter, revolutionDeltaMs);
        SpeedometerProfile_SetParameter(SPEEDOMETER_PROFILE_CHAR_BIKE_DATA, BIKE_DATA_LEN, bikeData);

        return (events ^ SEND_DATA_EVENT);
    }

    if (events & BUTTON_PRESS_EVENT)
    {
        // TODO: If not connected, enable advertising

        if (REED_SW_BIT & P0)
        {
            specialButtonPresses++;

            if (specialButtonPresses >= CONSECUTIVE_SPECIAL_PRESS_THRESHOLD)
            {
                ResetRevolutionCounter();
                specialButtonPresses = 0;
            }
        } else
        {
            specialButtonPresses = 0;
        }

        return (events ^ BUTTON_PRESS_EVENT);
    }

    return 0;
}

void PeripheralStateChange(gaprole_States_t newState)
{

}

// P0 interrupt handler for any port 0 interrupts
// Library source file hal_key.c was modified to remove
// the P0 ISR, so it would not overwrite this one.
HAL_ISR_FUNCTION(P0INT_ISR, P0INT_VECTOR)
{
    HAL_ENTER_ISR();

    if (P0IFG & REED_SW_BIT)
    {
        P0IFG &= ~REED_SW_BIT; // Clear interrupt

        while (!(SLEEPSTA & 1)); // Wait for positive clock

        uint32 currentTicks = GetSleepTimer();
        revolutionDeltaMs = NEW_OLD_24BIT_TIME_DELTA_MS(currentTicks, lastRevolutionTimeTicks);

        // Filter out impossibly short times below some threshold
        if (revolutionDeltaMs >= REVOLUTION_DELTA_THRESHOLD_MS)
        {
            lastRevolutionTimeTicks = currentTicks;
            revolutionCounter++;

            isPreviousTimeValid = TRUE;

            uint32 sendDeltaMs = NEW_OLD_24BIT_TIME_DELTA_MS(currentTicks, lastSendTimeTicks);

            if (sendDeltaMs >= SEND_DELTA_THRESHOLD_MS)
            {
                lastSendTimeTicks = currentTicks;
                osal_set_event(speedometerTaskId, SEND_DATA_EVENT);
            }
        }
    }

    if (P0IFG & BUTTON_BIT)
    {
        P0IFG &= ~BUTTON_BIT; // Clear interrupt

        uint32 currentTicks = GetSleepTimerSafe();
        uint32 deltaMs = NEW_OLD_24BIT_TIME_DELTA_MS(currentTicks, lastButtonPressTimeTicks);

        if (deltaMs >= BUTTON_DEBOUNCE_MS)
        {
            lastButtonPressTimeTicks = currentTicks;
            osal_set_event(speedometerTaskId, BUTTON_PRESS_EVENT);
        }        
    }

    P0IFG = 0; // Clear remaining interrupts
    P0IF = 0; // Clear CPU interrupt flag

    HAL_EXIT_ISR();
}