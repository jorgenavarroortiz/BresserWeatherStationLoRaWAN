/*

Module:  BresserLoRaWAN.ino

Author:
    Jorge Navarro-Ortiz (jorgenavarro@ugr.es), University of Granada, 2023

Features:
  - Bresser 7-in-1 decoder
  - LoRaWAN with OTAA (Over The Air Activation)
  - Using RTC memory for maintaining data after deep sleep (NwkSKey, AppSKey, etc. so there is no need of a new LoRaWAN join after every reboot)
  - ESP32 WatchDog Timer (WDT) to avoid infinite hangs
  - Using Cayenne Low Power Payload (LLP) encoding to send weather station data
  - Check memory leaks (memory heap size after each reboot)

TO DO:
  - Check the time between messages (deep sleeps)... there is some deviation
  - OLED

*/

// ---------
// Libraries
// ---------
#include <esp_task_wdt.h>  // For ESP32 watch dog timer
#include "esp32/rom/rtc.h" // To get reset reason

// Watchdog timer
#define WDT_TIMEOUT 180
#define WDT_RESET 150

// For Cayenne Low Power Payload (LPP) encoding
#include <CayenneLPP.h>
#define CAYENNELPPSIZE 51

CayenneLPP lpp(CAYENNELPPSIZE);

// For the Bresser weather station
#include <Arduino.h>
#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"

// For LoRaWAN
#include <Arduino_LoRaWAN_network.h>
#include <Arduino_LoRaWAN_EventLog.h>
#include <arduino_lmic.h>

// ---------------------
// Data to be configured
// ---------------------
static const std::int32_t        getWeatherDataTimeout    = 60 * 1000;      // ms
static const std::int32_t intervalBetweenTransmissions    = 2 * 60 * 1000;  // ms (to calculate time for deep sleep)
static const std::int32_t         minimumDeepSleepTime    = 5 * 1000;       // ms
static const std::int32_t                 maxAwakeTime    = 30 * 60 * 1000; // ms
static const std::int32_t timeToSleepAfterMaxAwakeTime    = 30 * 60 * 1000; // ms
static const std::int32_t  noResetsToEraseLoRaWANState    = 10;             // After this number of resets, the LoRaWAN state will be reset
static const std::int32_t waitTimeForSensorInitialization = 2000;           // ms (for retries)

// deveui, little-endian
static const std::uint8_t deveui[] = { 0x2C, 0xB4, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

// appeui, little-endian
static const std::uint8_t appeui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// appkey: just a string of bytes, sometimes referred to as "big endian".
static const std::uint8_t appkey[] = { 0x18, 0x23, 0x50, 0xF9, 0xCC, 0xC3, 0x73, 0x5F, 0x96, 0xE2, 0xF2, 0x83, 0x1B, 0x71, 0x9F, 0x42 };

// ---------
// Constants
// ---------

#if defined(ARDUINO_heltec_wireless_stick)
// https://github.com/espressif/arduino-esp32/blob/master/variants/heltec_wireless_stick/pins_arduino.h
#define PIN_LMIC_NSS SS
#define PIN_LMIC_RST RST_LoRa
#define PIN_LMIC_DIO0 DIO0
#define PIN_LMIC_DIO1 DIO1
#define PIN_LMIC_DIO2 DIO2
#endif

// Uplink message payload size
// The maximum allowed for all data rates is 51 bytes.
const uint8_t PAYLOAD_SIZE = 51;

// RTC Memory Handling
#define MAGIC1 (('m' << 24) | ('g' < 16) | ('c' << 8) | '1')
#define MAGIC2 (('m' << 24) | ('g' < 16) | ('c' << 8) | '2')
#define EXTRA_INFO_MEM_SIZE 64

// The following variables are stored in the ESP32's RTC RAM -
// their value is retained after a Sleep Reset.
// JNa: RTC_DATA_ATTR not working, not preserving data on reboot after deep sleep
RTC_NOINIT_ATTR int32_t                        bootCount;
RTC_NOINIT_ATTR uint32_t                        magicFlag1;               //!< flag for validating Session State in RTC RAM 
RTC_NOINIT_ATTR Arduino_LoRaWAN::SessionState   rtcSavedSessionState;     //!< Session State saved in RTC RAM
RTC_NOINIT_ATTR uint32_t                        magicFlag2;               //!< flag for validating Session Info in RTC RAM 
RTC_NOINIT_ATTR Arduino_LoRaWAN::SessionInfo    rtcSavedSessionInfo;      //!< Session Info saved in RTC RAM
RTC_NOINIT_ATTR size_t                          rtcSavedNExtraInfo;       //!< size of extra Session Info data
RTC_NOINIT_ATTR uint8_t                         rtcSavedExtraInfo[EXTRA_INFO_MEM_SIZE]; //!< extra Session Info data

// ---------
// Variables
// ---------

// Sleep request, set in NetTxComplete()
bool sleepReq = false;

// Boot time and WDT reset time
std::int32_t tBoot;
std::uint32_t lastTimeWDTReset;

// Arduino IDE: Tools->Core Debug Level: "Debug|Verbose"
//#define CORE_DEBUG_LEVEL ARDUHAL_LOG_LEVEL_DEBUG
//#define CORE_DEBUG_LEVEL ARDUHAL_LOG_LEVEL_VERBOSE

#define DEBUG_PRINTF(...) \
  { Serial.printf(__VA_ARGS__); }

/****************************************************************************\
|
|	The LoRaWAN object
|
\****************************************************************************/

class cMyLoRaWAN : public Arduino_LoRaWAN_network {
public:
    cMyLoRaWAN() {};
    using Super = Arduino_LoRaWAN_network;
    void setup();
    void printSessionInfo(const SessionInfo &Info);
    void printSessionState(const SessionState &State);

protected:
    // you'll need to provide implementation for this.
    virtual bool GetOtaaProvisioningInfo(Arduino_LoRaWAN::OtaaProvisioningInfo*) override;
    virtual void NetTxComplete(void) override; // notify client that transmission has completed
    virtual void NetJoin(void) override;       // notify client has joined the network
    // Used to store/load data to/from persistent (at least during deep sleep) memory 
    virtual void NetSaveSessionInfo(const SessionInfo &Info, const uint8_t *pExtraInfo, size_t nExtraInfo) override;
    //virtual bool GetSavedSessionInfo(SessionInfo &Info, uint8_t*, size_t, size_t*) override;
    virtual void NetSaveSessionState(const SessionState &State) override;
    virtual bool NetGetSessionState(SessionState &State) override;
    virtual bool GetAbpProvisioningInfo(AbpProvisioningInfo *pAbpInfo) override;
};


/****************************************************************************\
|
|	The sensor object
|
\****************************************************************************/

class cSensor {
public:
    /// \brief the constructor. Deliberately does very little.
    cSensor() {};

    ///
    /// \brief set up the sensor object
    ///
    /// \param uplinkPeriodMs optional uplink interval. If not specified,
    ///         transmit every six minutes.
    ///
    void setup(std::uint32_t uplinkPeriodMs = 6 * 60 * 1000);

    ///
    /// \brief update sensor loop.
    ///
    /// \details
    ///     This should be called from the global loop(); it periodically
    ///     gathers and transmits sensor data.
    ///
    void loop();

private:
    void doUplink();

    bool m_fUplinkRequest;              // set true when uplink is requested
    bool m_fBusy;                       // set true while sending an uplink
    std::uint32_t m_uplinkPeriodMs;     // uplink period in milliseconds
    std::uint32_t m_tReference;         // time of last uplink

    // The weather station sensor
    bool getWeatherStationData(uint32_t timeout);
    void printWeatherStationData();
};

/****************************************************************************\
|
|	Globals
|
\****************************************************************************/

// Pin mapping
const cMyLoRaWAN::lmic_pinmap myPinMap = {
  .nss = PIN_LMIC_NSS,
  .rxtx = cMyLoRaWAN::lmic_pinmap::LMIC_UNUSED_PIN,
  .rst = PIN_LMIC_RST,
  .dio = { PIN_LMIC_DIO0, PIN_LMIC_DIO1, PIN_LMIC_DIO2 },
  .rxtx_rx_active = 0,
  .rssi_cal = 0,
  .spi_freq = 8000000,
  .pConfig = NULL
};

// the global LoRaWAN instance.
cMyLoRaWAN myLoRaWAN {};

// the global sensor instance
cSensor mySensor {};

// the weather station sensor
WeatherSensor weatherSensor;

// the global event log instance
Arduino_LoRaWAN::cEventLog myEventLog;


/****************************************************************************\
|
|	auxiliary functions()
|
\****************************************************************************/
/// Determine sleep duration and enter Deep Sleep Mode
void checkGoingToSleep(void) {
  std::int32_t timeFromBoot = millis() - tBoot; // ms

  if (sleepReq) {
      // Going to deep sleep requested after LoRaWAN transmission complete
      std::int32_t timeToSleep = intervalBetweenTransmissions - timeFromBoot; // ms
      if (timeToSleep < minimumDeepSleepTime) {
          timeToSleep = minimumDeepSleepTime;
        log_d("Requested going to deep sleep after LoRaWAN transmission complete! Sleep time is set to its minimum value (%d ms).", minimumDeepSleepTime);
      } else {
        log_d("Requested going to deep sleep after LoRaWAN transmission complete! Sleep time is %d ms.", timeToSleep);
      }
      log_d("LoRaWAN.Shutdown()");

      myLoRaWAN.Shutdown();
      delay(2000);
      ESP.deepSleep(timeToSleep * 1000); // usec
  }

  if (timeFromBoot > maxAwakeTime) {
      // Too much time awake!

      log_d("Max awake timer expired! (time from boot %d ms, max awake time %d ms)");
      log_d("LoRaWAN.Shutdown()");

      myLoRaWAN.Shutdown();
      delay(2000);
      ESP.deepSleep(timeToSleepAfterMaxAwakeTime * 1000);
  }
}


/****************************************************************************\
|
|	setup()
|
\**************   **************************************************************/

void setup() {
    tBoot = millis();
    lastTimeWDTReset = tBoot;

    // set baud rate
    Serial.begin(115200);

    // wait for serial to be ready
//    while (! Serial)
//        yield();

    // WatchDog Timer
    log_d("Configuring WDT (%d s)...", WDT_TIMEOUT);
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               //add current thread to WDT watch

    // JNa: tested empirically on a Heltec Wireless Stick board. It should be POWERON_RESET, but it is not.
    int resetReason = (int) rtc_get_reset_reason(0);
    if ((resetReason == RTCWDT_RTC_RESET) || (resetReason == POWERON_RESET))
    {
      // First time after powering on the board... RTCWDT_RTC_RESET=16, i.e. RTC Watch dog reset digital core and rtc module
      delay(2000); // So the following messages are shown in Arduino's serial monitor...
      log_d("Power on reset (reason %d)", resetReason);
	    bootCount = 0;
    } else if (resetReason == TG0WDT_SYS_RESET || resetReason == DEEPSLEEP_RESET || resetReason == SW_RESET) {
      log_d("Reset due to deep sleep (reason %d)", resetReason);
      bootCount++;
    } else {
      // Other reasons should be problematic...
      log_d("Other reason for reset (reason %d), erasing LoRaWAN state...", resetReason);
      bootCount++;
      magicFlag1 = 0;
      magicFlag2 = 0;
    }
    log_d("Boot count: %d", bootCount);

//     if ((bootCount % noResetsToEraseLoRaWANState) == 0) {
//       log_d("Erasing LoRaWAN state after %d resets...", noResetsToEraseLoRaWANState);
//       magicFlag1 = 0;
//       magicFlag2 = 0;
//#if defined(ARDUINO_heltec_wireless_stick)
//       // LoRa: reset chip
//       log_d("Resetting the LoRa chip after %d reboots (current reboot %d)...", noResetsToEraseLoRaWANState, bootCount);
//       LoRaChipReset();
//#endif
//     }

    // To check memory leaks
    uint32_t tmp =  ESP.getHeapSize();   // total heap size
    log_d("Heap size:         %d", tmp);
    tmp =  ESP.getFreeHeap();            // available heap
    log_d("Free heap:         %d", tmp); 
    tmp = ESP.getMinFreeHeap();          // lowest level of free heap since boot
    log_d("Min free heap:     %d", tmp); 
    tmp = ESP.getMaxAllocHeap();         // largest block of heap that can be allocated at once
    log_d("Max block of heap: %d", tmp); 

    // set up the log; do this fisrt.
    myEventLog.setup();

    // set up lorawan.
    LoRaChipReset();
    myLoRaWAN.setup();

    // similarly, set up the sensor.
    mySensor.setup(getWeatherDataTimeout);
}

/****************************************************************************\
|
|	loop()
|
\****************************************************************************/

void loop() {
    if (millis() - lastTimeWDTReset >= (1000*WDT_RESET)) {
      log_d("Resetting WDT (after %d s)...", WDT_RESET);
      esp_task_wdt_reset();
      lastTimeWDTReset = millis();
    }

    // the order of these is arbitrary, but you must poll them all.
    myLoRaWAN.loop();
    mySensor.loop();
    myEventLog.loop();

    checkGoingToSleep();
}

/****************************************************************************\
|
|	LoRaWAN methods
|
\****************************************************************************/

void LoRaChipReset(void) {
    log_d("Resetting LoRa chip...");
    pinMode(PIN_LMIC_RST, OUTPUT);
    digitalWrite(PIN_LMIC_RST, LOW);
    delay(100);
    digitalWrite(PIN_LMIC_RST, HIGH);
}

// our setup routine does the class setup and then registers an event handler so
// we can see some interesting things
void
cMyLoRaWAN::setup() {
    log_d("cMyLoRaWAN::setup()");
    // simply call begin() w/o parameters, and the LMIC's built-in
    // configuration for this board will be used.
    this->Super::begin(myPinMap);

    //LMIC_reset(); // JNa, testing

//    LMIC_selectSubBand(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    this->RegisterListener(
        // use a lambda so we're "inside" the cMyLoRaWAN from public/private perspective
        [](void *pClientInfo, uint32_t event) -> void {
            auto const pThis = (cMyLoRaWAN *)pClientInfo;

            // for tx start, we quickly capture the channel and the RPS
            if (event == EV_TXSTART) {
                // use another lambda to make log prints easy
                myEventLog.logEvent(
                    (void *) pThis,
                    LMIC.txChnl,
                    LMIC.rps,
                    0,
                    // the print-out function
                    [](cEventLog::EventNode_t const *pEvent) -> void {
#if CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
                        DEBUG_PRINTF(" TX:");
                        //log_d(" TX:");
#endif
                        myEventLog.printCh(std::uint8_t(pEvent->getData(0)));
                        myEventLog.printRps(rps_t(pEvent->getData(1)));
                    }
                );
            }
            // else if (event == some other), record with print-out function
            else {
                // do nothing.
            }
        },
        (void *) this   // in case we need it.
        );
}

// this method is called when the LMIC needs OTAA info.
// return false to indicate "no provisioning", otherwise
// fill in the data and return true.
bool
cMyLoRaWAN::GetOtaaProvisioningInfo(OtaaProvisioningInfo *pInfo) {
    // these are the same constants used in the LMIC compliance test script; eases testing
    // with the RedwoodComm RWC5020B/RWC5020M testers.
    log_v("cMyLoRaWAN::GetOtaaProvisioningInfo()");

    // initialize info
    memcpy(pInfo->DevEUI, deveui, sizeof(pInfo->DevEUI));
    memcpy(pInfo->AppEUI, appeui, sizeof(pInfo->AppEUI));
    memcpy(pInfo->AppKey, appkey, sizeof(pInfo->AppKey));

    return true;
}

// This method is called after the node has joined the network.
void
cMyLoRaWAN::NetJoin(void) {
    log_d("The node has joined the network!");
}

// This method is called after transmission has been completed.
// If enabled, the controller goes into deep sleep mode now.
void
cMyLoRaWAN::NetTxComplete(void) {
    sleepReq = true;
    log_d("Transmission complete, going to deep sleep...");
}

// Print session info for debugging
void 
cMyLoRaWAN::printSessionInfo(const SessionInfo &Info)
{
    log_v("cMyLoRaWAN::printSessionInfo():");
    log_v("Tag:\t\t%d", Info.V1.Tag);
    log_v("Size:\t\t%d", Info.V1.Size);
    log_v("Rsv2:\t\t%d", Info.V1.Rsv2);
    log_v("Rsv3:\t\t%d", Info.V1.Rsv3);
    log_v("NetID:\t\t0x%08X", Info.V1.NetID);
    log_v("DevAddr:\t\t0x%08X", Info.V1.DevAddr);
    if (CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG) {
        char buf[64];
        *buf = '\0';
        for (int i=0; i<15;i++) {
            sprintf(&buf[strlen(buf)], "%02X ", Info.V1.NwkSKey[i]);
        }
        log_v("NwkSKey:\t\t%s", buf);
    }
    if (CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG) {
        char buf[64];
        *buf = '\0';
        for (int i=0; i<15;i++) {
            sprintf(&buf[strlen(buf)], "%02X ", Info.V1.AppSKey[i]);  
        }
        log_v("AppSKey:\t\t%s", buf);
    }    
}

// Print session state for debugging
void 
cMyLoRaWAN::printSessionState(const SessionState &State)
{
    log_v("cMyLoRaWAN::printSessionState():");
    log_v("Tag:\t\t%d", State.V1.Tag);
    log_v("Size:\t\t%d", State.V1.Size);
    log_v("Region:\t\t%d", State.V1.Region);
    log_v("LinkDR:\t\t%d", State.V1.LinkDR);
    log_v("FCntUp:\t\t%d", State.V1.FCntUp);
    log_v("FCntDown:\t\t%d", State.V1.FCntDown);
    log_v("gpsTime:\t\t%d", State.V1.gpsTime);
    log_v("globalAvail:\t%d", State.V1.globalAvail);
    log_v("Rx2Frequency:\t%d", State.V1.Rx2Frequency);
    log_v("PingFrequency:\t%d", State.V1.PingFrequency);
    log_v("Country:\t\t%d", State.V1.Country);
    log_v("LinkIntegrity:\t%d", State.V1.LinkIntegrity);
    // There is more in it...
}


// Save Info to ESP32's RTC RAM
// if not possible, just do nothing and make sure you return false
// from NetGetSessionState().
void
cMyLoRaWAN::NetSaveSessionInfo(const SessionInfo &Info, const uint8_t *pExtraInfo, size_t nExtraInfo) {
    log_v("cMyLoRaWAN::NetSaveSessionInfo():");
    if (nExtraInfo > EXTRA_INFO_MEM_SIZE)
        return;
    rtcSavedSessionInfo = Info;
    rtcSavedNExtraInfo = nExtraInfo;
    memcpy(rtcSavedExtraInfo, pExtraInfo, nExtraInfo);
    magicFlag2 = MAGIC2;
    log_v("magicFlag2: %d", magicFlag2);
    printSessionInfo(Info);
}


// Save State in RTC RAM. Note that it's often the same;
// often only the frame counters change.
// [If not possible, just do nothing and make sure you return false
// from NetGetSessionState().]
void
cMyLoRaWAN::NetSaveSessionState(const SessionState &State) {
    log_v("cMyLoRaWAN::NetSaveSessionState():");  
    rtcSavedSessionState = State;
    magicFlag1 = MAGIC1;
    log_v("magicFlag1: %d", magicFlag1);
    printSessionState(State);
}

// Either fetch SessionState from somewhere and return true or...
// return false, which forces a re-join.
bool
cMyLoRaWAN::NetGetSessionState(SessionState &State) {
    log_v("cMyLoRaWAN::NetGetSessionState():");
    log_v("magicFlag1: %d, MAGIC1: %d", magicFlag1, MAGIC1);
    if (magicFlag1 == MAGIC1) {
        State = rtcSavedSessionState;
        log_v("o.k.");
        printSessionState(State);
        return true;
    } else {
        log_v("failed");
        return false;
    }
}

// Get APB provisioning info - this is also used in OTAA after a succesful join.
// If it can be provided in OTAA mode after a restart, no re-join is needed.
bool
cMyLoRaWAN::GetAbpProvisioningInfo(AbpProvisioningInfo *pAbpInfo) {
    log_v("cMyLoRaWAN::GetAbpProvisioningInfo():");
    SessionState state;

    // ApbInfo:
    // --------
    // uint8_t         NwkSKey[16];
    // uint8_t         AppSKey[16];
    // uint32_t        DevAddr;
    // uint32_t        NetID;
    // uint32_t        FCntUp;
    // uint32_t        FCntDown;
    
    log_v("magicFlag1: %d, MAGIC1: %d", magicFlag1, MAGIC1);
    log_v("magicFlag2: %d, MAGIC2: %d", magicFlag2, MAGIC2);
    if ((magicFlag1 != MAGIC1) || (magicFlag2 != MAGIC2)) {
         return false;
    }

    pAbpInfo->DevAddr = rtcSavedSessionInfo.V2.DevAddr;
    pAbpInfo->NetID   = rtcSavedSessionInfo.V2.NetID;
    memcpy(pAbpInfo->NwkSKey, rtcSavedSessionInfo.V2.NwkSKey, 16);
    memcpy(pAbpInfo->AppSKey, rtcSavedSessionInfo.V2.AppSKey, 16);
    NetGetSessionState(state);
    pAbpInfo->FCntUp   = state.V1.FCntUp;
    pAbpInfo->FCntDown = state.V1.FCntDown;

    if (CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG) {
        char buf[64];
        
        *buf = '\0';
        for (int i=0; i<15;i++) {
          sprintf(&buf[strlen(buf)], "%02X ", pAbpInfo->NwkSKey[i]);  
        }
        log_d("NwkSKey:\t%s", buf);
        
        *buf = '\0';
        for (int i=0; i<15;i++) {
          sprintf(&buf[strlen(buf)], "%02X ", pAbpInfo->AppSKey[i]);  
        }
        log_d("AppSKey:\t%s", buf);
        log_d("FCntUp:\t%d", state.V1.FCntUp);
    }
    return true;
}


/****************************************************************************\
|
|	Sensor methods
|
\****************************************************************************/

void
cSensor::setup(std::uint32_t uplinkPeriodMs) {
/*    if (!sensor.begin()) {
        while (true) {
            // just loop forever, printing an error occasionally.
            log_d("sensor.begin failed");
            delay(2000);
        }
    }
*/
    int16_t errorOnWeatherSensor = true;
    errorOnWeatherSensor = weatherSensor.begin();
    while (errorOnWeatherSensor != RADIOLIB_ERR_NONE) {
// #if defined(ARDUINO_heltec_wireless_stick)
//       // LoRa: reset chip
//       log_d("Error on weatherSensor.begin(), resetting the LoRa chip and retrying after %d ms...", waitTimeForSensorInitialization);
//       LoRaChipReset();
// #endif
      log_d("Error on weather sensor initialization (%d), trying again after %d seconds...", errorOnWeatherSensor, waitTimeForSensorInitialization);
      delay(waitTimeForSensorInitialization);
      errorOnWeatherSensor = weatherSensor.begin();
    }

    // set the initial time.
    this->m_uplinkPeriodMs = uplinkPeriodMs;
    this->m_tReference = millis();

    // uplink right away
    this->m_fUplinkRequest = true;
}

void
cSensor::loop(void) {
    auto const tNow = millis();
    auto const deltaT = tNow - this->m_tReference;

    if (deltaT >= this->m_uplinkPeriodMs) {
        // request an uplink
        this->m_fUplinkRequest = true;

        // keep trigger time locked to uplinkPeriod
        auto const advance = deltaT / this->m_uplinkPeriodMs;
        this->m_tReference += advance * this->m_uplinkPeriodMs; 
    }

    // if an uplink was requested, do it.
    if (this->m_fUplinkRequest) {
        this->m_fUplinkRequest = false;
        this->doUplink();
    }
}

// ------------------------------------
// Functions related to weather station
// ------------------------------------
bool
cSensor::getWeatherStationData(uint32_t timeout) {
    // Tries to receive radio message (non-blocking) and to decode it.
    // Timeout occurs after a small multiple of expected time-on-air.
    const uint32_t timestamp = millis();

    while ((millis() - timestamp) < timeout) {
      int decode_status = weatherSensor.getMessage();
      if (decode_status == DECODE_OK || decode_status == DECODE_PAR_ERR) {
          return true;
      } // if (decode_status == DECODE_OK || decode_status == DECODE_PAR_ERR) {
    } // while ((millis() - timestamp) < timeout)

    // Timeout
    return false;
}

void
cSensor::printWeatherStationData() {
    // This example uses only a single slot in the sensor data array
    int const i=0;

#if CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    DEBUG_PRINTF("Id: [%8X] Typ: [%X] Battery: [%s] ",
        weatherSensor.sensor[i].sensor_id,
        weatherSensor.sensor[i].s_type,
        weatherSensor.sensor[i].battery_ok ? "OK " : "Low");
    #ifdef BRESSER_6_IN_1
        DEBUG_PRINTF("Ch: [%d] ", weatherSensor.sensor[i].chan);
    #endif
    if (weatherSensor.sensor[i].temp_ok) {
        DEBUG_PRINTF("Temp: [%5.1fC] ",
            weatherSensor.sensor[i].temp_c);
    } else {
        DEBUG_PRINTF("Temp: [---.-C] ");
    }
    if (weatherSensor.sensor[i].humidity_ok) {
        DEBUG_PRINTF("Hum: [%3d%%] ",
            weatherSensor.sensor[i].humidity);
    } else {
        DEBUG_PRINTF("Hum: [---%%] ");
    }
    if (weatherSensor.sensor[i].wind_ok) {
        DEBUG_PRINTF("Wind max: [%4.1fm/s] Wind avg: [%4.1fm/s] Wind dir: [%5.1fdeg] ",
                weatherSensor.sensor[i].wind_gust_meter_sec,
                weatherSensor.sensor[i].wind_avg_meter_sec,
                weatherSensor.sensor[i].wind_direction_deg);
    } else {
        DEBUG_PRINTF("Wind max: [--.-m/s] Wind avg: [--.-m/s] Wind dir: [---.-deg] ");
    }
    if (weatherSensor.sensor[i].rain_ok) {
        DEBUG_PRINTF("Rain: [%7.1fmm] ",  
            weatherSensor.sensor[i].rain_mm);
    } else {
        DEBUG_PRINTF("Rain: [-----.-mm] "); 
    }
    if (weatherSensor.sensor[i].moisture_ok) {
        DEBUG_PRINTF("Moisture: [%2d%%] ",
            weatherSensor.sensor[i].moisture);
    } else {
        DEBUG_PRINTF("Moisture: [--%%] ");
    }
    #if defined BRESSER_6_IN_1 || defined BRESSER_7_IN_1
    if (weatherSensor.sensor[i].uv_ok) {
        DEBUG_PRINTF("UV index: [%1.1f] ",
            weatherSensor.sensor[i].uv);
    } else {
        DEBUG_PRINTF("UV index: [-.-%%] ");
    }
    #endif
    #ifdef BRESSER_7_IN_1
    if (weatherSensor.sensor[i].light_ok) {
        DEBUG_PRINTF("Light (Klux): [%2.1fKlux] ",
            weatherSensor.sensor[i].light_klx);
    } else {
        DEBUG_PRINTF("Light (lux): [--.-Klux] ");
    }
    #endif      
    DEBUG_PRINTF("RSSI: [%5.1fdBm]", weatherSensor.sensor[i].rssi);
#endif
}

// -------------------------------------------
// End of functions related to weather station
// -------------------------------------------

void
cSensor::doUplink(void) {
    // if busy uplinking, just skip
    if (this->m_fBusy) {
        log_d("cSensor::doUplink(): busy");
        return;
    }
    // if LMIC is busy, just skip
    if (LMIC.opmode & (OP_POLL | OP_TXDATA | OP_TXRXPEND)) {
        log_d("cSensor::doUplink(): other operation in progress");
        return;
    }

    // Send measurements from the Bresser weather station
      // Clear all sensor data
    weatherSensor.clearSlots();
      // Get weather data
    bool bWeatherData = getWeatherStationData(getWeatherDataTimeout);
    lpp.reset();
    lpp.addDigitalInput(0, (uint8_t) bWeatherData); // 0 = no data, 1 = there is valid weather data
    if (bWeatherData) {
        printWeatherStationData();
        log_d("Sending valid weather data");

        // Encode measurement
        int const i=0;
        lpp.addDigitalInput(1, weatherSensor.sensor[i].sensor_id);
        lpp.addDigitalInput(2, weatherSensor.sensor[i].s_type);
        lpp.addDigitalInput(3, weatherSensor.sensor[i].battery_ok);
        if (weatherSensor.sensor[i].temp_ok)
            lpp.addTemperature(4, weatherSensor.sensor[i].temp_c);
        if (weatherSensor.sensor[i].humidity_ok)
            lpp.addRelativeHumidity(5, weatherSensor.sensor[i].humidity);
        if (weatherSensor.sensor[i].wind_ok) {
            lpp.addAnalogInput(6, weatherSensor.sensor[i].wind_gust_meter_sec);
            lpp.addAnalogInput(7, weatherSensor.sensor[i].wind_avg_meter_sec);
            lpp.addAnalogInput(8, weatherSensor.sensor[i].wind_direction_deg);
        }
        if (weatherSensor.sensor[i].rain_ok)
            lpp.addAnalogInput(9, weatherSensor.sensor[i].rain_mm);
        if (weatherSensor.sensor[i].uv_ok)
            lpp.addAnalogInput(10, weatherSensor.sensor[i].uv);
        if (weatherSensor.sensor[i].light_ok)
            lpp.addAnalogInput(11, weatherSensor.sensor[i].light_klx);
        lpp.addAnalogInput(11, weatherSensor.sensor[i].rssi);
    } else {
      log_d("Sending invalid weather data indication");
    }

    this->m_fBusy = true;
    
    if (! myLoRaWAN.SendBuffer(
        lpp.getBuffer(), lpp.getSize(),
        // this is the completion function:
        [](void *pClientData, bool fSucccess) -> void {
            auto const pThis = (cSensor *)pClientData;
            pThis->m_fBusy = false;
        },
        (void *)this,
        /* confirmed */ false,
        /* port */ 1
        )) {
        // sending failed; callback has not been called and will not
        // be called. Reset busy flag.
        this->m_fBusy = false;
    }
}
