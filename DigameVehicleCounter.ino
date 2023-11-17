#include <esp_task_wdt.h>
#define WDT_TIMEOUT 60 //60 seconds WDT

#include <ESPAsyncWebServer.h>

//---------------------------------------------------------------------------------------------
/* HEIMDALL VCS - Vehicle Counting System
   A traffic counting system that uses LIDAR to track pedestrians and vehicles.

   Copyright 2021-23, Digame Systems. All rights reserved.
*/
//---------------------------------------------------------------------------------------------

#define debugUART Serial

// Pick one LoRa or WiFi as the data reporting link. These are mutually exclusive.
#define USE_LORA false   // Use LoRa as the reporting link
#define USE_WIFI true  // Use WiFi as the reporting link

#if USE_LORA
String model = "DS-VC-LIDAR-LORA-1";
String model_description = "(LIDAR Traffic Counter with LoRa Back Haul)";
#endif

#if USE_WIFI
String model = "DS-VC-LIDAR-WIFI-1";
String model_description = "(LIDAR Traffic Counter with WiFi Back Haul)";
#define APPEND_RAW_DATA_WIFI true // In USE_WIFI mode, add 100 points of raw LIDAR data to 
// the wifi JSON msg for analysis at the server.
#endif

#define NOEVENT 0
#define LANE1EVENT 1
#define LANE2EVENT 2

//---------------------------------------------------------------------------------------------

#include <digameDebug.h>      // Debug message handling.
#include "digameVersion.h"    // Global SW version constants
#include <digameJSONConfig.h> // Read program parameters from config file on SD card. 
//  TODO: Re-think this strategy. Too much coupling.
//  The config struct is used all over the place.

Config config;                // Declare here so other libs have access to the Singleton.

// Utility Function
void DEBUG_LOG(String message);
void DEBUG_LOG(String message){
  if (config.logDebugEvents != "checked") return;
  appendTextFile("/eventlog.txt", "[" + String(millis()) +  "] "  + message + "\n");
}

#include <digameTime.h>       // Time Functions - RTC, NTP Synch etc
#include <digameNetwork.h>    // Network Functions - Login, MAC addr
#include <digamePowerMgt.h>   // Power management modes 
#include <digameDisplay.h>    // eInk Display Functions
#include <digameLIDAR.h>      // Functions for working with the TFMini series LIDAR sensors
#if USE_LORA
#include <digameLoRa.h>       +// Functions for working with Reyax LoRa module
#endif

#include <digameCounterWebServer.h>  // Web page to tweak parameters. Uses 'count' to update
// the UI with the current value. TODO: Find a cleaner way
// of doing this...

#include <CircularBuffer.h>   // Adafruit library for handling circular buffers of data. 
// https://github.com/rlogiacco/CircularBuffer

//---------------------------------------------------------------------------------------------

const int MAX_MSGBUFFER_SIZE = 5; // The maximum number of messages we'll buffer
CircularBuffer<String *, MAX_MSGBUFFER_SIZE> msgBuffer; // The buffer containing pointers to JSON messages
                                                        //  to be sent to the LoRa basestation or server.
String msgPayload;   // The message being sent to the base station.
                     // (JSON format depends on link type: LoRa or WiFi)

// Access point mode
bool accessPointMode = false; // In AP mode, we provide a web interface for configuration
bool usingWiFi = false;       // True if USE_WIFI or AP mode is enabled

// Multi-Tasking
SemaphoreHandle_t mutex_v;        // Mutex used to protect variables across RTOS tasks.
TaskHandle_t messageManagerTask;  // A task for handling data reporting
TaskHandle_t displayManagerTask;  // A task for updating the EInk display

// Messaging flags
bool   jsonPostNeeded         = false; // We have data to send
bool   bootMessageNeeded      = true;  // Send boot and heartbeat messages at startup
bool   heartbeatMessageNeeded = true;  // Periodic 'I'm alive!' message
int    vehicleMessageNeeded   = 0;     // 0=false, 1=lane 1, 2=lane

// Over the Air Updates work when we have WiFi or are in Access Point Mode...
bool useOTA = true;

// DIO
int    LED_DIAG  = 12;     // Indicator LED
int    CTR_RESET = 32;     // Counter Reset Input

// Bookeeping
String myMACAddress;       // Read at boot
unsigned long bootMillis = 0; // When did we boot according to millis()? Used to calculate uptime.
int    bootMinute;         // The minute (0-59) within the hour we woke up at boot.
int    heartbeatMinute;    // We issue Heartbeat message once an hour. Holds the minute we do it.
int    oldheartbeatMinute; // Value the last time we looked.
unsigned long lastHeartbeatMillis = 0; // millis() value of the last hearbeat
String currentTime;        // Set in the main loop from the RTC
int    currentSecond;      // Set in the main loop from the RTC
int    lidarReadingAtBoot; // At boot, do a reading of the LIDAR to check if it's blocked. Used
                           //   as a trigger to go into access point (AP) mode.

bool wifiMessagePending = true;

bool inTestMode = false;    // When we are using the web UI, don't send messages to the server.


//---------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------

// Used in setup()
void loadConfiguration(String &statusMsg);
void showSplashScreen();
void configurePorts(String &statusMsg);
void configureEinkDisplay(String &statusMsg);
int  configureLIDAR(String &statusMsg);
void configureLoRa(String &statusMsg);
void configureAPMode(String &statusMsg);
void configureStationMode(String &statusMsg);
void configureRTC(String &statusMsg);
void configureNetworking(String &statusMsg);
void configureCore0Tasks(String &statusMsg);
void configureTimers(String &statusMsg);

// Used in loop()
void handleBootEvent();       // Boot messages are sent at startup.
void handleResetEvent();      // Look for reset flag getting toggled
void handleModeButtonPress(); // Check for display mode button being pressed and switch display
void handleVehicleEvent();    // Read the LIDAR sensor and enque a count event msg, if needed
void handleHeartBeatEvent();  // Check timers and enque a heartbeat event msg, if needed


// Check the free heap and see how much as been gained/lost since the last call. 
// TODO: consider moving to digameDebug.h 
void heapCheck(String title){
  //return; // quick return when not needed. 
  static uint32_t newHeap = 0;
  static uint32_t oldHeap = 0; 

  String strOutput = "\n";
  strOutput +=  "*** " + title + "\n";
  //strOutput +=  " Old Heap:   " + String(oldHeap) +"\n";
  newHeap = ESP.getFreeHeap();
  strOutput +=  " Heap:   " + String(newHeap) +"\n";
  int32_t delta; 
  delta = newHeap - oldHeap;
  //strOutput +=  " Heap Delta: " + String(delta) + "\n\n"; 
  
  DEBUG_PRINT(strOutput);
  DEBUG_LOG(strOutput);

  oldHeap = newHeap;  
  
}

//****************************************************************************************
void setup() // DEVICE INITIALIZATION
//****************************************************************************************
{
  String statusMsg = "\n";       // String to hold the results of self-test
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

#if USE_LORA
  setLowPowerMode(); // Slow Down CPU to 40MHz and turn off Bluetooth & WiFi. See: DigamePowerMangement.h
  powerMode = LOW_POWER;
#endif

#if USE_WIFI
  setMediumPowerMode(); // Slow Down CPU to 80MHz and turn off Bluetooth. See: DigamePowerMangement.h
  powerMode = MEDIUM_POWER;
#endif

  configurePorts(statusMsg);     // Set up UARTs and GPIOs

  showSplashScreen();
  
  configureEinkDisplay(statusMsg);

  heapCheck("After configuring Eink");
  
  loadParameters(statusMsg);     // Grab program settings from SD card

  heapCheck("System BOOT...");
  
  heapCheck("After Loading parameters");
  
  lidarReadingAtBoot = configureLIDAR(statusMsg); // Sets up the LIDAR Sensor and
                                                  // returns an intial reading.
  heapCheck("After lidar config");
 

  config.ssid = "Foo";
  config.password = "ohpp8972";

  configureNetworking(statusMsg);

  heapCheck("After network config");

#if USE_LORA
  configureLoRa(statusMsg);
  heapCheck("After LoRa config");  
#endif

  configureRTC(statusMsg);         // Configure RTC. Set it if NTP is available.

  DEBUG_PRINTLN("***************************************************************");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("LAUNCHING BACKGROUND TASKS");

  heapCheck("After config RTC");

  displayRawText("STATUS", statusMsg);
  displayCopyright();
  
  delay(5000);

  displayTitle("COUNT");
  displayCopyright();
  showValue(count);

  heapCheck("After Eink splash and counter screen");

  configureCore0Tasks(statusMsg);  // Set up eink display and message tasks on core 0

  configureTimers(statusMsg);      // intitialize timer variables

  heapCheck("After setup");

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("***************************************************************");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("RUNNING!\n");

}


//****************************************************************************************
void loop() // MAIN LOOP
//****************************************************************************************
{
  unsigned long T1, T2; // for timing the acquisition loop

  static unsigned long T3 = 0;
  static unsigned long T4 = 0; // For network posts and counter updates

  T1 = millis(); // Time at the start of the loop.

  if (!accessPointMode) {
    handleModeButtonPress(); // Check for display mode button being pressed and switch display
    handleBootEvent();       // Boot messages are sent at startup.
    handleHeartBeatEvent();  // Check timers and enque a heartbeat event msg, if needed
  }

  handleResetEvent();        // Look for reset flag getting toggled.
  handleVehicleEvent();      // Read the LIDAR sensor and enque a count event msg, if needed 
  countDisplayManager(NULL);// TESTING MOVING THIS INTO THE MAIN LOOP. -- Turned off spinner updates to check speed.
  
  T4 = millis();
  if ((T4-T3) > 10000){
    //DEBUG_PRINTLN(String(T4) + ", " + String(T3));
    //DEBUG_PRINTLN("T4-T3 =" + String(T4-T3));
    messageManager(NULL);     // TESTING MOVING THIS INTO THE MAIN LOOP
    DEBUG_PRINTLN("Kicking the WDT from loop()...");
    esp_task_wdt_reset();
    T3 = millis();
  }
  

  T2 = millis();
  if ((T2 - T1) < 20) { // Adjust to c.a. 50 Hz.
    delay(20 - (T2 - T1));
  }
  
  upTimeMillis = millis() - bootMillis;

}


//**************************************************************************************
void loadParameters(String &statusMsg)
//**************************************************************************************
{
  // Setup SD card and load default values.
  if (initJSONConfig(filename, config))
  {
    statusMsg += "    SD   : OK\n\n";
  } else {
    statusMsg += "    SD   : ERROR!\n\n";
  };
}


//**************************************************************************************
void configureTimers(String &statusMsg) {
//**************************************************************************************
  bootMillis          = millis();
  upTimeMillis        = millis() - bootMillis;
  lastHeartbeatMillis = millis();
  currentTime         = getRTCTime();
  bootMinute          = getRTCMinute();
}


//**************************************************************************************
void configureCore0Tasks(String &statusMsg) {
//**************************************************************************************
  // Set up two tasks that run on CPU0 -- One to handle updating the display and one to
  // handle reporting data
  mutex_v = xSemaphoreCreateMutex(); // The mutex we will use to protect variables
  // across tasks

  // Create a task that will be executed in the messageManager() function,
  //   with priority 0 and executed on core 0

  /*
  xTaskCreatePinnedToCore(
    messageManager,      //* Task function. 
    "Message Manager",   //* name of task. 
    8000,                //* Stack size of task  was 10000
    NULL,                //* parameter of the task 
    0,                   //* priority of the task 
    &messageManagerTask, //* Task handle to keep track of created task 
    0);                  //* pin task to core 0 
   */

  /*/ Create a task that will be executed in the CountDisplayManager() function,
  // with priority 0 and executed on core 0
  xTaskCreatePinnedToCore(
    countDisplayManager, //* Task function. 
    "Display Manager",   //* name of task. 
    2000,                //* Stack size of task - Originally 10000
    NULL,                //* parameter of the task 
    0,                   //* priority of the task 
    &displayManagerTask, //* Task handle to keep track of created task 
    0);
  */
}


//**************************************************************************************
void configureNetworking(String &statusMsg) {
  //**************************************************************************************
  myMACAddress = getMACAddress();

  // If the unit is unconfigured or is booted with the RESET button held down, enter AP mode.
  // Recently added a check to see if the unit was booted with something in front of the
  // sensor. If so, come up in AP mode. -- This way folks don't have to press the button at
  // boot. (LIDAR has to be configured before calling this function for that feature to work!)
  accessPointMode = ( (config.deviceName == "YOUR_DEVICE_NAME") ||
                      (digitalRead(CTR_RESET) == LOW) ||
                      (lidarReadingAtBoot < 20)
                    );

  if (accessPointMode) {
    inTestMode = true; 
    setMediumPowerMode();
    configurePorts(statusMsg);
    String s;
    configureLIDAR(s);
    useOTA = true;
    usingWiFi = true;
    configureAPMode(statusMsg);
  } else {
    // Data reporting over the LoRa link.
    #if USE_LORA
        useOTA = false;
        usingWiFi = false;
    #endif
    
    // Data reporting over the WiFi link
    #if USE_WIFI
        useOTA = false;  // TEST TEST TEST! - Turning off OTA updates for now.
        usingWiFi = true;
        configureStationMode(statusMsg);
        inTestMode = false;

        DEBUG_PRINTLN("    Initializing web server.");
        initWebServer();
        http.setReuse(true); // See digameNetwork.h for this guy.
    #endif
  }

  if (accessPointMode) {
    //DEBUG_PRINTLN("    Initializing web server.");
    //initWebServer();
    //http.setReuse(true); // See digameNetwork.h for this guy.
  }

  DEBUG_PRINT("  USE OTA: ");
  DEBUG_PRINTLN(useOTA);

}

//**************************************************************************************
void configureRTC(String &statusMsg) {
//**************************************************************************************
  // Check that the RTC is present
  if (initRTC()) {
    statusMsg += "    RTC  : OK\n";
    DEBUG_LOG("RTC initialized.");
  } else {
    statusMsg += "    RTC  : ERROR!\n";
    DEBUG_LOG("RTC failed to initialize.");
  }

  // Synchronize the RTC to an NTP server, if available
#if USE_WIFI
  if (wifiConnected) { // Attempt to synch ESP32 clock with NTP Server... 
    DEBUG_LOG("Attempting to synch ESP32 clock with NTP Server...");
    synchTimesToNTP();
  } else {
    DEBUG_LOG("WiFi not connected. RTC not synched to NTP.");
  }
#endif

  DEBUG_PRINTLN();

}



#if USE_LORA
//****************************************************************************************
void configureLoRa(String &statusMsg) {
  //**************************************************************************************
  initLoRa();

  // Configure radio params
  DEBUG_PRINT("  Configuring LoRa...");
  if (configureLoRa(config)) {
    statusMsg += "    LoRa : OK\n\n";
    DEBUG_PRINTLN(" OK.");
  } else {
    statusMsg += "    LoRa : ERROR!\n\n";
    DEBUG_PRINTLN(" ERROR!");
  }
}
#endif

//****************************************************************************************
void configureStationMode(String &statusMsg) {
//****************************************************************************************
  DEBUG_LOG("Setting WiFi to Station Mode…");

  int networkRetryCount = 0;
  
  while (networkRetryCount<5){
    if (enableWiFi(config)) {
      DEBUG_PRINTLN("    WiFi : OK");
      statusMsg += "    WiFi : OK\n\n";
      DEBUG_LOG("WiFi enabled in Station Mode.");
      break;
    } else {
      DEBUG_PRINTLN("    WiFi : ERROR!");
      statusMsg += "    WiFi : ERROR!\n\n";
      networkRetryCount += 1;
      DEBUG_LOG("WiFi failed to enable in Station Mode. Retrying... Retry Count: " + String(networkRetryCount));
      delay(1000);
    }
  }

  displayCenteredText("NETWORK", "(Station Mode)", "", "", "IP Address", String(WiFi.localIP().toString()));
  displayCopyright();
  delay(5000);
  //statusMsg += "    WiFi : OK\n\n";
}


//****************************************************************************************
// Configure the device as an access point. TODO: move to DigameNetwork.h
void configureAPMode(String &statusMsg) {
//**************************************************************************************
  //String mySSID = String("Digame_AP");// + getShortMACAddress();
  const char* ssid = "Digame_AP"; //mySSID.c_str();

  DEBUG_PRINTLN("  Stand-Alone Mode. Setting AP (Access Point).");
  DEBUG_LOG("Setting WiFi to AP (Access Point) Mode.");
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();
  
  DEBUG_PRINT("    AP SSID:       ");
  DEBUG_PRINTLN(ssid);
  DEBUG_PRINT("    AP IP Address: ");
  DEBUG_PRINTLN(IP);
  
  displayCenteredText("NETWORK", "(AP Mode)"     , "SSID", ssid,  "", "IP Address", String(WiFi.softAPIP().toString()));
  displayCopyright();

  delay(10000);
}

//**************************************************************************************
int configureLIDAR(String &statusMsg) {
  //**************************************************************************************
  // Turn on the LIDAR Sensor and take an initial reading (initLIDARDist)
  if (initLIDAR(true)) { // Triggered mode = TRUE.
    statusMsg += "    LIDAR: OK\n\n";
  } else {
    statusMsg += "    LIDAR: ERROR!\n\n";
  }
  return initLIDARDist; // sloppy. TODO: return with initLIDAR.
}


//**************************************************************************************
void configureEinkDisplay(String &statusMsg) {
  //**************************************************************************************
  initDisplay();
  showWhite();

  //displayCenteredText("DIGAME", "(LIDAR)", "", "Vehicle", "Counting System", "Version", SW_VERSION);
#if USE_LORA
  displayCenteredText("DIGAME", "(LIDAR-LoRa)", "", "Vehicle", "Counting System", "Version", SW_VERSION);
#else
    displayCenteredText("DIGAME", "(LIDAR-WiFi)", "", "Vehicle", "Counting System", "Version", SW_VERSION);
#endif
  displayCopyright();

  //displaySplashScreen("(LIDAR Counter)", SW_VERSION);
}


//****************************************************************************************
// Prepare the GPIOs and debugUART serial port. Bring up the Wire interface for SPI bus.
void configurePorts(String &statusMsg) {
  //**************************************************************************************
  // Ready the LED and RESET pins
  pinMode(LED_DIAG, OUTPUT);
  pinMode(CTR_RESET, INPUT_PULLUP);
  debugUART.begin(115200);   // Intialize terminal serial port
  delay(1000);               // Give port time to initalize
  Wire.begin();
}


//****************************************************************************************
void showSplashScreen() {
  //**************************************************************************************
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);

  DEBUG_PRINTLN("***************************************************************");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("              DIGAME Vehicle Counting System");
  DEBUG_PRINTLN("                 - VEHICLE COUNTING UNIT -");
  DEBUG_PRINTLN();
  DEBUG_PRINT(model);
  DEBUG_PRINTLN(model_description);
  DEBUG_PRINTLN("Version: " + SW_VERSION);
  DEBUG_PRINT("Main Loop Running on Core #: ");
  DEBUG_PRINTLN(xPortGetCoreID());
  DEBUG_PRINTLN("Copyright 2023, Digame Systems. All rights reserved.");
  DEBUG_PRINTLN();
  DEBUG_PRINT("Compiled on ");
  DEBUG_PRINT(compileDate);
  DEBUG_PRINT(" at ");
  DEBUG_PRINTLN(compileTime);
  DEBUG_PRINT("Free Heap: ");
  DEBUG_PRINTLN(ESP.getFreeHeap());
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("***************************************************************");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("HARDWARE INITIALIZATION");
  DEBUG_PRINTLN();
}


//**************************************************************************************
// Add a message to the queue for transmission.
void pushMessage(String message) {
  //**************************************************************************************
  xSemaphoreTake(mutex_v, portMAX_DELAY);
#if USE_LORA
  String * msgPtr = new String(message);
  msgBuffer.push(msgPtr);
#endif

#if USE_WIFI
  if (accessPointMode == false) {
    String * msgPtr = new String(message);
    msgBuffer.push(msgPtr);
  }
#endif
  xSemaphoreGive(mutex_v);
  DEBUG_LOG("Message pushed to queue. Queue size: " + String(msgBuffer.size()));
  DEBUG_PRINTLN("Message pushed to queue. Queue size: " + String(msgBuffer.size()));

}


//****************************************************************************************
// LoRa can't handle big payloads. We use a terse JSON message in this case.
String buildLoRaJSONHeader(String eventType, double count, String lane = "1") {
  //**************************************************************************************
  String loraHeader;
  String strCount;

  strCount = String(count, 0);
  strCount.trim();

  if (rtcPresent()) {
    loraHeader = "{\"ts\":\"" + getRTCTime(); // Timestamp
  } else {
    loraHeader = "{\"ts\":\"" + getESPTime();
  }

  loraHeader = loraHeader +
               "\",\"ma\":\"" + myMACAddress +
               "\",\"v\":\""  + TERSE_SW_VERSION + // Firmware version
               "\",\"et\":\"" + eventType +        // Event type: boot, heartbeat, vehicle
               "\",\"c\":\""  + strCount +         // Total counts registered
               "\",\"t\":\""  + String(getRTCTemperature(), 1) + // Temperature in C
               "\",\"r\":\"" + "0" ;               // Retries

  if (eventType == "v") {
    loraHeader = loraHeader +
                 "\",\"da\":\"" + "t";             // Detection algorithm (Threshold)
    loraHeader = loraHeader +
                 "\",\"l\":\"" + lane + "\"";      // Lane number for the vehicle event
  }

  if ((eventType == "b") || (eventType == "hb")) { //In the boot/heartbeat messages, send the current settings.
    loraHeader = loraHeader +
                 "\",\"s\":{" +
                 "\"ui\":\"" + config.lidarUpdateInterval  + "\"" +
                 ",\"sf\":\"" + config.lidarSmoothingFactor + "\"" +
                 ",\"rt\":\"" + config.lidarResidenceTime   + "\"" +
                 ",\"1m\":\"" + config.lidarZone1Min        + "\"" +
                 ",\"1x\":\"" + config.lidarZone1Max        + "\"" +
                 ",\"2m\":\"" + config.lidarZone2Min        + "\"" +
                 ",\"2x\":\"" + config.lidarZone2Max        + "\"" +
                 "}";
  }
  // DEBUG_PRINTLN(loraHeader);
  return loraHeader;
}

//****************************************************************************************
// WiFi can handle a more human-readable JSON data payload.
String buildWiFiJSONHeader(String eventType, double count, String lane = "1") {
  //**************************************************************************************
  String jsonHeader;
  String strCount;

  strCount = String(count, 0);
  strCount.trim();

  if (eventType == "b") eventType  = "Boot";
  if (eventType == "hb") eventType = "Heartbeat";
  if (eventType == "v") eventType  = "Vehicle";

  jsonHeader = "{\"deviceName\":\""    + config.deviceName +
               "\",\"deviceMAC\":\""   + myMACAddress +      // Read at boot
               "\",\"firmwareVer\":\"" + TERSE_SW_VERSION  +
               "\",\"timeStamp\":\""   + getRTCTime() +       
               "\",\"eventType\":\""   + eventType +
               "\",\"count\":\""       + strCount +          // Total counts registered
               "\",\"temp\":\""        + String(getRTCTemperature(), 1); // Temperature in C

  if (eventType == "Vehicle") {
    jsonHeader = jsonHeader +
                 "\",\"detAlgorithm\":\"" + "Threshold";// Detection algorithm (Threshold)
    jsonHeader = jsonHeader +
                 "\",\"lane\":\"" + lane + "\"";        // Lane in which the vehicle was seen
  }

  if ((eventType == "Boot") || (eventType == "Heartbeat")) { //In the boot/heartbeat messages, send the current settings.
    jsonHeader = jsonHeader +
                 "\",\"settings\":{" +
                 "\"ui\":\"" + config.lidarUpdateInterval   + "\"" +
                 ",\"sf\":\"" + config.lidarSmoothingFactor + "\"" +
                 ",\"rt\":\"" + config.lidarResidenceTime   + "\"" +
                 ",\"1m\":\"" + config.lidarZone1Min        + "\"" +
                 ",\"1x\":\"" + config.lidarZone1Max        + "\"" +
                 ",\"2m\":\"" + config.lidarZone2Min        + "\"" +
                 ",\"2x\":\"" + config.lidarZone2Max        + "\"" +
                 "}";
  }

  //  jsonHeader = jsonHeader + "\"";

  return jsonHeader;
}

//****************************************************************************************
// LoRa messages to the server all have a similar format. This builds the common header.
String buildJSONHeader(String eventType, double count, String lane = "1") {
  //**************************************************************************************
  String retValue = "";

#if USE_LORA
  retValue = buildLoRaJSONHeader(eventType, count, lane);
  //DEBUG_PRINTLN(retValue);
  return retValue;
#endif

#if USE_WIFI
  return buildWiFiJSONHeader(eventType, count, lane);
#endif

}


//****************************************************************************************
/* Playing around with scheduling message delivery to minimize interference between LoRa
    counters.

    Divide the minute up into equal portions for the number of counters
    we are dealing with. Each counter has his own window within the minute to transmit.

    TODO: Think about having the base station tell us
    how many friends we have in the area and which counter we are...

    counterNumber = 1 to 4
    numCounters   = 1 to 4
*/
bool inTransmitWindow(int counterNumber, int numCounters = 3) {
  //**************************************************************************************

  int thisSecond;
  thisSecond = getRTCSecond();

  //return true; // Uncomment to turn off time window check and allow counters to respond at any time.

  if (config.showDataStream == "false") {
    //DEBUG_PRINT("This Second: ");
    //DEBUG_PRINTLN(thisSecond);
  }

  int windowInterval = 60 / (numCounters);
  // EX: for four counters, we get windows of...
  // counter 0: 0-14,
  // counter 1:      15-29,
  // counter 2:            30-44,
  // counter 3:                  45-59

  if ( (thisSecond >=   (counterNumber - 1) * windowInterval ) &&
       (thisSecond <=  ( (counterNumber - 1) * windowInterval + windowInterval ) )
     )
  {
    return true;
  } else {
    return false;
  }
}


//****************************************************************************************
// A task that runs on Core0 using a circular buffer to enqueue messages to the server...
// Current version keeps retrying forever if an ACK isn't received. 
// TODO: Fix.
// Later: Put a retry counter in to fix. 

void messageManager(void *parameter) {

//**************************************************************************************

  bool       messageACKed = true;
  static int retryCount = 0;

  //DEBUG_PRINTLN();
  //DEBUG_PRINT("  Message Manager Running on Core #: ");
  //DEBUG_PRINTLN(xPortGetCoreID());

  //for (;;) {

    //*******************************
    // Process a message on the queue
    //*******************************
    if (msgBuffer.size() ==  0) { // If there are no messages in the queue, quit.
      return;
    }

    // If there are one or more messages in the queue and we are in a transmit window,
    while (msgBuffer.size() > 0)  // Try to send what we have. 
    //&& (inTransmitWindow(config.counterID.toInt(), config.counterPopulation.toInt())) 
    {

      if (config.showDataStream == "false") {
        DEBUG_PRINT("Message Buffer Size: ");
        DEBUG_PRINTLN(msgBuffer.size());
        heapCheck("About to attempt post...");
      }

      String activeMessage = String(msgBuffer.first()->c_str()); // Read from the buffer without removing the data from it.

#if USE_LORA
      // Send the data to the LoRa-WiFi base station that re-formats and routes it to the
      // ParkData server.      
      if (inTestMode){
        messageACKed = true; 
      } else {
        messageACKed = sendReceiveLoRa(activeMessage, config);
      }
#endif
  
#if USE_WIFI
      // Send the data directly to the ParkData server via http(s) POST
      //DEBUG_PRINT("Power Mode: ");
      //DEBUG_PRINTLN(powerMode);
      
      if (powerMode == LOW_POWER) { // Speed up CPU and turn on WiFi.
        setMediumPowerMode();
        Serial.begin(115200);
        delay(500);
        DEBUG_PRINTLN();
        String s;
        configureLIDAR(s); // Reconfigure LIDAR sensor
      }

      if (inTestMode){
        messageACKed = true; 
      } else {
        messageACKed = postJSON(activeMessage, config); 
        DEBUG_PRINTLN("Kicking the WDT from messageManager()...");
        esp_task_wdt_reset();
      }
      
#endif

      if (messageACKed) // If the message was ACK'd, remove it from the queue.
      {
        // Message sent and received. Take it off of the queue.
        xSemaphoreTake(mutex_v, portMAX_DELAY);
          String  * entry = msgBuffer.shift();
          delete entry;
        xSemaphoreGive(mutex_v);

        if (config.showDataStream == "false")
        {
          DEBUG_PRINTLN("Success!");
          DEBUG_PRINTLN();
          wifiMessagePending = false;
        }
      } else { // The message failed. No ACK. 
        
        if (config.showDataStream == "false")
        {
          retryCount += 1;
          DEBUG_PRINTLN();
          DEBUG_PRINTLN("******* Message delivery error! *********");
        }

      }

    } // End of while loop

   // vTaskDelay(100 / portTICK_PERIOD_MS);
  //}
}


//****************************************************************************************
// Text-based "I'm alive" indicator that we stick on the eInk display and update
// periodically to show the program is running. TODO: move to a utlilities library.
String rotateSpinner() {
  //**************************************************************************************
  static String spinner = "|";

  //OLD SCHOOL! :)
  if (spinner == "|") {
    spinner = "/";
  } else if (spinner == "/") {
    spinner = "-";
  } else if (spinner == "-") {
    spinner = "\\";
  } else {
    spinner = "|";
  }
  return spinner;
}


//****************************************************************************************
// A task that runs on Core0 to update the display when the count changes.
//**************************************************************************************
void countDisplayManager(void *parameter) {
  //int countDisplayUpdateRate = 200;
  static unsigned int oldCount = 0;

  //if (config.showDataStream == "false") {
  //  DEBUG_PRINT("  Display Manager Running on Core #: ");
  //  DEBUG_PRINTLN(xPortGetCoreID());
  //}
  
  if (count == oldCount) return;
  
  //for (;;) {
    //if (count != oldCount) {
      // Total refresh every 10 counts. (Or when we zero out the counter.)
      if (count % 10 == 0) {
        //initDisplay();
        showWhite();
        displayTitle("COUNT");
        displayCopyright();
      }
      showValue(count);
      oldCount = count;
    //}
    //showPartialXY(rotateSpinner(), 180, 180);
    //upTimeMillis = millis() - bootMillis; //TODO: Put this somewhere else.
    //vTaskDelay(countDisplayUpdateRate / portTICK_PERIOD_MS);
  //}
}

//***************************************************************************************
// Check for display mode button being pressed and reset the vehicle count
void handleModeButtonPress() {
  //**************************************************************************************
  // Check for RESET button being pressed. If it has been, reset the counter to zero.
  if (digitalRead(CTR_RESET) == LOW) {
    count = 0;
    outCount = 0;
    inCount = 0;
    clearLIDARDistanceHistogram();
    if (config.showDataStream == "false") {
      DEBUG_PRINT("Loop: RESET button pressed. Count: ");
      DEBUG_PRINTLN(count);
    }
  }
}

//**************************************************************************************
void handleBootEvent() {
  //**************************************************************************************
  if (bootMessageNeeded) {
    msgPayload = buildJSONHeader("b", count);
    msgPayload = msgPayload + "}";
    pushMessage(msgPayload);
    DEBUG_LOG("BOOT Event!: " + msgPayload + "\n");
    bootMessageNeeded = false;
  }
}


//**************************************************************************************
void handleResetEvent() {
  //**************************************************************************************  
  if (resetFlag) {
    DEBUG_PRINTLN("Reset flag has been flipped. Rebooting the processor.");
    delay(1000);
    ESP.restart();
  }
}

//**************************************************************************************
void handleHeartBeatEvent() { // Issue a heartbeat message, if needed.
//**************************************************************************************

  unsigned long deltaT = (millis() - lastHeartbeatMillis); // How long has it been since 
                                                           // we issued a heartbeat?
  unsigned long slippedMilliSeconds = 0; // A variable to track how late we are on our HB.
  
  if ( (deltaT) >= config.heartbeatInterval.toInt() * 1000 ) { // If it has it been longer 
                                                               // than heartbeatInterval 
                                                               // since we issued a 
                                                               // heartbeat...
    // Figure out how late we are... 
    slippedMilliSeconds = deltaT - config.heartbeatInterval.toInt() * 1000; 
    // Since this Task is on a schedule, we'll always be a little late...

    lastHeartbeatMillis = millis() - slippedMilliSeconds;
    
    if (config.showDataStream == "false") {
      DEBUG_PRINT("millis: ");
      DEBUG_PRINTLN(deltaT);
      DEBUG_PRINT("slipped ms: ");
      DEBUG_PRINTLN(slippedMilliSeconds);
    }
    
    heartbeatMessageNeeded = true;
  }

  if (heartbeatMessageNeeded) {
    
    // *** Build and enque the JSON message.
    heartbeatMessageNeeded = false; 

    msgPayload = buildJSONHeader("hb", count);
    msgPayload = msgPayload + "}";
    pushMessage(msgPayload); // This puts the msg on a queue for sending. It may succeed. It may fail. 
                             // The sending operation runs on a separate task. See messageManager. 
    
    DEBUG_LOG("HEARTBEAT Event!: " + msgPayload + "\n");

    
  }
}


//**************************************************************************************
String appendRawDataToMsgPayload() {
  //**************************************************************************************
  // Vehicle passing event messages may include raw data from the sensor.
  // If so, tack the data buffer on the JSON message.

  //DEBUG_PRINTLN("In appendRawDataToMsgPayload");
  //DEBUG_PRINTLN("History buffer size: ");
  //DEBUG_PRINTLN(lidarHistoryBuffer.size());


  msgPayload = msgPayload + ",\"rawSignal\":[";

  using index_t = decltype(lidarHistoryBuffer)::index_t;
  for (index_t i = 0; i < lidarHistoryBuffer.size(); i++) {
    msgPayload = msgPayload + lidarHistoryBuffer[i];
    if (i < lidarHistoryBuffer.size() - 1) {
      msgPayload = msgPayload + ",";
    }
  }
  msgPayload = msgPayload + "]";

  //DEBUG_PRINTLN(msgPayload);
  return msgPayload;
}


//**************************************************************************************
void printDataStreamEntry(String s) {
//**************************************************************************************
  DEBUG_PRINTLN(s);
}


//****************************************************************************************
void handleVehicleEvent() { // Test if vehicle event has occured. Route message if needed.
//****************************************************************************************
  
  vehicleMessageNeeded = processLIDARSignal3(config); 

  /****************** BEGIN STRESS TEST ******************************
  int desiredReturnValue = 0;
  static unsigned long lastEventTime = 0; 
  unsigned long eventTime = millis();
  if ((eventTime - lastEventTime)>50) {
    DEBUG_PRINTLN("LAST EVENT TIME: " + String(lastEventTime));
    DEBUG_PRINTLN("EVENT TIME: " + String(eventTime));
    desiredReturnValue = 1;
    lastEventTime = eventTime;
  }
  vehicleMessageNeeded = processLIDARSignal4(config, desiredReturnValue);
  ****************** END STRESS TEST ********************************/

  // if we're in test mode, don't send messages to the server.
  if (config.showDataStream == "true") {
     printDataStreamEntry(getLIDARStreamEntry());
     vehicleMessageNeeded = 0;
     return;
  } 

  if (msgBuffer.size() >= MAX_MSGBUFFER_SIZE) {
    DEBUG_PRINTLN("Message buffer full. Dropping message.");
    vehicleMessageNeeded = 0;
    return;
  }

  if (vehicleMessageNeeded > 0) {

    if (lidarBuffer.size() == lidarSamples) { // Fill up the buffer before processing so
      // we don't get false events at startup.

      if (vehicleMessageNeeded == LANE1EVENT) {

        inCount++;
      } else if (vehicleMessageNeeded == LANE2EVENT) {
        outCount++;
      }

      count = inCount + outCount;

      config.lidarZone1Count = String(inCount); // Update this so entities making use of config have access to the current count.
      // e.g., digameWebServer.
      config.lidarZone2Count = String(outCount);

      if (config.showDataStream == "false") {

        String strResult = "VEHICLE Event in Lane " + String(vehicleMessageNeeded) + "!: Total counts: " + String(count) + "\n"; 
        strResult = strResult + " RTC Time: " + getRTCTime() + "\n";
        strResult = strResult + " Lane 1: " + String (inCount) + " Lane 2: " + String(outCount) + "\n";
        
        DEBUG_PRINTLN(strResult);
        DEBUG_LOG(strResult);

      }

      if (!accessPointMode) {
        if (vehicleMessageNeeded == LANE1EVENT) {
          msgPayload = buildJSONHeader("v", inCount, String(vehicleMessageNeeded));
        } else if (vehicleMessageNeeded == LANE2EVENT) {
          msgPayload = buildJSONHeader("v", outCount, String(vehicleMessageNeeded));
        }

#if (USE_WIFI) && (APPEND_RAW_DATA_WIFI)
        appendRawDataToMsgPayload();
#endif

        msgPayload = msgPayload + "}"; // Close out the JSON

        pushMessage(msgPayload); // Don't enque messages in AP mode.

        if (config.logVehicleEvents == "checked") {
          appendTextFile("/eventlog.txt", msgPayload);
        }
      }
    }

    vehicleMessageNeeded = 0;
  }
}
