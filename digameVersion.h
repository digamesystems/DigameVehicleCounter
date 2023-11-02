#ifndef __DIGAME_VERSION_H__
#define __DIGAME_VERSION_H__

const String SW_VERSION       = "0.9.92";
const String TERSE_SW_VERSION = "0992";  

/*
 * 0.9.82 -  Added in code for Websocket server in WiFi mode. When streaming and a client is connected
 *           data will be echoed on both the UART and the Websocket. ONLY USED IN WiFi MODE.
 *           
 * 0.9.83 -  LoRa mode changes: Reconfigure communications in LoRa AP mode after CPU speed switch. 
 *           Moved to independent count values for each lane. 
 *           Added MAC address to LoRa message to base station.
 *           
 * 0.9.84 -  Code cleanup, Add support for lora server parameters.           
 * 
 * 0.9.85/6 - Changes to support AP mode when running as a WiFi counter. (Network configuration)
 * 
 * 0.9.87  - Changes to accomodate refactoring of digameDisplay.h / digameDisplay.cpp
 * 
 * 0.9.88  - DEBUG Version for testing network error handling. 60 sec heartbeat configured in setup()
 * 
 * 0.9.89  - Removed code for network testing. Added back in /histo and /histograph links to web UI. 
 *           Cleaned up serial output. Changed size of message buffer to avoid memory overflow when 
 *           network is unavailable. Updated copyright statement. 
 * 
 */

#endif
