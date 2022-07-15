#ifndef __DIGAME_VERSION_H__
#define __DIGAME_VERSION_H__

const String SW_VERSION       = "0.9.83";
const String TERSE_SW_VERSION = "0983";  

/*
 * 0.9.82 -  Added in code for Websocket server in WiFi mode. When streaming and a client is connected
 *           data will be echoed on both the UART and the Websocket. ONLY USED IN WiFi MODE.
 *           
 * 0.9.83 -  LoRa mode changes: Reconfigure communications in LoRa AP mode after CPU speed switch. 
 *           Moved to independent count values for each lane. 
 *           Added MAC address to LoRa message to base station.
 *           
 *           
 */

#endif
