// CREDENTIALS
// This file is to set your "secret" credentials. E.g. in case you want to publish your code on GitHub,
// you can exclude this file from the repository; thus, your credentials are not published.
// (https://arduino.stackexchange.com/questions/40411/hiding-wlan-password-when-pushing-to-github)

#ifndef _CREDENTIALS_h
#define _CREDENTIALS_h


// Wifi
#define WIFI_SSID	"Your SSID here"
#define WIFI_PASSWD "Your Password here"

// NTP
// Note, if you want to use a different NTP server, e.g. to a local server as "Chrony" (addon for HomeAssistant),
// you can change the NTP_SERVER to the IP address of your NTP server (192.168.0:123).
#define NTP_SERVER	"pool.ntp.org"

// MQTT
#define MQTT_BROKER	"IP address of your MQTT broker"
#define MQTT_PORT	1883
#define MQTT_USER	"Your MQTT username"
#define MQTT_PASSWD "Your MQTT password"


#endif

