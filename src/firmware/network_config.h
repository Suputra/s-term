#pragma once

#define MAX_WIFI_APS 8

struct WiFiAP {
    char ssid[64];
    char pass[64];
};
