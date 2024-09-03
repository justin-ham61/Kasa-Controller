#ifndef CONFIG_H
#define CONFIG_H

typedef struct {
    const char* SSID;
    const char* PASSWORD;
} WifiParameters_t;

const char* SSID = "Odyssey";
const char* PASS = "Blue4524.";
WifiParameters_t wifi_params = {SSID, PASS};

char* aliases[] = {
  "Lauter",
  "Dejsa",
  "Ball",
  "Bedroom 1"
};

const int size = sizeof(aliases)/sizeof(aliases[0]);

#endif