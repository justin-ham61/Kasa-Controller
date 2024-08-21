#ifndef CONFIG_H
#define CONFIG_H

typedef struct {
    char* SSID;
    char* PASSWORD;
} WifiParameters_t;

WifiParameters_t wifi_params = {"Odyssey", "Blue4524."};

char* aliases[] = {
  "Lauter",
  "Dejsa",
  "Ball"
};

#endif