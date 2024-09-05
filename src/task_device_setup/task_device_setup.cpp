#include "task_device_setup.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "Arduino.h"
#include <EEPROM.h>
#include <KasaSmartPlug.h>
#include <typedef/menu_item.h>

extern SemaphoreHandle_t wifiSemaphore;
extern KASAUtil kasaUtil;
extern menu_item menuItems[12];
extern TaskHandle_t display_task_handle;
extern int numberOfBulbs;
extern char* aliases[];
extern const int size;

void vAddDeviceTask(void *parameter)
{
        while(1){
        xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
        int read_index = EEPROM.read(1);
        int data_len = EEPROM.read(2);
        int alias_len = EEPROM.read(read_index);

        //Load fail safe
        int loop = 0;

        while(alias_len != 255 && loop < size){
            char alias[10];
            char ip[20];

            read_index++;
            if(read_index >= 256){
                read_index = 3;
            }

            for(int i = 0; i < alias_len; i++){
                alias[i] = char(EEPROM.read(read_index));
                read_index++;
                if(read_index >= 256){
                    read_index = 3;
                }
            }
            alias[alias_len] = '\0';

            int ip_len = EEPROM.read(read_index);
            read_index++;
            if(read_index >= 256){
                read_index = 3;
            }

            for(int i = 0; i < ip_len; i++){
                ip[i] = char(EEPROM.read(read_index));
                read_index++;
                if(read_index >= 256){
                    read_index = 3;
                }
            }
            ip[ip_len] = '\0';

            uint8_t dev_type = EEPROM.read(read_index);
            read_index++;
            if(read_index >= 256){
                read_index = 3;
            }      



            Serial.println(alias);
            Serial.println(ip);
            Serial.println(dev_type);

            for(int i = 0; i < size; i++){
                if(strcmp(alias, aliases[i]) == 0){
                    const char *alias_ptr = alias;
                    const char *ip_ptr = ip;
                    
                    switch(dev_type){
                        case 0: 
                            break;
                        case 1:
                            kasaUtil.CreateDevice(alias, ip, "bulb");
                            break;
                        case 2: 
                            kasaUtil.CreateDevice(alias, ip, "strip");
                            break;
                    }

                    Serial.print("Added from memory: ");
                    Serial.println(alias);
                }
            }

            memset(alias, 0, sizeof(alias));
            memset(ip, 0, sizeof(ip));

            alias_len = EEPROM.read(read_index);
            loop++;
        } 

        //Load devices from back up first, then scan for new devices or IP address update
        numberOfBulbs = kasaUtil.ScanDevicesAndAdd(1000, aliases, size);
        for(int i = 0; i < numberOfBulbs; i++){
            Serial.println(kasaUtil.GetSmartPlugByIndex(i)->alias);
            menuItems[i] = {kasaUtil.GetSmartPlugByIndex(i)->alias, 1, 0};
        }
        
        menuItems[numberOfBulbs] = {"White", 5, 1};
        menuItems[numberOfBulbs + 1] = {"Blue", 5, 1};
        menuItems[numberOfBulbs + 2] = {"Red", 5, 1};
        menuItems[numberOfBulbs + 3] = {"Green", 5, 1};
        menuItems[numberOfBulbs + 4] = {"Purple", 5, 1};
        menuItems[numberOfBulbs + 5] = {"Reset", 3, 2};
        menuItems[numberOfBulbs + 6] = {"Save", 4, 3};

        xTaskNotifyGive(display_task_handle);
        xSemaphoreGive(wifiSemaphore);
        vTaskSuspend(NULL);
    }
}