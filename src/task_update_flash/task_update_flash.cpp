#include "task_update_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include <EEPROM.h>
#include <KasaSmartPlug.h>

extern KASAUtil kasaUtil;
extern int numberOfBulbs;

//memory buffer layout
//length of alias -> alias
//length of ip -> ip
//device type
//end = demark

void vTaskUpdateFlash(void *parameter)
{
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
        Serial.println("notified");
        uint8_t memory_buffer[256];
        int index = 0;

        //Saving device with types
        //0 = plug, 1 = bulb, 2 = strip
        for (int i = 0; i < numberOfBulbs; i++){
            KASADevice* dev = kasaUtil.GetSmartPlugByIndex(i);
            uint8_t alias_len = static_cast<int>(strlen(dev->alias));
            memory_buffer[index] = alias_len;
            index++;
            for(int j = 0; j < alias_len; j++){
                memory_buffer[index] = dev->alias[j];
                index++;
            }

            uint8_t ip_len = static_cast<int>(strlen(dev->ip_address));
            memory_buffer[index] = ip_len;
            index++;
            for(int j = 0; j < ip_len; j++){
                memory_buffer[index] = dev->ip_address[j];
                index++;
            }

            //Save device type on a byte
            const char* dev_type = dev->getType();
            if(strcmp(dev_type, "KASASmartBulb") == 0){
                memory_buffer[index] = 1;
            } else if (strcmp(dev_type, "KASASmartStrip") == 0){
                memory_buffer[index] = 2;
            } else {
                memory_buffer[index] = 0;
            }
            index++;
        }

        memory_buffer[index] = 255;
        index++;

        //Initializes the begin index for EEPROM memory
        int begin = EEPROM.read(0);

        //Saves where the program should read from after this write
        if(begin == 256 - 3){
            EEPROM.write(1, 2);
        } else {
            EEPROM.write(1, begin + 3);
        }

        //Saves length of new data
        EEPROM.write(2, index);


        for(int i = 0; i < index; i++){
            if(begin >= 256 - 3){
                begin = 0;
            }
            EEPROM.write(begin + 3, static_cast<int>(memory_buffer[i]));
            begin++;
        }

        Serial.println();

        //Write the next begin index into flash
        EEPROM.write(0, begin);
        EEPROM.commit();
    }
}