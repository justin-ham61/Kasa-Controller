#include "task_command.h"
#include "typedef/command.h"
#include <freertos/FreeRTOS.h>
#include "Arduino.h"
#include "freertos/task.h"
#include <typedef/menu_item.h>
#include <KasaSmartPlug.hpp>

extern QueueHandle_t command_queue;
extern QueueHandle_t toggle_queue;
extern QueueHandle_t brightness_queue;
extern QueueHandle_t color_queue;
extern int numberOfBulbs;
extern SemaphoreHandle_t wifiSemaphore;
extern menu_item menuItems[12];
extern KASAUtil kasaUtil;
extern TaskHandle_t display_task_handle;

void vReadCommandTask(void *parameter)
{
    command curr_command;
    while(1){
        if(xQueueReceive(command_queue, (void *)&curr_command, portMAX_DELAY) == pdTRUE){
            if(curr_command.index < numberOfBulbs){
                switch(curr_command.task){
                    case 0: 
                        if(xQueueSend(toggle_queue, (void *)&curr_command, 10) != pdTRUE){
                            Serial.println("Toggle Queue is Full");
                        }
                        break;
                    case 1: 
                        if(xQueueSend(brightness_queue, (void *)&curr_command, 10) != pdTRUE){
                            Serial.println("Brightness Queue is Full");
                        }
                        break;
                    case 2: 
                        Serial.println("Temperature Control");
                        break;
                    case 3: 
                        if(xQueueSend(color_queue, (void *)&curr_command, 10) != pdTRUE){
                            Serial.println("Color queue is full");
                        }

                        break;
                }
            } else {
                Serial.println("Bulb does not exist");
            }
        }
    }
}

void vToggleTask(void *parameter)
{
    command toggle_command;
    while(1){
        if(xQueueReceive(toggle_queue, (void *)&toggle_command, portMAX_DELAY) == pdTRUE){
            xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
            KASADevice* dev = kasaUtil.GetSmartPlugByIndex(toggle_command.index);
            if(toggle_command.value == 0){
                dev->turnOn();
                menuItems[toggle_command.index].icon = 1;
            } else {
                dev->turnOff();
                menuItems[toggle_command.index].icon = 0;
            }
            if(dev->err_code == 1){
                menuItems[toggle_command.index].icon = 2;
            }
            xTaskNotifyGive(display_task_handle);
            xSemaphoreGive(wifiSemaphore);

        }
    }
}

void vBrightnessTask(void *parameter)
{
    command brightness_command;
    while(1){
        if(xQueueReceive(brightness_queue, (void *)&brightness_command, portMAX_DELAY) == pdTRUE){
            xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
            KASADevice* dev = kasaUtil.GetSmartPlugByIndex(brightness_command.index);
            dev->setBrightness(brightness_command.value);
            xSemaphoreGive(wifiSemaphore);
        }
    }
}

void vColorTask(void *parameter)
{
    command color_command;
    while(1){
        if(xQueueReceive(color_queue, (void *)&color_command, portMAX_DELAY) == pdTRUE){
            xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
            KASADevice* dev = kasaUtil.GetSmartPlugByIndex(color_command.index);
            dev->setColor(color_command.value);
            xSemaphoreGive(wifiSemaphore);
        }
    }
}
