#include <stdio.h>
#include "Arduino.h"
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "typedef/command.h"
#include "config.h"
#include <ArduinoJson.h>
#include <AiEsp32RotaryEncoder.h>
#include <AiEsp32RotaryEncoderNumberSelector.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else 
static const BaseType_t app_cpu = 1;
#endif

//Pin Set up
#include "pin_def.h"

AiEsp32RotaryEncoder *quick_rotary_encoder = new AiEsp32RotaryEncoder(QUICK_ROTARY_DT, QUICK_ROTARY_CLK, QUICK_ROTARY_SW, VCC_PIN, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoderNumberSelector quick_number_selector = AiEsp32RotaryEncoderNumberSelector();

AiEsp32RotaryEncoder menu_rotary_encoder = AiEsp32RotaryEncoder(MENU_ROTARY_DT, MENU_ROTARY_CLK, MENU_ROTARY_SW, VCC_PIN, ROTARY_ENCODER_STEPS);

//Queue Set up
static const uint8_t max_command_queue_len = 10;
static QueueHandle_t command_queue;

//Debounce and Button Timer
TimerHandle_t xTimer;

//Button Flag
volatile uint8_t button_state_flag;

//Bulb State Flags
uint8_t bulb_state_flag;

//Task Handles
static TaskHandle_t command_read_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;
static TaskHandle_t wifi_task_handle = NULL;

IRAM_ATTR void buttonHandle1(){
    if(button_state_flag & 1){
        return;
    } else {
        button_state_flag |= 1;
        if(xTimerStart(xTimer, 0) != pdPASS){
            return;
        }
    }
}
IRAM_ATTR void buttonHandle2(){
    if(button_state_flag & 1 << 1){
        return;
    } else {
        button_state_flag |= 1 << 1;
        if(xTimerStart(xTimer, 0) != pdPASS){
            return;
        }
    }
}
IRAM_ATTR void buttonHandle3(){
    if(button_state_flag & 1 << 2){
        return;
    } else {
        button_state_flag |= 1 << 2;
        if(xTimerStart(xTimer, 0) != pdPASS){
            return;
        }
    }
}
IRAM_ATTR void buttonHandle4(){
    if(button_state_flag & 1 << 3){
        return;
    } else {
        button_state_flag |= 1 << 3;
        if(xTimerStart(xTimer, 0) != pdPASS){
            return;
        }
    }
}
IRAM_ATTR void buttonHandle5(){
    if(button_state_flag & 1 << 4){
        return;
    } else {
        button_state_flag |= 1 << 4;
        if(xTimerStart(xTimer, 0) != pdPASS){
            return;
        }
    }
}

void vButtonTimerCallback(TimerHandle_t xTimer){
    command new_command;
    for(int i = 0; i < 5; i++){
        uint8_t curr_bitmask = 1 << i;
        if (button_state_flag & curr_bitmask){
            button_state_flag &= ~(1 << i);

            if(bulb_state_flag & curr_bitmask){
                new_command.state = 0;
            } else {
                new_command.state = 1;
            }
            new_command.index = i;
            bulb_state_flag ^= curr_bitmask;
            if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
                Serial.println("Queue Full");
            }
        }
    }
}

void readCommandTask(void *parameter){
    command curr_command;
    while(1){
        if(xQueueReceive(command_queue, (void *)&curr_command, portMAX_DELAY) == pdTRUE){
            Serial.print("Index: ");
            Serial.println(curr_command.index);
            Serial.print("State: ");
            Serial.println(curr_command.state);
        }
    }
}

void displayTask(void *parameter){
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Serial.println("Update Display");
    }
}

void connectToWifi(void *parameter){
    WifiParameters_t *wifi_params = (WifiParameters_t *)parameter;
    WiFi.begin(wifi_params->SSID, wifi_params->PASSWORD);
    while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.println(".");
    }
    Serial.println("Connected to the WiFi network");
    vTaskDelete(NULL);
}


void setup() {
    Serial.begin(115200);
    vTaskDelay(200/portTICK_PERIOD_MS);

    xTaskCreatePinnedToCore(
        connectToWifi, 
        "Wifi Task",
        1024,
        &wifi_params, 
        4,
        &wifi_task_handle,
        app_cpu
    );


    //Command Queue Intialization
    command_queue = xQueueCreate(max_command_queue_len, sizeof(command));

    //Debounce & Button Timer 
    xTimer = xTimerCreate(
        "Debounce Timer",
        pdMS_TO_TICKS(200),
        pdFALSE,
        (void *)0,
        vButtonTimerCallback
    );
    if(xTimer == NULL){
        Serial.println("Was not able to initialize a timer");
    }
    
    //Command Read Task Initialization
    xTaskCreatePinnedToCore(
        readCommandTask, 
        "Command Task",
        2048,
        NULL, 
        2,
        &command_read_task_handle,
        app_cpu
    );

    xTaskCreatePinnedToCore(
        displayTask,
        "Display Task",
        1048,
        NULL,
        1,
        &display_task_handle,
        app_cpu
    );

    //Pin initialization for buttons
    pinMode(SW_1_PIN, INPUT_PULLDOWN);
    pinMode(SW_2_PIN, INPUT_PULLDOWN);
    pinMode(SW_3_PIN, INPUT_PULLDOWN);
    pinMode(SW_4_PIN, INPUT_PULLDOWN);
    pinMode(SW_5_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(SW_1_PIN), buttonHandle1, RISING);
    attachInterrupt(digitalPinToInterrupt(SW_2_PIN), buttonHandle2, RISING);
    attachInterrupt(digitalPinToInterrupt(SW_3_PIN), buttonHandle3, RISING);
    attachInterrupt(digitalPinToInterrupt(SW_4_PIN), buttonHandle4, RISING);
    attachInterrupt(digitalPinToInterrupt(SW_5_PIN), buttonHandle5, RISING);
}

void loop(){

}