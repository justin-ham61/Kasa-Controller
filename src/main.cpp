#include <stdio.h>
#include "Arduino.h"
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "config.h"
#include "esp_sleep.h"
#include "typedef/command.h"
#include "typedef/menu_item.h"

#include "task_device_setup/task_device_setup.h"
#include "task_update_flash/task_update_flash.h"
#include "task_display/task_display.h"
#include "task_command/task_command.h"

// ------------------- Libraries ------------------------ //
#include <ArduinoJson.h>
#include <AiEsp32RotaryEncoder.h>
#include <AiEsp32RotaryEncoderNumberSelector.h>
#include <KasaSmartPlug.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <EEPROM.h>

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
QueueHandle_t command_queue;

static const uint8_t max_toggle_queue_len = 5;
QueueHandle_t toggle_queue;

static const uint8_t max_brightness_queue_len = 5;
QueueHandle_t brightness_queue;

static const uint8_t max_color_queue_len = 5;
QueueHandle_t color_queue;


static const uint8_t max_menu_display_queue_len = 5;
static QueueHandle_t menu_display_queue;

//Debounce and Button Queue Timers
TimerHandle_t xTimer;
TimerHandle_t xQuickRotaryTimer;
TimerHandle_t xModeSwitchTimer;
TimerHandle_t xMenuRotarySwitchTimer;
TimerHandle_t xQuickRotarySwitchTimer;

//Button Flag
volatile uint8_t button_state_flag;


//Task Handles
static TaskHandle_t command_read_task_handle = NULL;
TaskHandle_t display_task_handle = NULL;
static TaskHandle_t wifi_task_handle = NULL;
static TaskHandle_t device_discover_task_handle = NULL;
static TaskHandle_t toggle_bulb_task_handle = NULL;
static TaskHandle_t quick_rotary_task_handle = NULL;
static TaskHandle_t brightness_task_handle = NULL;
static TaskHandle_t menu_rotary_task_handle = NULL;
static TaskHandle_t brightness_display_task_handle = NULL;
static TaskHandle_t color_task_handle = NULL;
static TaskHandle_t update_flash_task_handle = NULL;

//Sleep Handles
TimerHandle_t xIdleTimer;
esp_sleep_source_t wakeup_source; 

//Wifi Dependancies
SemaphoreHandle_t wifiSemaphore;

//Bulbs
KASAUtil kasaUtil;

int numberOfBulbs;
bool all_state = false;
int size = sizeof(aliases)/sizeof(aliases[0]);


//DeviceMode
uint8_t device_mode = 0b00000001;

//Display config
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);
menu_item menuItems[12];


//Task Params
int all_brightness = 0;

IRAM_ATTR void buttonHandle1(){
    if(button_state_flag & 1){
        return;
    } else {
        button_state_flag |= 1;
        xTimerStart(xTimer, 0);
    }
}
IRAM_ATTR void buttonHandle2(){
    if(button_state_flag & 2){
        return;
    } else {
        button_state_flag |= 2;
        xTimerStart(xTimer, 0);
    }
}
IRAM_ATTR void buttonHandle3(){
    if(button_state_flag & 4){
        return;
    } else {
        button_state_flag |= 4;
        if(xTimerStartFromISR(xTimer, 0) != pdPASS){
            return;
        }
    }
}
IRAM_ATTR void buttonHandle4(){
    if(button_state_flag & 8){
        return;
    } else {
        button_state_flag |= 8;
        if(xTimerStartFromISR(xTimer, 0) != pdPASS){
            return;
        }
    }
}
IRAM_ATTR void buttonHandle5(){
    if(button_state_flag & 16){
        return;
    } else {
        button_state_flag |= 16;
        if(xTimerStartFromISR(xTimer, 0) != pdPASS){
            return;
        }
    }
}
IRAM_ATTR void readEncoderISRQuick(){
    quick_rotary_encoder->readEncoder_ISR();
    xTimerStartFromISR(xQuickRotaryTimer, 0);
    xTimerStartFromISR(xIdleTimer, 0);
    xTimerStartFromISR(xModeSwitchTimer, 0);
    xTaskNotifyGive(brightness_display_task_handle);
}

IRAM_ATTR void readEncoderISRMenu(){
    menu_rotary_encoder.readEncoder_ISR();
    if(device_mode & 1){
        xTaskNotifyGive(display_task_handle);
        xTimerStartFromISR(xIdleTimer, 0);
    }

}

IRAM_ATTR void menuRotaryHandle(){
    xTimerStart(xMenuRotarySwitchTimer, 0);
    xTimerStartFromISR(xIdleTimer, 0);
}

IRAM_ATTR void quickRotaryHandle(){
    xTimerStart(xQuickRotarySwitchTimer, 0);
    xTimerStartFromISR(xIdleTimer, 0);
}

// Handles buttons based on current context and adds the command to the queue
// Button presses should get translated to toggle or color based on what the current mode is 
void vButtonTimerCallback(TimerHandle_t xTimer){
    command new_command; 
    if(device_mode & 1){
        for(int i = 0; i < numberOfBulbs; i++){
            uint8_t curr_bitmask = 1 << i;
            if (button_state_flag & curr_bitmask){
                button_state_flag &= ~(1 << i);
                KASADevice* dev = kasaUtil.GetSmartPlugByIndex(i);
                new_command.value = dev->state;
                new_command.index = i;
                new_command.task = 0;
                if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
                    Serial.println("Queue Full");
                }
            }
        }
    } else if (device_mode & 4){
        new_command.task = 3;
        new_command.index = menu_rotary_encoder.readEncoder();
        for(int i = 0; i < 5; i++){
            uint8_t curr_bitmask = 1 << i;
            if(button_state_flag & curr_bitmask){
                button_state_flag &= ~curr_bitmask;
                new_command.value = i;
            }
        }
        if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
            Serial.println("Queue Full");
        }
        //Insert code for sending color change commands
    }
    if(xTimerStart(xIdleTimer, portMAX_DELAY) != pdTRUE){
        Serial.println("Failed to start idle timer");
    }
}

void vIdleTimerCallback(TimerHandle_t xIdleTimer){
    Serial.println("Waiting for semaphore for idle");
    //Awaits tasks that depend on wifi to finish before going to sleep
    xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
    Serial.println("Disconnected WiFi and going to sleep");
    WiFi.disconnect();

    gpio_wakeup_enable(GPIO_NUM_12, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    display.clearDisplay();
    display.display();


    vTaskDelay(100/portTICK_PERIOD_MS);
    esp_light_sleep_start();
    vTaskDelay(200/portTICK_PERIOD_MS);

    xTimerStart(xIdleTimer, 0);
    gpio_wakeup_disable(GPIO_NUM_12);
    xTaskNotifyGive(display_task_handle);
    WiFi.reconnect();
}

void vQuickRotaryCallback(TimerHandle_t xQuickRotaryTimer){
    command new_command;
    new_command.task = 1;
    new_command.value = quick_number_selector.getValue();
    if(device_mode & 1){
        for(int i = 0; i < numberOfBulbs; i++){
            new_command.index = i;
            if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
                Serial.println("Queue is Full");
            }
        }
    } else if (device_mode & 4){
        Serial.println("Sending");
        new_command.index = menu_rotary_encoder.readEncoder();
        if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
            Serial.println("Queue is Full");
        }
    }
}

void vModeSwitchCallback(TimerHandle_t xModeSwitchTimer){
    device_mode = 1;
    xTaskNotifyGive(display_task_handle);
}

void vMenuSwitchCallback(TimerHandle_t xMenuRotarySwitchTimer){
    int menu_index = menu_rotary_encoder.readEncoder();
    int curr_type = menuItems[menu_index].type;

    //Handles if button press is trying to select a bulb
    if(curr_type == 0){
        device_mode = 4;
        xTaskNotifyGive(brightness_display_task_handle);
        xTimerStart(xModeSwitchTimer, 0);
    //Handles if button press is activating a preset
    } else if (curr_type == 1){
        command color_command;
        color_command.task = 3;
        color_command.value = menu_index - numberOfBulbs;
        for(int i = 0; i < numberOfBulbs; i++){
            color_command.index = i;
            if(xQueueSend(command_queue, (void *)&color_command, 10) != pdTRUE){
                Serial.println("Queue Full");
            }
        }
    //Handles if button press is resetting device
    } else if (curr_type == 2){
        Serial.println("reset");
        esp_restart();
    //Handles if button press is saving bulb configuration
    } else if (curr_type == 3){
        Serial.println("Save");
        xTaskNotifyGive(update_flash_task_handle);
    }

    xTimerStart(xIdleTimer, 0);
}

void vQuickSwitchCallback(TimerHandle_t xQuickRotarySwitchTimer){
    command new_command;
    new_command.task = 0;
    new_command.value = all_state;
    for(int i = 0; i < numberOfBulbs; i++){
        new_command.index = i;
        if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
            Serial.println("Queue is Full");
        }
    }
    all_state = !all_state;
}

void WiFiEvent(WiFiEvent_t event){
    switch(event){
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.println("WiFi Connected");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
            xSemaphoreGive(wifiSemaphore);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xSemaphoreTake(wifiSemaphore, 0);
            Serial.println("Attempting to reconnect");
            WiFi.reconnect();
            break;
        default:
            break;
    }
}

void connectToWifi(void *parameter){
    while(1){
        WiFi.disconnect(true);

        xSemaphoreTake(wifiSemaphore, portMAX_DELAY);

        vTaskDelay(500/portTICK_PERIOD_MS);

        WiFi.onEvent(WiFiEvent);

        WifiParameters_t *wifi_params = (WifiParameters_t *)parameter;
        WiFi.begin(wifi_params->SSID, wifi_params->PASSWORD);
        while(WiFi.status() != WL_CONNECTED){
            delay(500);
            Serial.println(".");
        }
        
        Serial.println("Connected to the WiFi network");
        vTaskDelete(NULL); 
    }
}


void setup() {
    Serial.begin(115200);
    vTaskDelay(200/portTICK_PERIOD_MS);

    // ------------------------------------ WIFI TASK INIT ------------------------------------------ //
    if (!EEPROM.begin(256)) {
        Serial.println("Failed to initialise EEPROM");
        return;
    }


    wifiSemaphore = xSemaphoreCreateBinary();
    if(wifiSemaphore == NULL){
        Serial.println("Unable to create wifi semaphore");
        while(1);
    } else {
        xSemaphoreGive(wifiSemaphore);
    }

    xTaskCreate(
        connectToWifi, 
        "Wifi Task",
        4096,
        &wifi_params, 
        4,
        &wifi_task_handle
    );

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setFont();
    display.display(); // Display the message
    display.clearDisplay();
    // ------------------------------------ KASA BULB INIT ------------------------------------------ //
    xTaskCreate(
        vAddDeviceTask,
        "Device Task",
        10000,
        NULL,
        3,
        &device_discover_task_handle
    );

    //Command Queue Intialization
    command_queue = xQueueCreate(max_command_queue_len, sizeof(command));
    toggle_queue = xQueueCreate(max_toggle_queue_len, sizeof(command));
    brightness_queue = xQueueCreate(max_brightness_queue_len, sizeof(command));
    color_queue = xQueueCreate(max_color_queue_len, sizeof(command));
    menu_display_queue = xQueueCreate(max_menu_display_queue_len, sizeof(int));


    // ------------------------------------ TIMER INIT ------------------------------------------ //
    //Debounce & Button Timer 
    xTimer = xTimerCreate(
        "Debounce Timer",
        pdMS_TO_TICKS(200),
        pdFALSE,
        (void *)0,
        vButtonTimerCallback
    );
    if(xTimer == NULL){
        Serial.println("Was not able to initialize a debounce timer");
    }

    //Sleep timer
    xIdleTimer = xTimerCreate(
        "Idle Timer",
        pdMS_TO_TICKS(15000),
        pdFALSE,
        (void *)0,
        vIdleTimerCallback
    );
    if(xIdleTimer == NULL){
        Serial.println("Was not able to initialize idle timer");
    }

    //Rotary Timer
    xQuickRotaryTimer = xTimerCreate(
        "Quick Rotary Timer",
        pdMS_TO_TICKS(1000),
        pdFALSE,
        (void *)0,
        vQuickRotaryCallback
    );
    if(xQuickRotaryTimer == NULL){
        Serial.println("Was not able to initialize quick rotary timer");
    }

    //Display Mode Switch Timer
    xModeSwitchTimer = xTimerCreate(
        "Mode Switch Timer",
        pdMS_TO_TICKS(3000),
        pdFALSE,
        (void *)0,
        vModeSwitchCallback
    );

    xMenuRotarySwitchTimer = xTimerCreate(
        "Menu Switch Timer",
        pdMS_TO_TICKS(50),
        pdFALSE,
        (void *)0,
        vMenuSwitchCallback
    );

    xQuickRotarySwitchTimer = xTimerCreate(
    "Quick Switch Timer",
        pdMS_TO_TICKS(50),
        pdFALSE,
        (void *)0,
        vQuickSwitchCallback
    );

    
    // ------------------------------------ RUNTIME TASK INIT ------------------------------------------ //
    //Command Read Task Initialization
    xTaskCreate(
        vReadCommandTask, 
        "Command Task",
        2048,
        NULL, 
        2,
        &command_read_task_handle
    );

    xTaskCreate(
        vToggleTask,
        "Toggle Task", 
        2048, 
        NULL,
        2,
        &toggle_bulb_task_handle
    );

    xTaskCreate(
        vBrightnessTask,
        "Brightness Task",
        2048,
        NULL,
        2,
        &brightness_task_handle
    );

    xTaskCreate(
        vColorTask,
        "Color Task",
        2048,
        NULL,
        2,
        &color_task_handle
    );
    
    //Rotary Encoder Set up
    quick_rotary_encoder->begin();
    quick_rotary_encoder->setup(readEncoderISRQuick);
    quick_number_selector.attachEncoder(quick_rotary_encoder);
    quick_number_selector.setRange(0, 99, 5, false, 0);
    quick_number_selector.setValue(50);
    quick_rotary_encoder->disableAcceleration();

    menu_rotary_encoder.begin();
    menu_rotary_encoder.setup(readEncoderISRMenu);
    menu_rotary_encoder.setBoundaries(0,size + 6,true);
    menu_rotary_encoder.disableAcceleration();

    xTaskCreate(
        vMenuDisplayTask,
        "Menu Display Task",
        4096,
        NULL,
        1,
        &display_task_handle
    );

    xTaskCreate(
        vBrightnessDisplayTask,
        "Brightness Display Task",
        4096,
        &all_brightness,
        1,
        &brightness_display_task_handle
    );

    xTaskCreate(
        vTaskUpdateFlash,
        "Update Flash",
        4096,
        NULL, 
        1, 
        &update_flash_task_handle
    );

    // ------------------------------------ GPIO INIT ------------------------------------------ //
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

    pinMode(MENU_ROTARY_SW, INPUT_PULLDOWN);
    pinMode(QUICK_ROTARY_SW, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(QUICK_ROTARY_SW), quickRotaryHandle, RISING);
    attachInterrupt(digitalPinToInterrupt(MENU_ROTARY_SW), menuRotaryHandle, RISING);


    //Pin set up for sleep wake up
    xTimerStart(xIdleTimer, portMAX_DELAY);
}

void loop(){

}