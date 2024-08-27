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
#include "icon_bitmap.h"

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
static const uint8_t max_command_queue_len = 5;
static QueueHandle_t command_queue;

static const uint8_t max_toggle_queue_len = 5;
static QueueHandle_t toggle_queue;

static const uint8_t max_brightness_queue_len = 5;
static QueueHandle_t brightness_queue;

static const uint8_t max_menu_display_queue_len = 5;
static QueueHandle_t menu_display_queue;

//Debounce and Button Queue Timers
TimerHandle_t xTimer;
TimerHandle_t xQuickRotaryTimer;

//Button Flag
volatile uint8_t button_state_flag;

//Bulb State Flags
uint8_t bulb_state_flag;

//Task Handles
static TaskHandle_t command_read_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;
static TaskHandle_t wifi_task_handle = NULL;
static TaskHandle_t device_discover_task_handle = NULL;
static TaskHandle_t toggle_bulb_task_handle = NULL;
static TaskHandle_t quick_rotary_task_handle = NULL;
static TaskHandle_t brightness_task_handle = NULL;
static TaskHandle_t menu_rotary_task_handle = NULL;

//Sleep Handles
TimerHandle_t xIdleTimer;
esp_sleep_source_t wakeup_source; 

//Wifi Dependancies
SemaphoreHandle_t wifiSemaphore;

//Bulbs
KASAUtil kasaUtil;
KASASmartBulb* currentBulb;
int numberOfBulbs;

//DeviceMode
uint8_t device_mode = 0b00000001;

//Display config
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);
menu_item menuItems[size + 1];

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
        if(xTimerStartFromISR(xTimer, 0) != pdPASS){
            return;
        }
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
}
IRAM_ATTR void readEncoderISRMenu(){
    menu_rotary_encoder.readEncoder_ISR();
    xTaskNotifyGive(display_task_handle);
    xTimerStartFromISR(xIdleTimer, 0);
}

// Handles buttons based on current context and adds the command to the queue
// Button presses should get translated to toggle or color based on what the current mode is 
void vButtonTimerCallback(TimerHandle_t xTimer){
        command new_command;
        for(int i = 0; i < 5; i++){
            uint8_t curr_bitmask = 1 << i;
            if (button_state_flag & curr_bitmask){
                button_state_flag &= ~(1 << i);
                new_command.index = i;
                new_command.task = 0;
                new_command.value = 0;
                bulb_state_flag ^= curr_bitmask;
                if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
                    Serial.println("Queue Full");
                }
            }
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
    esp_sleep_enable_gpio_wakeup();
    display.clearDisplay();
    display.display();
    vTaskDelay(100/portTICK_PERIOD_MS);
    esp_light_sleep_start();
    vTaskDelay(100/portTICK_PERIOD_MS);
    WiFi.reconnect();
}

void vQuickRotaryCallback(TimerHandle_t xQuickRotaryTimer){
    Serial.println("Rotary Timer Call back");
    command new_command;
    new_command.task = 1;
    new_command.value = quick_number_selector.getValue();
    for(int i = 0; i < numberOfBulbs; i++){
        new_command.index = i;
        if(xQueueSend(command_queue, (void *)&new_command, 10) != pdTRUE){
            Serial.println("Queue is Full");
        }
    }
}

void readCommandTask(void *parameter){
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
                        Serial.println("Color Control");
                        break;
                }
            } else {
                Serial.println("Bulb does not exist");
            }
        }
    }
}

void toggleTask(void *parameter){
    command toggle_command;
    while(1){
        if(xQueueReceive(toggle_queue, (void *)&toggle_command, portMAX_DELAY) == pdTRUE){
            xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
            currentBulb = static_cast<KASASmartBulb*>(kasaUtil.GetSmartPlugByIndex(toggle_command.index));
            if(currentBulb->state == 0){
                currentBulb->turnOn();
                menuItems[toggle_command.index].icon = 1;
            } else {
                currentBulb->turnOff();
                menuItems[toggle_command.index].icon = 0;
            }
            if(currentBulb->err_code == 1){
                menuItems[toggle_command.index].icon = 2;
            }
            xTaskNotifyGive(display_task_handle);
            xSemaphoreGive(wifiSemaphore);
        }
    }
}

void brightnessTask(void *parameter){
    command brightness_command;
    while(1){
        if(xQueueReceive(brightness_queue, (void *)&brightness_command, portMAX_DELAY) == pdTRUE){
            xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
            currentBulb = static_cast<KASASmartBulb*>(kasaUtil.GetSmartPlugByIndex(brightness_command.index));
            currentBulb->setBrightness(brightness_command.value);
            xSemaphoreGive(wifiSemaphore);
        }
    }
}

void menuDisplayTask(void *parameter){
    int currItem;
    int previousItem;
    int nextItem;

    while(1){
        //Wait for a signal from the rotary encoder to update the diplay
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        currItem = menu_rotary_encoder.readEncoder();
        previousItem = currItem - 1;
        if(previousItem < 0){
            previousItem = numberOfBulbs;
        }
        nextItem = currItem + 1;
        if(nextItem >= numberOfBulbs + 1){
            nextItem = 0;
        }

        display.clearDisplay();
        display.drawBitmap(4,2,bitmap_array[menuItems[previousItem].icon],16,16,1);
        display.drawBitmap(4,24,bitmap_array[menuItems[currItem].icon],16,16,1);
        display.drawBitmap(4,46,bitmap_array[menuItems[nextItem].icon],16,16,1);

        display.drawBitmap(0, 22, bitmap_item_sel_background, 128, 21, 1);
        display.drawBitmap(120, 0, bitmap_scrollbar_background, 8, 64, 1);

        display.setTextSize(1);
        display.setTextColor(WHITE);

        //Previous Item
        display.setFont(&FreeSans9pt7b);
        display.setCursor(26, 15);
        display.print(menuItems[previousItem].name);

        //Current Item
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(26, 37);
        display.print(menuItems[currItem].name);

        //Next Item
        display.setFont(&FreeSans9pt7b);
        display.setCursor(26, 59);
        display.print(menuItems[nextItem].name);

        //Scroll position box
        display.fillRect(125, (64/(numberOfBulbs + 1)) * currItem, 3, (64/(numberOfBulbs + 1)), WHITE);

        //Display
        display.display();
    }
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

void addDevices(void *parameter){
    while(1){
        xSemaphoreTake(wifiSemaphore, portMAX_DELAY);
        numberOfBulbs = kasaUtil.ScanDevicesAndAdd(1000, aliases, size);
        for(int i = 0; i < numberOfBulbs; i++){
            Serial.println(kasaUtil.GetSmartPlugByIndex(i)->alias);
            menuItems[i] = {kasaUtil.GetSmartPlugByIndex(i)->alias, 1};
        }
        menuItems[numberOfBulbs] = {"Reset", 3};
        xTaskNotifyGive(display_task_handle);
        xSemaphoreGive(wifiSemaphore);
        vTaskSuspend(NULL);
    }
}

void setup() {
    Serial.begin(115200);
    vTaskDelay(200/portTICK_PERIOD_MS);

    // ------------------------------------ WIFI TASK INIT ------------------------------------------ //
    wifiSemaphore = xSemaphoreCreateBinary();
    if(wifiSemaphore == NULL){
        Serial.println("Unable to create wifi semaphore");
        while(1);
    } else {
        xSemaphoreGive(wifiSemaphore);
    }

    xTaskCreatePinnedToCore(
        connectToWifi, 
        "Wifi Task",
        4096,
        &wifi_params, 
        4,
        &wifi_task_handle,
        app_cpu
    );

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setFont();
    display.display(); // Display the message
    display.clearDisplay();
    // ------------------------------------ KASA BULB INIT ------------------------------------------ //
    xTaskCreatePinnedToCore(
        addDevices,
        "Device Task",
        10000,
        NULL,
        3,
        &device_discover_task_handle,
        app_cpu
    );

    //Command Queue Intialization
    command_queue = xQueueCreate(max_command_queue_len, sizeof(command));
    toggle_queue = xQueueCreate(max_toggle_queue_len, sizeof(command));
    brightness_queue = xQueueCreate(max_brightness_queue_len, sizeof(command));
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
    
    // ------------------------------------ RUNTIME TASK INIT ------------------------------------------ //
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
        toggleTask,
        "Toggle Task", 
        2048, 
        NULL,
        2,
        &toggle_bulb_task_handle,
        app_cpu
    );
    xTaskCreatePinnedToCore(
        brightnessTask,
        "Brightness Task",
        2048,
        NULL,
        2,
        &brightness_task_handle,
        app_cpu
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
    menu_rotary_encoder.setBoundaries(0,size,true);
    menu_rotary_encoder.disableAcceleration();

    xTaskCreatePinnedToCore(
        menuDisplayTask,
        "Display Task",
        4096,
        NULL,
        1,
        &display_task_handle,
        app_cpu
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

    //Pin set up for sleep wake up
    gpio_wakeup_enable(GPIO_NUM_25, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(GPIO_NUM_26, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(GPIO_NUM_27, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(GPIO_NUM_14, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(GPIO_NUM_12, GPIO_INTR_HIGH_LEVEL);
    xTimerStart(xIdleTimer, portMAX_DELAY);
}

void loop(){

}