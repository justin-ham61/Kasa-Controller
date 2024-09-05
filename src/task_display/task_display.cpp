#include "task_display.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icon_bitmap.h"
#include <AiEsp32RotaryEncoderNumberSelector.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <typedef/menu_item.h>

extern AiEsp32RotaryEncoderNumberSelector quick_number_selector;
extern AiEsp32RotaryEncoder menu_rotary_encoder;
extern Adafruit_SSD1306 display;
extern int numberOfBulbs;
extern menu_item menuItems[12];

void vMenuDisplayTask(void *parameters)
{
    int currItem;
    int previousItem;
    int nextItem;

    while(1){
        //Wait for a signal from the rotary encoder to update the diplay
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        currItem = menu_rotary_encoder.readEncoder();
        previousItem = currItem - 1;
        if(previousItem < 0){
            previousItem = numberOfBulbs + 6;
        }
        nextItem = currItem + 1;
        if(nextItem >= numberOfBulbs + 7){
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
        display.fillRect(125, (64/(numberOfBulbs + 7)) * currItem, 3, (64/(numberOfBulbs + 7)), WHITE);

        //Display
        display.display();
    }
}

void vBrightnessDisplayTask(void *parameter)
{
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int brightness = quick_number_selector.getValue();
        display.clearDisplay();
        display.setFont();
        display.setCursor(20, 20);
        display.print("Brightness: ");
        display.print(brightness);
        display.print("%");
        int barWidth = map(brightness, 0, 100, 0, 100);
        display.drawRect(14, 40, 100, 10, WHITE); // Draw the outline of the bar
        display.fillRect(14, 40, barWidth, 10, WHITE); // Fill the bar according to currBrightness
        display.display();
    }
}
