//Task
//  0 = toggle
//  1 = brightness
//  2 = temperature
//  3 = color

//Value
//  int value of brightness or temperature
//  Color will have int values based on what color
    // 0 = Red
    // 1 = Blue
    // 2 = Yellow 
    // 3 = Green
    // 4 = Purple

//Index = int of bulb

#include <cstdint>
typedef struct{
    uint8_t task;
    uint16_t value;
    uint8_t index;
} command;