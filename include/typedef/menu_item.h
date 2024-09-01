#ifndef MENU_ITEM
#define MENU_ITEM

//type
    //bulb = 0
    //presets = 1
    //reset = 2
    //save = 3
typedef struct{
    const char* name;
    int icon;
    int type;
} menu_item;

#endif