

#ifndef BSP_BTN_BLE_H__
#define BSP_BTN_BLE_H__

#include <stdint.h>


#define  BUTTON_KEY1_ENABLED    1

// °´¼ü×´Ì¬ÏìÓ¦
typedef enum
{
    USER_BUTTON_NULL,
    USER_BUTTON_CLICK,
    USER_BUTTON_CLICK_3S,
    USER_BUTTON_CLICK_6S,
}enum_button_state;

void Buttons_Init(void);

void Buttons_AonGpioCallback(uint16_t triggered_pin);    
    

uint8_t Buttons_GetPressState(void);

uint16_t Buttons_GetPressTime(void);      
    


#endif /* BSP_BTN_BLE_H__ */
