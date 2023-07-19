
#include "gr55xx_hal.h"

#pragma arm section code = "RAM_CODE"
//#define BOARD_SK
#define BOARD_WT
//#define BOARD_DAU
//#define BOARD_EVK_Q

#ifdef BOARD_WT
void point_init(void)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    hal_gpio_init(GPIO0, &gpio_config);
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = GPIO_PIN_10|GPIO_PIN_14;
    hal_gpio_init(GPIO1, &gpio_config);
    aon_gpio_init_t aon_gpio_init = AON_GPIO_DEFAULT_CONFIG;
    aon_gpio_init.pin  = AON_GPIO_PIN_7;
    aon_gpio_init.mode = AON_GPIO_MODE_OUTPUT;
    hal_aon_gpio_init(&aon_gpio_init);
}

void point_at(int idx)
{
     switch (idx)
     {
         case 0:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_14);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_14);
               break;
         case 1:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_10);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_10);
               break;        
         case 3:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_RESET);
               break; 				 
     }
}
#endif

#ifdef BOARD_SK
void point_init(void)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    hal_gpio_init(GPIO0, &gpio_config);
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_1|GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_8;
    hal_gpio_init(GPIO1, &gpio_config);
    aon_gpio_init_t aon_gpio_init = AON_GPIO_DEFAULT_CONFIG;
    aon_gpio_init.pin  = AON_GPIO_PIN_2 | AON_GPIO_PIN_3 | AON_GPIO_PIN_4 | AON_GPIO_PIN_5;
    aon_gpio_init.mode = AON_GPIO_MODE_OUTPUT;
    hal_aon_gpio_init(&aon_gpio_init);
}

void point_at(int idx)
{
     switch (idx)
     {
         case 0:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_15);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_15);
               break;
         case 1:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_14);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_14);
               break;
         case 2:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_2, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_2, AON_GPIO_PIN_RESET);
               break;
         case 3:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_3, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_3, AON_GPIO_PIN_RESET);
               break;
         case 4:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_4, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_4, AON_GPIO_PIN_RESET);
               break;
         case 5:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_5, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_5, AON_GPIO_PIN_RESET);
               break;
         case 6:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_10);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_10);
               break;
         case 7:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_11);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_11);
               break;
         case 8:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_1);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_1);
               break;
         case 9:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_9);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_9);
               break;
         case 10:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_0);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_0);
               break;
         case 11:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_8);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_8);
               break;
         case 0x10:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_2, AON_GPIO_PIN_RESET);
               break;    
         case 0x12:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_3, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_3, AON_GPIO_PIN_RESET);
               break; 
         
     }
}
#endif

#ifdef BOARD_DAU
void point_init(void)
{
    aon_gpio_init_t aon_gpio_init = AON_GPIO_DEFAULT_CONFIG;
    aon_gpio_init.pin  = AON_GPIO_PIN_6 | AON_GPIO_PIN_7;
    aon_gpio_init.mode = AON_GPIO_MODE_OUTPUT;
    hal_aon_gpio_init(&aon_gpio_init);
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = GPIO_PIN_11;
    hal_gpio_init(GPIO0, &gpio_config);
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = GPIO_PIN_10 | GPIO_PIN_11 |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    hal_gpio_init(GPIO1, &gpio_config);
}

void point_at(int idx)
{
     switch (idx)
     {
         case 0x0:
               ll_gpio_set_output_pin(GPIO0, GPIO_PIN_11);
               ll_gpio_reset_output_pin(GPIO0, GPIO_PIN_11);
               break;
         case 0x1:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_10);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_10);
               break;
         case 0x2:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_11);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_11);
               break;
         case 0x3:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_12);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_12);
               break;
         case 0x4:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_13);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_13); 
               break;
         case 0x5:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_14);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_14);
               break;
         case 0x6:
               ll_gpio_set_output_pin(GPIO1, GPIO_PIN_15);
               ll_gpio_reset_output_pin(GPIO1, GPIO_PIN_15);
               break;
         case 0x7:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_6, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_6, AON_GPIO_PIN_RESET);
               break;
         case 0x8:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_RESET);
         case 0x9:
               ll_gpio_set_output_pin(GPIO0, GPIO_PIN_8);
               ll_gpio_reset_output_pin(GPIO0, GPIO_PIN_8);
         case 10:
               ll_gpio_set_output_pin(GPIO0, GPIO_PIN_9);
               ll_gpio_reset_output_pin(GPIO0, GPIO_PIN_9);
               break;
     }
}
#endif



#ifdef BOARD_EVK_Q
void point_init(void)
{
    aon_gpio_init_t aon_gpio_init = AON_GPIO_DEFAULT_CONFIG;
    aon_gpio_init.pin  = AON_GPIO_PIN_0 | AON_GPIO_PIN_1 | AON_GPIO_PIN_2 | AON_GPIO_PIN_3 | AON_GPIO_PIN_4 | AON_GPIO_PIN_5 | AON_GPIO_PIN_6 | AON_GPIO_PIN_7;
    aon_gpio_init.mode = AON_GPIO_MODE_OUTPUT;
    hal_aon_gpio_init(&aon_gpio_init);
//    
//    
//    hal_aon_gpio_write_pin(AON_GPIO_PIN_0, AON_GPIO_PIN_SET);
//    hal_aon_gpio_write_pin(AON_GPIO_PIN_1, AON_GPIO_PIN_SET);
//    hal_aon_gpio_write_pin(AON_GPIO_PIN_2, AON_GPIO_PIN_SET);
//    hal_aon_gpio_write_pin(AON_GPIO_PIN_3, AON_GPIO_PIN_SET);
//    hal_aon_gpio_write_pin(AON_GPIO_PIN_4, AON_GPIO_PIN_SET);
//    hal_aon_gpio_write_pin(AON_GPIO_PIN_5, AON_GPIO_PIN_SET);
}

void point_at(int idx)
{
     switch (idx)
     {
         case 0:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_0, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_0, AON_GPIO_PIN_RESET);
               break;
         case 1:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_1, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_1, AON_GPIO_PIN_RESET);
               break;
         case 2:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_2, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_2, AON_GPIO_PIN_RESET);
               break;
         case 3:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_3, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_3, AON_GPIO_PIN_RESET);
               break;
         case 4:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_4, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_4, AON_GPIO_PIN_RESET);
               break;
         case 5:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_5, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_5, AON_GPIO_PIN_RESET);
               break;
         case 6:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_6, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_6, AON_GPIO_PIN_RESET);
               break;
         case 7:
               hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_SET);
               hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_RESET);
               break;
     }
}
#endif





void p_test()
{
    while (1)
    {
          
         point_at(0);  
         point_at(1); 
         point_at(2); 
         point_at(3); 
         point_at(4); 
         point_at(5); 
         point_at(6);
         point_at(7);
         point_at(8);
        point_at(9);
        point_at(10);
        point_at(11);
 
    }
}


#pragma arm section
