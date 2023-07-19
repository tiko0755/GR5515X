/**
 *****************************************************************************************
 *
 * @file maxeye_ble_cli.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"
#include "app_rtc.h"

#include "gr55xx_ll_i2c.h"
#include "gr55xx_nvds.h"

#include "maxeye_io_it.h"
#include "maxeye_gpio.h"


#include "lis2dh12.h"
#include "maxeye_ra9520.h"
#include "maxeye_cw221x.h"
#include "maxeye_mp2662.h"
#include "maxeye_ft3308.h"


#include "maxeye_wlc.h"
#include "maxeye_touch.h"
#include "maxeye_battery.h"
#include "maxeye_sensor.h"
#include "maxeye_mcu_stylus.h"

#include "maxeye_fatfs.h"
#include "maxeye_nvds.h"

#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_dfu.h"
#include "maxeye_notify.h"
#include "maxeye_private_services.h"

#include "maxeye_nvds.h"


#include "maxeye_sleep.h"
#include "maxeye_product_test.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define MSGKEY_SIZE    3



#define MAXEYE_FATFS_TEST  "test.txt"
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t logIndex=0;

const uint8_t msgkey[MSGKEY_SIZE]={0x0C,0x22,0x4E};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool fgBleLog=false;
bool fgDevIntBLELog=false;


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t get_xorcheck(uint8_t *buf, uint8_t len) 
{ 
    uint8_t i = 0; 
    uint8_t checkxor = 0; 

    for (i = 0; i < len; i++) 
    { 
        checkxor = checkxor^buf[i]; 
    } 
    return ~checkxor; 
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint16_t hByte_swop_lByte(uint16_t wData)
{
    uint16_t wValue;

    wValue=(wData>>8)|(wData<<8);

    return wValue;
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_ble_log(char *pData)
{
    char logstr[100];
    
    if(bleConnStatus<BLE_CONNECTED)
    {
        fgBleLog=false;
        return;
    }

    if(!fgBleLog)
    {
        return;
    }

    if(strlen(pData)>97)
    {
        return;
    }

    logIndex++;
    sprintf(logstr,"%02x %s",logIndex,pData);
    maxeye_srvc2_char4_notify(0,(uint8_t *)logstr,strlen(logstr));
}



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_ble_int_log(char *pData)
{
    char logstr[100];
    
    if(bleConnStatus<BLE_CONNECTED)
    {
        fgDevIntBLELog=false;
        return;
    }

    if(!fgDevIntBLELog)
    {
        return;
    }
    
    logIndex++;
    sprintf(logstr,"%02x %s",logIndex,pData);
    maxeye_srvc2_char4_notify(0,(uint8_t *)logstr,strlen(logstr));
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_cli_cb(uint8_t *pData, uint8_t size)
{
    char logstr[256];
    uint16_t ret;

    maxeye_cli_t * maxeye_cli= (maxeye_cli_t *)pData; 

    ret=get_xorcheck(pData,size-1);

    if(pData[size-1]!=ret)
    {
        sprintf(logstr,"check err,%02x",ret);
        maxeye_ble_log(logstr);
        return;
    }

    if(memcmp(msgkey,&maxeye_cli->data[0],MSGKEY_SIZE)!=0)
    {
        sprintf(logstr,"msgkey err");
        maxeye_ble_log(logstr);
        return;
    }
    switch(maxeye_cli->bCmd)
    {
        case MAXEYE_CLI_BATTERY_LOG:
        {
            fgBattParaLog=(maxeye_cli->data[MSGKEY_SIZE]>0)?true:false;
        }
        break;

        case MAXEYE_CLI_TOUCH_POWER_CTRL:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                touch_sensor_powerdown();
                sprintf(logstr,"touch power dw");
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                touch_sensor_powerup();
                sprintf(logstr,"touch power up");
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==2)
            {
                hal_msio_write_pin(TOUCH_CE_PIN,MSIO_PIN_RESET);;
                sprintf(logstr,"touch ce io rst");
            }    
            else if(maxeye_cli->data[MSGKEY_SIZE]==3)
            {
                hal_msio_write_pin(TOUCH_CE_PIN,MSIO_PIN_SET);;
                sprintf(logstr,"touch ce io set");
            }   
            else
            {
                break;
            }

            maxeye_ble_log(logstr);

        }
        break;

        case MAXEYE_CLI_SHIP_MODE:
        {
            ship_mode_enable();
        }
        break;

        case MAXEYE_CLI_PENCIL_SLEEP:
        {
            maxeye_pencil_to_sleep();
        }
        break;

        case MAXEYE_CLI_TOUCH_UPGRADE:
        {
            touch_upgrade_event_register(); 
        }
        break;

        case MAXEYE_CLI_DEVICE_STATUS:
        {
            sprintf(logstr,"Ch:%d,Mt:%d,Gs:%d,Mc:%d,Wl:%d,Th:%d",battChargeStatus,battMeterStatus,
            gSensorStatus,mcuStatus,wlcStatus,touchStatus);
            maxeye_ble_log(logstr);
        }
        break;

        case MAXEYE_CLI_GET_BATT_PARA:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                batt_meter_event_stop();
                sprintf(logstr,"meter event stop");
                maxeye_ble_log(logstr);
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                batt_meter_event_start(50);
            }
        }
        break;


        case MAXEYE_CLI_STYLUS_CONTROL:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                maxeye_stylus_sleep();
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                maxeye_stylus_wakeup();
            }
        }
        break;


        case MAXEYE_CLI_FIRMWARE_KEY:
        {
            uint8_t Databuff[16]={0};

            
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                sys_device_uid_get(Databuff);
                sprintf(logstr,"uid:");
                for(uint8_t i=0;i<sizeof(Databuff);i++)
                {
                    sprintf(&logstr[strlen(logstr)],"%02x ",Databuff[i]);
                }         
                maxeye_ble_log(logstr);
            }
        }
        break;


        case MAXEYE_CLI_SLEEP_DELAY:
        {
            uint32_t wDelay;

            wDelay=maxeye_cli->data[MSGKEY_SIZE];
            wDelay=wDelay<<8|maxeye_cli->data[MSGKEY_SIZE+1];
            wDelay=wDelay*1000;
            ble_idle_event_start(wDelay);
            sprintf(logstr,"delay sleep %dms",wDelay);
            maxeye_ble_log(logstr);
        }
        break;


        case MAXEYE_CLI_MCU_UPGRADE:
        {
            mcu_upgrade_event_register();
        }
        break;


        case MAXEYE_CLI_BLE_RESET:
        {
            hal_nvic_system_reset();
        }
        break;

        case MAXEYE_CLI_GPIO_STATUS:
        {
            uint16_t gpio_pin;

            maxeye_cli->data[MSGKEY_SIZE+1]=maxeye_cli->data[MSGKEY_SIZE+1]&0x0F;
            gpio_pin=1<<maxeye_cli->data[MSGKEY_SIZE+1];

            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                ret=hal_gpio_read_pin(GPIO0,gpio_pin);
                sprintf(logstr,"GPIO%d val=%x",maxeye_cli->data[MSGKEY_SIZE+1],ret);

            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                ret=hal_gpio_read_pin(GPIO1,gpio_pin);
                sprintf(logstr,"GPIO%d val=%x",maxeye_cli->data[MSGKEY_SIZE+1]|0x10,ret);
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==2)
            {
                ret=hal_msio_read_pin(gpio_pin);
                sprintf(logstr,"MISO_GPIO%d val=%x",maxeye_cli->data[MSGKEY_SIZE+1],ret);
            }

            else if(maxeye_cli->data[MSGKEY_SIZE]==3)
            {
                ret=hal_aon_gpio_read_pin(gpio_pin);
                sprintf(logstr,"AON_GPIO%d val=%x",maxeye_cli->data[MSGKEY_SIZE+1],ret);
            }

            else
            {
                break;
            }
            maxeye_ble_log(logstr);
        }
        break;

        case MAXEYE_CLI_GET_MCU_VERSION:
        {
            uint8_t bMcuSleep=false;
            uint8_t Databuff[4];
            
            if(mcuStatus!=STYLUS_DEV_WAKEUP)
            {
                bMcuSleep=true;
                maxeye_stylus_wakeup();
                ble_idle_event_start(10000);
            }
            
            if(maxeye_get_mcu_firmware_version(Databuff)==0)
            {
                sprintf(logstr,"mcu status:%d,version:%d.%d.%d.%d",bMcuSleep,Databuff[0],Databuff[1],Databuff[2]
                ,Databuff[3]);
                maxeye_ble_log(logstr);
            }
        }
        break;


        case MAXEYE_CLI_BLE_PARAMETER:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0) 
            {
                uint16_t wInterVal;

                wInterVal=maxeye_cli->data[MSGKEY_SIZE+1];
                wInterVal=wInterVal<<8|maxeye_cli->data[MSGKEY_SIZE+2];
                ble_connection_parameter_set(wInterVal);
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1) 
            {
                uint16_t wMtu;

                ble_gatt_mtu_get(0,&wMtu);
                sprintf(logstr,"wMtu:%d",wMtu);
                maxeye_ble_log(logstr);
            }
        }
        break;

        case MAXEYE_CLI_GET_BOOT_IMG_INFO:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                bootloader_info_get(); //获取boot msg
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                second_boot_info_get(); //获取second boot msg
            }
        }
        break;


        case MAXEYE_CLI_GET_FILM_VERSION:
        {
            uint8_t fmVersion=0;
            
            if(touchStatus!=TOUCH_DEV_POWR_UP)
            {
                sprintf(logstr,"touch status:%d",touchStatus);
            }
            else
            {
                touch_read_firmware_version(&fmVersion); //唤醒用
                if(touch_read_firmware_version(&fmVersion)!=0)
                {
                    fmVersion=0;
                }
                sprintf(logstr,"touch version:%02x",fmVersion);
            }
            maxeye_ble_log(logstr);
        }
        break;


        case MAXEYE_CLI_GET_WLC_CTRL:
        {
            uint16_t chipID=0;

            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                wlc_read_chip_type(&chipID);
                sprintf(logstr,"wlc type:%4x",chipID); 
                maxeye_ble_log(logstr);
            }
        }
        break;   

        case MAXEYE_CLI_CIREL_CTRL:
        {
            uint8_t bValue;

            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                ret=maxeye_read_cirel_reg(&maxeye_cli->data[MSGKEY_SIZE+1],&bValue);
                if(ret==0)
                {
                    sprintf(logstr,"regVal:%02x",bValue); 
                }
                else
                {
                    sprintf(logstr,"read reg err:%d",ret); 
                }
            }

            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                ret=maxeye_write_cirel_reg(&maxeye_cli->data[MSGKEY_SIZE+1],&bValue);
                if(ret==0)
                {
                    sprintf(logstr,"write reg:%02x",bValue); 
                }
                else
                {
                    sprintf(logstr,"write reg err:%d",ret); 
              
                }
     
            }
            else
            {
                break;
            }
            maxeye_ble_log(logstr);
        }
        break;

        case MAXEYE_CLI_MCU_CTRL:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                uint16_t wPressure;

                ret=maxeye_get_pressure(&wPressure);
                if(ret==0)
                {
                    sprintf(logstr,"Pressure:%d",wPressure); 
                }
                else
                {
                    sprintf(logstr,"mcu err:%d",ret); 
                }
            }

            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                uint8_t bStatus;

                ret=maxeye_get_downlink_status(&bStatus);
                if(ret==0)
                {
                    sprintf(logstr,"dwlink:%d",bStatus); 
                }
                else
                {
                   sprintf(logstr,"mcu err:%d",ret); 
                }
            }

            else if(maxeye_cli->data[MSGKEY_SIZE]==2)
            {
                uint8_t bValue;

                ret=maxeye_mcu_reset_cirel(&bValue);
                if(ret==0)
                {
                    sprintf(logstr,"write rest:%d",bValue); 
                }
                else
                {
                    sprintf(logstr,"mcu err:%d",ret); 
                }
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==3)
            {
                uint8_t bValue;

                ret=maxeye_get_decoding_status(&bValue);
                if(ret==0)
                {
                    sprintf(logstr,"decoding:%d",bValue); 
                }
                else
                {
                    sprintf(logstr,"mcu err:%d",ret); 
                }
            }
            else
            {
                break;
            }
            maxeye_ble_log(logstr);
        }
        break;

        case MAXEYE_CLI_I2C_CTRL:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                ll_i2c_disable(I2C0);
                ll_i2c_set_data_tx_hold_time(I2C0,maxeye_cli->data[MSGKEY_SIZE+1]&0x0F);
                ll_i2c_is_enabled(I2C0);
                sprintf(logstr,"set tx hold:%d",maxeye_cli->data[MSGKEY_SIZE+1]&0x0F); 
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                sprintf(logstr,"get tx hold:%d",ll_i2c_get_data_tx_hold_time(I2C0)); 
            }
            else
            {
                break;
            }
            maxeye_ble_log(logstr);  
        }
        break;

        case MAXEYE_CLI_CHARGE_CTRL:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                MP2662_DisableCharge();
                sprintf(logstr,"discharge");
            }

            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                MP2662_EnableCharge();
                sprintf(logstr,"charge en");
            }

            else if(maxeye_cli->data[MSGKEY_SIZE]==2)
            {
                uint16_t wCCVal;

                wCCVal=maxeye_cli->data[MSGKEY_SIZE+1];
                wCCVal=wCCVal<<8|maxeye_cli->data[MSGKEY_SIZE+2];
                MP2662_SetChargeCurrent(wCCVal,maxeye_cli->data[MSGKEY_SIZE+3]);
                sprintf(logstr,"charge cc:%dmA,term:%dmA",wCCVal,maxeye_cli->data[MSGKEY_SIZE+3]);
            }

            else if(maxeye_cli->data[MSGKEY_SIZE]==3)
            {
                uint16_t wVoltag;

                wVoltag=maxeye_cli->data[MSGKEY_SIZE+1];
                wVoltag=wVoltag<<8|maxeye_cli->data[MSGKEY_SIZE+2];
                battery_SetChargeVoltage(wVoltag);
                sprintf(logstr,"charge voltag:%dmV",wVoltag);
            }

            else
            {
                break;
            }
            maxeye_ble_log(logstr); 
        }
        break;

        case MAXEYE_CLI_BOOT_INFO_ERASE:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                sprintf(logstr,"boot info erase:%d",boot_info_erase());
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                sprintf(logstr,"app img info erase:%d",app_img_info_erase());
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==2)
            {
                sprintf(logstr,"dfu img info erase:%d",dfu_img_info_erase());
            }
            else
            {
                break;
            }
            maxeye_ble_log(logstr); 
        }
        break;

        case MAXEYE_CLI_TEST:
        {
  
        }
        break;


        case MAXEYE_CLI_MCU_COMMON_CLI:
        { 
            ret=maxeye_mcu_common_cli(&maxeye_cli->data[MSGKEY_SIZE],maxeye_cli->bLen-3);
            if(ret==0)
            {
                // sprintf(logstr,"dwlink:%d",bStatus); 
                // maxeye_ble_log(logstr); 
                maxeye_srvc2_char4_notify(0,&maxeye_cli->data[MSGKEY_SIZE],maxeye_cli->data[MSGKEY_SIZE+2]);
            }
            else
            {
                sprintf(logstr,"mcu common cli err:%d",ret); 
                maxeye_ble_log(logstr); 
            }
  
        }
        break;

        case MAXEYE_CLI_INT_LOG:
        {
            fgDevIntBLELog=(maxeye_cli->data[MSGKEY_SIZE]>0)?true:false;
        }
        break;

        case MAXEYE_CLI_G_SENSOR_INT_CTL:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                maxeye_g_int_disable();
                ble_idle_event_start(20000);
                sprintf(logstr,"disable g-int"); 
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                maxeye_g_int_enable();
                sprintf(logstr,"enable g-int"); 
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==2)
            {
                sprintf(logstr,"g-int:%d",maxeye_g_int_status()); 
            }
            else
            {
                break;
            }
            maxeye_ble_log(logstr);
        }
        break;

        case MAXEYE_CLI_HANDSHAKE_STATUS:
        {
            sprintf(logstr,"handshake:%x",bHandshakeStatus); 
            maxeye_ble_log(logstr);
        }
        break;


        case MAXEYE_CLI_REQ_AGING_TEST:
        {
            //if(maxeye_cli->data[MSGKEY_SIZE]==0)
            //{
                //sprintf(logstr,"s:%d,cnt:%d,t:%d,f:%d ",wAgingStatus,wAgingCnt,wAging_time_tick,wAging_fail_tick); 
                //maxeye_ble_log(logstr);
                //APP_LOG_INFO("s2:%d,cnt:%d,t:%d,f:%d---ms:%d,ma:%d,mp:%d ",wAgingStatus,wAgingCnt,wAging_time_tick,wAging_fail_tick,wAging_fail_tick_mcu_sleep,wAging_fail_tick_aisc,wAging_fail_tick_P_sensor);
                sprintf(logstr,"s2:%d,cnt:%d,f:%d---ms:%d,ma:%d,mp:%d",wAgingStatus,wAgingCnt,wAging_fail_tick,wAging_fail_tick_mcu_sleep,wAging_fail_tick_aisc,wAging_fail_tick_P_sensor);
                maxeye_ble_log(logstr);
            //}
        }
        break;

        case MAXEYE_CLI_WRITE_PRODUCT_TEST_FLAG:
        {
            if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                maxeye_write_product_test(&maxeye_cli->data[MSGKEY_SIZE+1],1);
                sprintf(logstr,"write product test flag"); 
                maxeye_ble_log(logstr);
            }

            if(maxeye_cli->data[MSGKEY_SIZE]==1)//PCBA测试
            {
                maxeye_write_pcba_test(&maxeye_cli->data[MSGKEY_SIZE+1],1);
                sprintf(logstr,"write pcba test flag"); 
                maxeye_ble_log(logstr);
            }

        }
        break;


        case MAXEYE_CLI_WRITE_SN:
        {
            if(maxeye_write_device_sn(&maxeye_cli->data[MSGKEY_SIZE],maxeye_cli->bLen-4)==NVDS_SUCCESS)
            {
                sprintf(logstr,"write sn ok"); 
            }
            else
            {
                sprintf(logstr,"write sn failed"); 
            }
            maxeye_ble_log(logstr);
            APP_LOG_INFO("CLI_WRITE_SN3:%d-- %d-%d-%d-%d-%d-%d-%d-%d--%d-%d-%d-%d-%d-%d-%d-%d\n",maxeye_cli->bLen,\
            maxeye_cli->data[0],maxeye_cli->data[1],maxeye_cli->data[2],maxeye_cli->data[3],maxeye_cli->data[4],maxeye_cli->data[5],maxeye_cli->data[6],maxeye_cli->data[7],\
            maxeye_cli->data[8],maxeye_cli->data[9],maxeye_cli->data[10],maxeye_cli->data[11],maxeye_cli->data[12],maxeye_cli->data[13],maxeye_cli->data[14],maxeye_cli->data[15]); 
        }
        break;

        default:
        {
            sprintf(logstr,"ble cli invalid");
            maxeye_ble_log(logstr);
        }
        break;
    }


}















