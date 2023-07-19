/**
 *****************************************************************************************
 *
 * @file maxeye_notfiy.c
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
#include "app_timer.h"

#include "gr55xx_nvds.h"

#include "maxeye_cw221x.h"
#include "maxeye_ra9520.h"


#include "maxeye_battery.h"
#include "maxeye_sensor.h"
#include "maxeye_mcu_stylus.h"
#include "maxeye_gpio.h"
#include "maxeye_wdt.h"
#include "maxeye_touch.h"

#include "maxeye_version.h"

#include "maxeye_product_test.h"

#include "maxeye_nvds.h"
#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_sleep.h"
#include "maxeye_notify.h"
#include "maxeye_private_services.h"

#include "app_queue.h"
#include "bas.h"

#include "user_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define HANDSHAKE_PACK_SIZE            36//21 //过短影响到量产测试
#define SRVC_PACK_SIZE                 36//21 //过短影响到量产测试

#define HANDSHAKE_ACK_DELAY            
#define HANDSHAKE_SRVC_ACK_DELAY       2  //ms


#define HANDSHAKE_REQ_CHARGE_STATUS    1
#define HANDSHAKE_REQ_BATT_CAP         2
#define HANDSHAKE_REQ_FW_VERSION       4 
#define HANDSHAKE_REQ_HW_VERSION       8
#define HANDSHAKE_REQ_MODE_ID          0x10
#define HANDSHAKE_REQ_SN               0x20

#define BLE_MSG_QUEUE_ITEM_SIZE        2  //数值偏大会导致异常重启
#define BLE_MSG_QUEUE_LENGTH           4


typedef struct
{
    uint8_t   msgHead;   
    uint8_t   msgCmd;
    uint8_t   msgRev;
    uint8_t   msgLen; 
    uint8_t   msgbuff[SRVC_PACK_SIZE-4];  

} ble_msg_t;



/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t maxeye_srvc_event_id;

//static uint16_t wMasterCmd;


app_queue_t ble_msg_QueueHandle;

static uint8_t  bleMsgQueuebuff[BLE_MSG_QUEUE_LENGTH*BLE_MSG_QUEUE_ITEM_SIZE];   



static bool fgDbClickTest=false;
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t bHandshakeStatus;  //握手状态
bool fgBattCapNtfEnable=false;

uint8_t battLevel=BATT_CAP_LEVEL_INVAILD;
uint8_t doubleClick=0;


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
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
void maxeye_msg_queue_init(void) 
{
    sdk_err_t ret;
    ret=app_queue_init(&ble_msg_QueueHandle,bleMsgQueuebuff,BLE_MSG_QUEUE_LENGTH,BLE_MSG_QUEUE_ITEM_SIZE);
    if(ret!=SDK_SUCCESS)
    {
        logX("creat ble msg queue failed");
    }
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
void maxeye_msg_queue_clean(void) 
{
    app_queue_clean(&ble_msg_QueueHandle);
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
sdk_err_t maxeye_msg_queue_push(uint16_t msgId) 
{
    return app_queue_push(&ble_msg_QueueHandle,(uint8_t*)&msgId);
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
void srvc1_refresh_battery_paramenter(uint8_t bValue)  //刷新电池参数
{
    cw_update_data();
    if(bValue)
    {
        battery_charging_strategy();
    }
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
void srvc1_battery_charge_notify(void)
{
	logX("<%s >", __func__);
    uint8_t notfbuff[HANDSHAKE_PACK_SIZE]={0};
    sdk_err_t  error_code;

    notfbuff[0]=0x2C;
    notfbuff[1]=0x01;

    if(battChargeStatus==BATT_CHARGEING_DISABLE)
    {
        notfbuff[2]=1; //0x7F;
        notfbuff[3]=1; //4;
        notfbuff[4]=0; //0-未充电，1-充电中，2-已充满，3-充电异常
    }
    else
    {
        notfbuff[2]=1;
        notfbuff[3]=1;
        notfbuff[4]=1;
    }
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__);
    }
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
void srvc1_battery_capacity_notify(void)
{
    logX("<%s >", __func__); 
    uint8_t notfbuff[HANDSHAKE_PACK_SIZE]={0};
    sdk_err_t  error_code;

    fgBattCapNtfEnable=true;

    if(cw_update_data() < 0){
        logX("<%s +err@cw_update_data >", __func__); 
    }

    notfbuff[0]=0x2C;
    notfbuff[1]=0x08;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    notfbuff[4]=cw_bat.capacity;
    battLevel=cw_bat.capacity;
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
        battLevel=BATT_CAP_LEVEL_INVAILD; 
    }
    else
    {
     	logX("</%s cap:%d temp:%d vol:%d>", __func__, cw_bat.capacity,cw_bat.temp,cw_bat.voltage); 
    }
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
void srvc1_fw_version_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[HANDSHAKE_PACK_SIZE]={0};
    const uint8_t fwVer[]={CUSTOMER_FIRMWARE_REV_STR};

    notfbuff[0]=0x2C;
    notfbuff[1]=0x06;
    notfbuff[2]=0x01;
    notfbuff[3]=0x05;

    memcpy(&notfbuff[4],fwVer,sizeof(fwVer));

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }
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
void srvc1_hw_version_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    char *pStr=CUSTOMER_HARDWARE_REV_STR;
    uint8_t notfbuff[HANDSHAKE_PACK_SIZE]={0};

    notfbuff[0]=0x2C;
    notfbuff[1]=0x25;
    notfbuff[2]=0x01;
    notfbuff[3]=0x03;
    memcpy(&notfbuff[4],pStr,strlen(pStr));
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_model_id_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    char *pStr=PENCIL_MODEL_NUM_STR;
    uint8_t notfbuff[HANDSHAKE_PACK_SIZE]={0};

    notfbuff[0]=0x2C; 
    notfbuff[1]=0x26;
    notfbuff[2]=0x01;
    notfbuff[3]=0x0B;

    memcpy(&notfbuff[4],pStr,strlen(pStr));
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_oppo_sn_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    char snStr[32];
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    uint16_t bLen=sizeof(snStr);
    if(maxeye_read_device_sn((uint8_t *)snStr,&bLen)!=NVDS_SUCCESS)
    {
        sys_device_uid_get(notfbuff);
        sprintf(snStr,"M7010001C2D%02X%02X%02X",notfbuff[13],notfbuff[14],notfbuff[15]);
    }
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_OPPO_SN;
    notfbuff[2]=0x01;
    notfbuff[3]=bLen;

    memcpy(&notfbuff[4],snStr,bLen);
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s bLen:%d >", __func__, bLen); 
    }

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
void srvc1_product_sn_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    char snStr[32];
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    uint16_t bLen=sizeof(snStr);
    if(maxeye_read_device_sn((uint8_t *)snStr,&bLen)!=NVDS_SUCCESS)
    {
        notfbuff[0]=0x2F;
        notfbuff[1]=MAXEYE_CLI_READ_SN;
        notfbuff[2]=1;
        notfbuff[3]=0;
    }
    else
    {
        notfbuff[0]=0x2F;
        notfbuff[1]=MAXEYE_CLI_READ_SN;
        notfbuff[2]=1;
        notfbuff[3]=bLen;
        memcpy(&notfbuff[4],snStr,bLen);
    }
    logX("CLI_read_SN0: %d-%d-%d-%d-%d-%d-%d-%d--%d-%d-%d-%d-%d-%d-%d-%d\n",\
    snStr[0],snStr[1],snStr[2],snStr[3],snStr[4],snStr[5],snStr[6],snStr[7],\
    snStr[8],snStr[9],snStr[10],snStr[11],snStr[12],snStr[13],snStr[14],snStr[15]); 

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s bLen:%d >", __func__, bLen); 
    }

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
void srvc1_mmi_sn_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    char snStr[32];
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    uint16_t bLen=sizeof(snStr);
    if(maxeye_read_device_sn((uint8_t *)snStr,&bLen)!=NVDS_SUCCESS)
    {
        sys_device_uid_get(notfbuff);
        sprintf(snStr,"M7010001C2D%02X%02X%02X",notfbuff[13],notfbuff[14],notfbuff[15]);
    }
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_MMI_SN;
    notfbuff[2]=0x01;
    notfbuff[3]=bLen;
    memcpy(&notfbuff[4],snStr,bLen);
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s bLen:%d >", __func__, bLen); 
    }

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
void srvc1_fp24_version_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint32_t McuVer;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

   
    notfbuff[0]=0x2C;
    notfbuff[1]=0x17;
    notfbuff[2]=0x01;
    notfbuff[3]=0x04;

    maxeye_get_mcu_firmware_version((uint8_t *)&McuVer);


    notfbuff[4]=McuVer>>8;
    notfbuff[5]=McuVer;
    notfbuff[6]=McuVer>>24;
    notfbuff[7]=McuVer>>16;


    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_to_sleep_ack_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

   
    notfbuff[0]=0x2C;
    notfbuff[1]=0x18;
    notfbuff[2]=0x7F;
    notfbuff[3]=0x04;
    notfbuff[4]=0xA0;
    notfbuff[5]=0x86;
    notfbuff[6]=0x01;

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_wdt_rst_ack_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

   
    notfbuff[0]=0x2C;
    notfbuff[1]=0x1A;
    notfbuff[2]=0x7F;
    notfbuff[3]=0x04;
    notfbuff[4]=0xA0;
    notfbuff[5]=0x86;
    notfbuff[6]=0x01;

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_voltage_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

   
    if(bleCurrentLatency!=0)
    {
        pencil_run_connection_parameter_set();
    }

    srvc1_refresh_battery_paramenter(1);

    notfbuff[0]=0x2C;
    notfbuff[1]=0x45;
    notfbuff[2]=0x01;
    notfbuff[3]=0x02;
    notfbuff[4]=cw_bat.voltage;
    notfbuff[5]=(uint8_t)(cw_bat.voltage>>8);
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_current_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

   
    srvc1_refresh_battery_paramenter(0);

    notfbuff[0]=0x2C;
    notfbuff[1]=0x16;
    notfbuff[2]=0x01;
    notfbuff[3]=0x02;
    notfbuff[4]=cw_bat.current;
    notfbuff[5]=(uint8_t)(cw_bat.current>>8);
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_temp_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

   
    srvc1_refresh_battery_paramenter(1);

    notfbuff[0]=0x2C;
    notfbuff[1]=0x19;
    notfbuff[2]=0x01;
    notfbuff[3]=1;//0x02;
    notfbuff[4]=(cw_bat.temp/10);
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_dbclick_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
   
    notfbuff[0]=0x2C;
    notfbuff[1]=0x02;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    notfbuff[4]=doubleClick;
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_ack_notify(uint8_t msgType)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
   
    notfbuff[0]=0x2C;
    notfbuff[1]=msgType;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_decoding_test_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
   
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_DECODING_TEST;
    notfbuff[2]=0x01;
    notfbuff[3]=0;
    error_code=maxeye_srvc1_notify(0,notfbuff,SRVC_PACK_SIZE);
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

    maxeye_start_decoding_test();
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
void srvc1_rep_mmi_flag_notify(void) //半成品测试结果
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint16_t wLen;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
   

    if(maxeye_read_pcba_test(notfbuff,&wLen)!=NVDS_SUCCESS) //未进行PCBA测试
    {
        return;
    }

    if(notfbuff[0]!=2)//0-未进行 PCBA 测试，1-PCBA 测试结果为失败，2-PCBA 测试通过
    {
        return;  
    }


    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_MMI_FLAG;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;


    if(maxeye_read_product_test(&notfbuff[4],&wLen)!=NVDS_SUCCESS)
    {
        notfbuff[4]=0;//0-未进行 MMI 测试，1-MMI 测试结果为失败，2-MMI 测试通过
    }


    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_coding_rate_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t decodingRate=0;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
   
    if(maxeye_get_decoding_status(&decodingRate)!=0)
    {
        // decodingRate=100;
    }

    notfbuff[0]=0x2C;
    notfbuff[1]=0x09;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    notfbuff[4]=decodingRate; //误码率
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_g_sensor_calibration_value_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t gSensorAxis[6];
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
   
    get_g_sensor_xyz_parameter(gSensorAxis);
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_G_SENSOR_CALI_VAL;
    notfbuff[2]=0x01;
    notfbuff[3]=0x07;
    notfbuff[4]=0x01; 
    memcpy(&notfbuff[5],gSensorAxis,6);
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_g_sensor_value_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t gSensorAxis[6];
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
   
    get_g_sensor_xyz_parameter(gSensorAxis);
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_G_SENSOR_VAL;
    notfbuff[2]=0x01;
    notfbuff[3]=0x00;
    notfbuff[4]=0x02; 
    notfbuff[5]=0x02; 
    notfbuff[6]=gSensorAxis[0]; 
    notfbuff[7]=gSensorAxis[1]; 
    
    notfbuff[8]=0x03; 
    notfbuff[9]=0x02; 
    notfbuff[10]=gSensorAxis[2]; 
    notfbuff[11]=gSensorAxis[3]; 

    notfbuff[12]=0x04; 
    notfbuff[13]=0x02; 
    notfbuff[14]=gSensorAxis[4]; 
    notfbuff[15]=gSensorAxis[5]; 

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_pressure_value_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
    uint16_t wPressure;

    if(maxeye_get_pressure(&wPressure)!=0)
    {
        logX("get pressure fail"); 
        return;
    }
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_PRESSURE_VAL;
    notfbuff[2]=0x01;
    notfbuff[3]=0x02;
    notfbuff[4]=(uint8_t)wPressure; 
    notfbuff[5]=(uint8_t)(wPressure>>8); 
    
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_pressure_test_notify(uint8_t bStatus)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_PRESSURE_TEST;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    notfbuff[4]=bStatus; 
    
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_pressure_cali_notify(uint8_t bStatus)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    notfbuff[0]=0x2F;
    notfbuff[1]=MAXEYE_CLI_PRESSURE_CALI;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    notfbuff[4]=bStatus; 
    
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_pressure_cali_result_notify(uint8_t *pData)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    notfbuff[0]=0x2F;
    notfbuff[1]=MAXEYE_CLI_GET_PRESSURE_CALI_RESULT;
    notfbuff[2]=0x01;
    notfbuff[3]=3;
    memcpy(&notfbuff[4],pData,3);
    
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

}


void srvc1_rep_disable_preload_notify(uint8_t *pData)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code = SDK_SUCCESS;
    uint8_t notfbuff[SRVC_PACK_SIZE] = {0};

    notfbuff[0] = 0x2F;
    notfbuff[1] = MAXEYE_CLI_DISABLE_PRELOAD;
    notfbuff[2] = 0x01;
    notfbuff[3] = 1;
    memcpy(&notfbuff[4], pData, 1);

    error_code = maxeye_srvc1_notify(0, notfbuff, sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

}

void srvc1_rep_rst_voltameter_notify(uint8_t *pData)
{
    logX("<%s >", __func__); 
    sdk_err_t  error_code = SDK_SUCCESS;
    uint8_t notfbuff[SRVC_PACK_SIZE] = {0};

    notfbuff[0] = 0x2F;
    notfbuff[1] = MAXEYE_CLI_RST_VOLTAMETER;
    notfbuff[2] = 0x01;
    notfbuff[3] = 1;
    memcpy(&notfbuff[4], pData, 1);

    error_code = maxeye_srvc1_notify(0, notfbuff, sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_shipmode_notify(void)
{
    ship_mode_enable();
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
void srvc1_rep_aging_flag_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
    uint16_t wLen;

    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_AGING_FLAG;
    notfbuff[2]=1;
    notfbuff[3]=1;

    if(maxeye_read_aging_test(&notfbuff[4],&wLen)!=NVDS_SUCCESS)
    {
        notfbuff[4]=0;//0-未完成老化，1-已成功完成老化
    }
    else
    {
        notfbuff[4]=1;
    }
    notfbuff[5]=0;

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

}





/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * 获取老化测试结果  2C 15 01 02 FF 03 Bit0：蓝牙测试 Bit1：ASIC 测试 Bit2：休眠唤醒测试 Bit3：电池测试 Bit4：压
 * Bit5：ASensor 测试 Bit6：充电 IC 测试 Bit7：电容触摸测试 Bit8：无线充电测试 Bit9：电量计测试
 * 
 * @return 
 *****************************************************************************************
 */
void srvc1_rep_aging_test_result_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
    uint16_t wLen;
    
    
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_AGING_RESULT;
    notfbuff[2]=0x01;
    notfbuff[3]=0x02;

    if(maxeye_read_aging_test(&notfbuff[4],&wLen)!=NVDS_SUCCESS)
    {
        notfbuff[4]=0xFF;
        notfbuff[5]=3;
    }

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_aging_test_ack_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
    // uint16_t chipid;

    notfbuff[0]=0xFF;
    notfbuff[1]=3;
    maxeye_write_aging_test(notfbuff,2); //重置老化状态，B0-标志，B1-2 老化状态B3 老化次数

    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_SET_AGING_TEST;
    notfbuff[2]=0x7F;
    notfbuff[3]=0x04;
    //if(wlc_read_chip_type(&chipid)==SDK_SUCCESS)
    {
        //if(chipid==WLC_CHIP_ID)
        {
            if(maxeye_mmi_aging_test_start(1000)==SDK_SUCCESS)//预留10s 确保笔能吸附到测试治具
            {
                notfbuff[4]=0xA0;
                notfbuff[5]=0x86;
                notfbuff[6]=0x01;                     //取值为十进制 100000 时表示进入老化成功
                wAgingCnt=AGING_TEST_CNT;             //配置测试循环次数
                wAging_time_tick=AGING_TEST_TIME_CNT; //配置测试循环时间


                
				error_code = qfy_maxeye_time1s_event_start(1000); 
				for(uint8_t i=0; i<3; i++){
					if(error_code == SDK_SUCCESS){
						i=4 ;
					}else {
						error_code = qfy_maxeye_time1s_event_start(1000); 
					}
				}
            }
            else{
                notfbuff[4]=1;
            }
        }
    } 
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
	logX("<%s wAgingStatus:%d wAgingCnt:%d time:%d ",__func__, wAgingStatus,wAgingCnt,wAging_time_tick); 
	
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_aging_number_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
    uint16_t wLen;

    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_AGING_NUMBER;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    
    maxeye_read_aging_test_number(&notfbuff[4],&wLen);
    
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_set_mmi_flag_notify(uint8_t bStatus)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_SET_MMI_FLAG;
    notfbuff[2]=0x01;
    notfbuff[3]=0;

    maxeye_write_product_test(&bStatus,1);

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_charge_coefficient_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_CHARGE_CODFFICIENT;
    notfbuff[2]=0x01;
    notfbuff[3]=0x01;
    notfbuff[4]=120;  //充电系数

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_device_check_ack_notify(uint8_t bTimer)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
    uint16_t chipid;

    notfbuff[0]=0xFF;
    notfbuff[1]=3;
    maxeye_write_aging_test(notfbuff,2); //重置老化状态，B0-标志，B1-2 老化状态B3 老化次数
    
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_DEVICE_CHECK;
    notfbuff[2]=0x7F;
    notfbuff[3]=0x04;
    if(wlc_read_chip_type(&chipid)==SDK_SUCCESS)
    {
        if(chipid==WLC_CHIP_ID)
        {
            if(maxeye_mmi_aging_test_start(200)==SDK_SUCCESS)
            {
                notfbuff[4]=0xA0;
                notfbuff[5]=0x86;
                notfbuff[6]=0x01;//取值为十进制 100000 时表示进入老化成功
                wAgingCnt=(bTimer>0)?bTimer:100;
            }
            else
            {
                notfbuff[4]=1;
            }
        }
    }
    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * 获取老化测试结果  2C 15 01 02 FF 03 Bit0：蓝牙测试 Bit1：ASIC 测试 Bit2：休眠唤醒测试 Bit3：电池测试 Bit4：压
 * Bit5：ASensor 测试 Bit6：充电 IC 测试 Bit7：电容触摸测试 Bit8：无线充电测试 Bit9：电量计测试
 * 
 * @return 
 *****************************************************************************************
 */
void srvc1_rep_device_check_result_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};
    uint16_t wLen;
    
    
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_DEVICE_CHECK_RESULT;
    notfbuff[2]=0x01;
    notfbuff[3]=0x02;

    if(maxeye_read_aging_test(&notfbuff[4],&wLen)!=NVDS_SUCCESS)
    {
        notfbuff[4]=0xFF;
        notfbuff[5]=3;
    }

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_touch_version_notify(void)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;
    uint8_t notfbuff[SRVC_PACK_SIZE]={0};

    
    notfbuff[0]=0x2C;
    notfbuff[1]=BLE_REQ_FILM_VERSION;
    notfbuff[2]=1;
    notfbuff[3]=1;

    touch_read_firmware_version(&notfbuff[4]); 
    touch_read_firmware_version(&notfbuff[4]);

    error_code=maxeye_srvc1_notify(0,notfbuff,sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
void srvc1_rep_pencil_status_notify(uint8_t bStatus)
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code = SDK_SUCCESS;
    uint8_t notfbuff[SRVC_PACK_SIZE] = {0};

    notfbuff[0] = 0x2C;
    notfbuff[1] = BLE_REP_PENCIL_STATUS;
    notfbuff[2] = 1;
    notfbuff[3] = 1;
    notfbuff[4] = !!bStatus;

    error_code=maxeye_srvc1_notify(0, notfbuff, sizeof(notfbuff));
    if(error_code!=SDK_SUCCESS)
    {
        logX("</%s error_code:0x%04x>", __func__, error_code); 
    }
    else
    {
        logX("</%s >", __func__); 
    }

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
sdk_err_t maxeye_srv_event_start(uint16_t wDelaymS,bool fgRefresh) //刷新
{
	logX("<%s >", __func__); 
    sdk_err_t     ret;

    if(fgRefresh)
    {
       app_timer_stop(maxeye_srvc_event_id);
    }
    ret=app_timer_start(maxeye_srvc_event_id, wDelaymS, NULL);
	logX("</%s >", __func__); 
    return ret;
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
static void maxeye_srv_event_handler(void* p_ctx)
{
	logX("<%s >", __func__); 
    uint8_t buff[BLE_MSG_QUEUE_ITEM_SIZE]={0};


    if(app_queue_pop(&ble_msg_QueueHandle,buff)!=SDK_SUCCESS)
    {
        return;
    }

    if(app_queue_items_count_get(&ble_msg_QueueHandle))
    {
        app_timer_start(maxeye_srvc_event_id, HANDSHAKE_SRVC_ACK_DELAY, NULL);
    }

    switch (buff[0])
    {
        case BLE_REQ_CHARGEING_STATUS:
        {
            bHandshakeStatus|=HANDSHAKE_REQ_CHARGE_STATUS;
            srvc1_battery_charge_notify();
        }
        break;

        case BLE_REQ_DB_CLICK_STATUS:
        {
            fgDbClickTest=true;
        }
        break;

        case BLE_REQ_FW_VERSION:
        {
            bHandshakeStatus|=HANDSHAKE_REQ_FW_VERSION;
            srvc1_fw_version_notify();
        }
        break;

        case BLE_REQ_BATTERY_CAP:
        {
            bHandshakeStatus|=HANDSHAKE_REQ_BATT_CAP;
            srvc1_battery_capacity_notify();
        }
        break;


        case BLE_REQ_BATT_CURRENT:
        {
            srvc1_rep_current_notify();
        }
        break;

        case BLE_REQ_BATT_TEMP:
        {
            srvc1_rep_temp_notify();
        }
        break;

        case BLE_REQ_BATT_VOLTAGE:
        {
            srvc1_rep_voltage_notify();
        }
        break;


        case BLE_REQ_FP24_VERSION:
        {
            srvc1_fp24_version_notify();
        }
        break;



        case BLE_REQ_PENCIL_SLEEP:
        {
            if(buff[1])
            {
                maxeye_pencil_to_sleep();
            }
            else //适配apk ，协议文档无需应答
            {
                buff[1]=1;
                srvc1_to_sleep_ack_notify();
                app_queue_push(&ble_msg_QueueHandle,buff);
                maxeye_srv_event_start(10,false);
            }
        }
        break;



        case BLE_REQ_HW_VERSION:
        {
            bHandshakeStatus|=HANDSHAKE_REQ_HW_VERSION;
            srvc1_hw_version_notify();
        }
        break;

        case BLE_REQ_MODEL_ID:
        {
            bHandshakeStatus|=HANDSHAKE_REQ_MODE_ID;
            srvc1_model_id_notify();
        }
        break;

        case BLE_REQ_OPPO_SN:
        {
            bHandshakeStatus|=HANDSHAKE_REQ_SN;
            srvc1_oppo_sn_notify();
        }
        break;


        case BLE_REQ_MMI_FLAG:
        {
            srvc1_rep_mmi_flag_notify();
            if(bleCurrentLatency!=0)
            {
                pencil_run_connection_parameter_set();
            }
            if(fgDevSleep)
            {
                maxeye_pencil_wakeup();
            }
            ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
        }
        break;


        case BLE_SET_MMI_FLAG:
        {
            srvc1_rep_set_mmi_flag_notify(buff[1]);
        }
        break;

//解码
        case BLE_REQ_CODING_RATE:
        {
            srvc1_rep_coding_rate_notify();
        }
        break;

        case BLE_REQ_DECODING_TEST:
        {
            // logX("start decoding");
            srvc1_rep_decoding_test_notify();

        }
        break;

        case BLE_G_SENSOR_CALIBRATION:
        {
            srvc1_rep_ack_notify(BLE_G_SENSOR_CALIBRATION);
        }
        break;

        case BLE_REQ_G_SENSOR_CALI_VAL:
        {
            srvc1_rep_g_sensor_calibration_value_notify();
        }
        break;

        case BLE_REQ_G_SENSOR_VAL:
        {
            srvc1_rep_g_sensor_value_notify();
        }
        break;

        case BLE_REQ_PRESSURE_TEST:
        {
            srvc1_rep_pressure_test_notify(buff[1]);
        }
        break;

        case BLE_SET_PRESSURE_GRADE:
        {
            srvc1_rep_ack_notify(BLE_SET_PRESSURE_GRADE);
        }
        break;

        case BLE_REQ_PRESSURE_VAL:
        {
            srvc1_rep_pressure_value_notify();
            ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
        }
        break;

        case BLE_REQ_CHARGE_CODFFICIENT:
        {
            srvc1_rep_charge_coefficient_notify();
        }
        break;

        case BLE_REQ_MMI_SN:
        {
            srvc1_mmi_sn_notify();
        }
        break;

        case BLE_REQ_AGING_FLAG:
        {
            srvc1_rep_aging_flag_notify();
        }
        break;


        case BLE_REQ_AGING_RESULT:
        {
            srvc1_rep_aging_test_result_notify();
        }
        break;

        case BLE_REQ_SHIPMODE:
        {
            srvc1_rep_shipmode_notify();
        }
        break;

        case BLE_SET_AGING_TEST:
        {
            srvc1_rep_aging_test_ack_notify();
        }
        break;

        case BLE_REQ_AGING_NUMBER:
        {
            srvc1_rep_aging_number_notify();
        }
        break;


        case BLE_CMD_HALL_CLOSE:
        {
            if((bHandshakeStatus&HANDSHAKE_REQ_BATT_CAP)==0)
            {
                logX("read batt cap lose");
                srvc1_battery_capacity_notify();
            }
        }
        break;

        case BLE_CMD_HALL_OPEN:
        {
            // 不可关充电
        }
        break;


        case BLE_REQ_PENCIL_WDT_RST:
        {
            srvc1_wdt_rst_ack_notify();
            maxeye_wdt_test();
        }
        break;



        case BLE_REQ_DEVICE_CHECK:
        {
            srvc1_rep_device_check_ack_notify(buff[1]);
        }
        break;


        case BLE_REQ_DEVICE_CHECK_RESULT:
        {
            srvc1_rep_device_check_result_notify();
        }
        break;


        case BLE_REQ_FILM_VERSION:
        {
            srvc1_rep_touch_version_notify();
        }
        break;

#ifndef DISABLE_PENCIL_STATE_REPORT
        case BLE_REP_PENCIL_STATUS:
        {
            pencil_status_rsp_handler(buff[1]);
        }
        break;
#endif

        default:
        break;
    }
	logX("</%s >", __func__); 
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
void maxeye_srv_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&maxeye_srvc_event_id, ATIMER_ONE_SHOT, maxeye_srv_event_handler);
    APP_ERROR_CHECK(error_code);
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
void srvc1_write_attr_parse(uint8_t *pData,uint8_t bLen)  
{
	logX("<%s >", __func__); 
    uint8_t buff[BLE_MSG_QUEUE_ITEM_SIZE]={0};

    if(pData[0]!=0x2C||bLen<2)
    {
        return;
    }
    buff[0]=pData[1];
    buff[1]=(bLen>4)?pData[4]:0;
    fgBattCapNtfEnable=true;
    app_queue_push(&ble_msg_QueueHandle,buff);
    maxeye_srv_event_start(HANDSHAKE_SRVC_ACK_DELAY,false);
	logX("</%s >", __func__); 
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
uint8_t srvc2_battery_capacity_notify(void)  
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;

    battLevel = cw_bat.capacity;
    bas_batt_lvl_update(0, 0, battLevel);

    if(!fgBattCapNtfEnable)
    {
        return 1;
    }

    error_code=maxeye_srvc2_char1_notify(0,&battLevel,1);
    if(error_code!=SDK_SUCCESS)
    {
        logX("s2 cap ntf err %x",error_code); 
        battLevel=BATT_CAP_LEVEL_INVAILD;
        // maxeye_srvc1_notify(0,&battLevel,1);//测试主机
        return 2;
    }
    else
    {
       logX("s2 cap ntf"); 
    }  
	logX("</%s >", __func__); 
    return 0;
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

void srvc2_battery_charge_notify(void)  
{
	logX("<%s >", __func__); 
    uint8_t bStatus;
    sdk_err_t  error_code;

    if(!fgBattCapNtfEnable)
    {
        return;
    }
    bStatus=(battChargeStatus==BATT_CHARGEING_ENABLE)?EN_CHARGE_STATUS:DIS_CHARGE_STATUS;
    error_code=maxeye_srvc2_char3_notify(0,&bStatus,1);
    if(error_code!=SDK_SUCCESS)
    {
        logX("s2 chg ntf err %x",error_code); 
    }
    else
    {
        LOG("s2 chg ntf %x",bStatus);  
    }
	logX("</%s >", __func__); 
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
void srvc2_double_click_notify(void)  
{
	logX("<%s >", __func__); 
    sdk_err_t  error_code;

    if(fgDbClickTest)
    {
        srvc1_rep_dbclick_notify();   
        fgDbClickTest=false;
    }

    if(!fgBattCapNtfEnable)
    {
        return;
    }

    doubleClick=0x01;
    error_code=maxeye_srvc2_char2_notify(0,&doubleClick,1);
    if(error_code!=SDK_SUCCESS)
    {
        logX("s2 db click err %x",error_code); 
    }
	logX("</%s >", __func__); 
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
void srvc2_log_notify(char *pStr)  
{
    uint16_t ret;

    if(bleConnStatus<BLE_CONNECTED)
    {
       return;        
    }

    ret=maxeye_srvc2_char4_notify(0,(uint8_t *)pStr,strlen(pStr));
    if(ret!=0)
    {
        logX("<%s ret:%d>", __func__, ret);
    }
}






