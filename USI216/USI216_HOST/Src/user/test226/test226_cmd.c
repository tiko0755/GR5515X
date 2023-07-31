/**
 *****************************************************************************************
 *
 * @file test226.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "test226.h"
#include "test226_cmd.h"
#include "string.h"
#include "app_log.h"
#include "app_error.h"

#include "test226_info.h"
#include "test226_batt.h"
#include "unknown_svr1.h"
#include "unknown_svr2.h"
#include "unknown_svr3.h"
#include "test226_rssi.h"
#include "build_services_proc.h"

#include "thsBoard.h"
#include "user_app.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
 static u8 oldMAC[6] = {0};
 
// completed callback for build services
static void buildSrvs_cmplt_str(s32 rslt, void* argv);
static void buildSrvs_cmplt_hex(s32 rslt, void* argv);
//completed callback for info.read.sn
static void infoReadSN_cmplt_str(s32 rslt, void* argv);
static void infoReadSN_cmplt_hex(s32 rslt, void* argv);
// completed callback for fetchmac
static void fetchMAC_cmplt_str(s32 rslt, void* argv);
static void fetchMAC_cmplt_hex(s32 rslt, void* argv);

// completed callback for pen.disconnect
static void disconnect_cmplt_str(s32 rslt, void* argv);
static void disconnect_cmplt_hex(s32 rslt, void* argv);

static void svr1chr1_res(s32 rslt, void* argv);
static void svr2chr1_res(s32 rslt, void* argv);
static void svr2chr2_res(s32 rslt, void* argv);
static void svr2chr3_res(s32 rslt, void* argv);
static void svr2chr4_res(s32 rslt, void* argv);
static void svr3chr1_res(s32 rslt, void* argv);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */

static int32_t svrChr_req_common(char *svr_chr, int32_t* x, uint8_t len){
    uint8_t i, xCmd[32];
    for(i=0;i<len;i++){    xCmd[i] = x[i];   }      // only take lower byte
    if(strncmp(svr_chr, "svr1chr1", strlen("svr1chr1"))==0)     {    proc_svr1chr1_req(xCmd, 8, svr1chr1_res, 5000); }
    else if(strncmp(svr_chr, "svr2chr1", strlen("svr2chr1"))==0){    proc_svr2chr1_req(xCmd, len, svr2chr1_res, 5000); }
    else if(strncmp(svr_chr, "svr2chr2", strlen("svr2chr2"))==0){    proc_svr2chr2_req(xCmd, len, svr2chr2_res, 5000); }
    else if(strncmp(svr_chr, "svr2chr3", strlen("svr2chr3"))==0){    proc_svr2chr3_req(xCmd, len, svr2chr3_res, 5000); }
    else if(strncmp(svr_chr, "svr2chr4", strlen("svr2chr4"))==0){    proc_svr2chr4_req(xCmd, len, svr2chr4_res, 5000); }
    else if(strncmp(svr_chr, "svr3chr1", strlen("svr3chr1"))==0){    proc_svr3chr1_req(xCmd, len, svr3chr1_res, 5000); }
    else{   return -1;  }
    return 0;
}   

u8 test226_cmd(const uint8_t* cmd, u8 len, XPrint xprint){
APP_LOG_DEBUG("<%s>", __func__);
    char* CMD = (char*)cmd;
    char svr_chr[16] = {0};
    uint8_t xCmd[32], buff[8];
    s32 x[32],i,j;

    if(strncmp(CMD, "pen.debug", strlen("pen.debug"))==0){
        print("+msg@pen.debug(g_loaded:%d, g_linked:%d)\r\n", g_loaded, g_linked);
    }
    else if(strncmp(CMD, "pen.reset", strlen("pen.reset"))==0){
        print("+ok@pen.reset()\r\n");
        NVIC_SystemReset();
    }
    else if(strncmp(CMD, "pen.mac", strlen("pen.mac"))==0){
        cps4041.start_getMAC(&cps4041.rsrc, fetchMAC_cmplt_str, 1);
        return 1;
    }

    else if(sscanf(CMD, "pen.connect %x %x %x %x %x %x",&x[0],&x[1],&x[2],&x[3],&x[4],&x[5])==6){
        buff[0] = x[5];
        buff[1] = x[4];
        buff[2] = x[3];
        buff[3] = x[2];
        buff[4] = x[1];
        buff[5] = x[0];
//        memcpy(buff,x,6);
        start_buildSrvProc(buff, buildSrvs_cmplt_str);
        return 1;
    }

    else if(strncmp(CMD, "pen.connect", strlen("pen.connect"))==0){
//        if(g_loaded == 0){
//            print("+err@pen.connect('invalid_mac')\r\n");
//            return 1;
//        }
        start_buildSrvProc(NULL, buildSrvs_cmplt_str);
        return 1;
    }
    else if(strncmp(CMD, "pen.disconnect", strlen("pen.disconnect"))==0){
        memcpy(oldMAC,g_loadedMAC,6);
        xBleGap_disconnect(disconnect_cmplt_str);
        return 1;
    }
    
    else if(sscanf(CMD, "req %s %x %x %x %x %x %x %x %x", svr_chr, &x[0],&x[1],&x[2],&x[3],&x[4],&x[5],&x[6],&x[7])==9){
        if(svrChr_req_common(svr_chr, x, 8) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }
    else if(sscanf(CMD, "req %s %x %x %x %x %x %x %x", svr_chr, &x[0],&x[1],&x[2],&x[3],&x[4],&x[5],&x[6])==8){
        if(svrChr_req_common(svr_chr, x, 7) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }
    else if(sscanf(CMD, "req %s %x %x %x %x %x %x", svr_chr, &x[0],&x[1],&x[2],&x[3],&x[4],&x[5])==7){
        if(svrChr_req_common(svr_chr, x, 6) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }
    else if(sscanf(CMD, "req %s %x %x %x %x %x", svr_chr, &x[0],&x[1],&x[2],&x[3],&x[4])==6){
        if(svrChr_req_common(svr_chr, x, 5) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }
    else if(sscanf(CMD, "req %s %x %x %x %x", svr_chr, &x[0],&x[1],&x[2],&x[3])==5){
        if(svrChr_req_common(svr_chr, x, 4) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }
    else if(sscanf(CMD, "req %s %x %x %x", svr_chr, &x[0],&x[1],&x[2])==4){
        if(svrChr_req_common(svr_chr, x, 3) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }
    else if(sscanf(CMD, "req %s %x %x", svr_chr, &x[0],&x[1])==3){
        if(svrChr_req_common(svr_chr, x, 2) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }
    else if(sscanf(CMD, "req %s %x", svr_chr, &x[0])==2){
        if(svrChr_req_common(svr_chr, x, 1) < 0){
            xprint("+err@%s",CMD); 
        }
        return 1;
    }


APP_LOG_DEBUG("</%s> ", __func__);
    return 0;
}

static void infoReadSN_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
    buff_t* x=(buff_t*)argv;
    if(rslt==0){    print("+ok@pen.sn('%s')\r\n", (char*)x->buff);    }
    else{    print("+err@pen.sn(%d)\r\n", rslt);    }
APP_LOG_DEBUG("</%s> ", __func__);
}


static void fetchMAC_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
    cps4041.cbRegFetched(&cps4041.rsrc, NULL);
    if(rslt==0){
        u8* mac = (u8*)argv;
        print("+ok@pen.mac('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
        mac[5],
        mac[4],
        mac[3],
        mac[2],
        mac[1],
        mac[0]);
    }
    else{    
        print("+err@pen.mac(%d)\r\n", rslt);
    }
APP_LOG_DEBUG("</%s> ", __func__);
}



static void buildSrvs_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
    if(rslt==0){
        u8* m = argv;
        print("+ok@pen.connect('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
            m[5],m[4],m[3],m[2],m[1],m[0]
        );
        cps4041.charger_en(&cps4041.rsrc);
    }
    else{
        print("<%s rslt:%d> ", __func__, rslt);
        print("+err@pen.connect()\r\n");
    }
APP_LOG_DEBUG("</%s> ", __func__);
}



// completed callback for pen.rssi %op
static void disconnect_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
    if(rslt==0){    print("+ok@pen.disconnect('%02x:%02x:%02x:%02x:%02x:%02x')\r\n", oldMAC[5],oldMAC[4],oldMAC[3],oldMAC[2],oldMAC[1],oldMAC[0]);    }
    else{    print("+err@pen.disconnect()\r\n", rslt);    }
APP_LOG_DEBUG("</%s> ", __func__);
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
u16 fetchHexCLFromRingBuffer(RINGBUFF_T* rb, u8* line, u16 len){
    u8 checkcode;
    u16 wCmd,wLen,ret=0;
    s32 i,bytes,count;
    u8 buff[256];
        
    count = RingBuffer_GetCount(rb);
    if((count <= 0) || (line==NULL) || (len==0))    return 0;
    
    // only take the lase receive
    while(count > 256){
        RingBuffer_Pop(rb, buff);    // abandoned
        count = RingBuffer_GetCount(rb);
    }
    memset(buff,0,256);
    bytes = RingBuffer_PopMult(rb, buff, 256);
    RingBuffer_Flush(rb);
    
    if(ret==0){    RingBuffer_InsertMult(rb, buff, bytes);        }    // restore

    return ret;
}


static void svrChr_res(const char* PRE, s32 rslt, void* argv){
    buff_t* x = (buff_t*)argv;
    char str[128] = {0}, *pStr;
    u8 i;
    
    for(i = 0; i < x->len; i++){
        pStr = &str[strlen(str)];
        sprintf(pStr,"%02x", x->buff[i]);
    }

    if(rslt==0){    print("+ok@req %s %s\r\n", PRE, str);    }
    else{    print("+err@%req %s %d\r\n", PRE, rslt);    }
}


static void svr1chr1_res(s32 rslt, void* argv){ svrChr_res("svr1chr1", rslt, argv); }
static void svr2chr1_res(s32 rslt, void* argv){ svrChr_res("svr2chr1", rslt, argv); }
static void svr2chr2_res(s32 rslt, void* argv){ svrChr_res("svr2chr2", rslt, argv); }
static void svr2chr3_res(s32 rslt, void* argv){ svrChr_res("svr2chr3", rslt, argv); }
static void svr2chr4_res(s32 rslt, void* argv){ svrChr_res("svr2chr4", rslt, argv); }
static void svr3chr1_res(s32 rslt, void* argv){ svrChr_res("svr3chr1", rslt, argv); }
