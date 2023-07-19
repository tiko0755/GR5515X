
#include "app_log.h"
#include "app_error.h"
#include "app_i2c.h"

#include "maxeye_i2c.h"
#include "maxeye_ft3308.h"

/*
 * DEFINES
 *****************************************************************************************
 */
// #define DRV_LOG_EN

#ifdef  DRV_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif

#define RAW_DATA_MIN           20000
#define RAW_DATA_MAX           32000

#define FT3308_DELAY_MS(X)            delay_ms(X)

#define FT3308_I2C_DEV_ADDR           (0x38) //(0x70)   /* FT3308 i2C address */


#define FTS_POINT_NUM_MAX             10  
#define FTS_BYTES_POINT               6   /*byte numbers of one point*/
#define ERROR_CODE_OK                 0

/*Length of touch point buffer, forbid to modify it*/
#define FTS_TOUCH_DATA_LEN            97//(2 + (FTS_POINT_NUM_MAX * FTS_BYTES_POINT))

#define FTS_MAX_ID                    0x0A    /* Max point ID */

/*Register map of touch point data, please refer to register table list*/
#define TOUCHR_CPOINT                 0x01
#define TOUCHR_XH                     0x02
#define TOUCHR_XL                     0x03
#define TOUCHR_YH                     0x04
#define TOUCHR_YL                     0x05
#define TOUCHR_WEIGHT                 0x06
#define TOUCHR_MISC                   0x07


											
struct ts_point {
    int event;             /*Event 双击事件*/
    int tx;                /* 双击位置Tx */
    int rx;                /* 双击位置Rx */
    int time;              /* 双击间隔时间 */
    int area;              /*面积大小 */
    int status;            /*Rawdata异常检测*/
   
};


struct ts_touch_event 
{
    uint8_t point_num;              /*valid touch point numbers in current touch event including pressed(down) and rised(up) fingers(points)*/
    uint8_t finger_num_on_screen;   /*finger numbers on touch screen in current touch event including pressed(down) fingers(points)*/
    struct ts_point points[FTS_POINT_NUM_MAX];  /*points informationin current touch event*/
};


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

// int m_RawData[36];

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
int8_t I2cPlatformWrite(uint8_t *reg_data, uint16_t length)
{
	return maxeye_i2c0_transmit(FT3308_I2C_DEV_ADDR,reg_data,length);
}


int8_t I2cPlatformRead(uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	return maxeye_i2c0_read(FT3308_I2C_DEV_ADDR,reg_addr,I2C_MEMADD_SIZE_8BIT,reg_data,length);
}

/************************************************************************
 * Name: touch_write
 * Brief: The function is used to write values to touch chip with I2C BUS
 *  interface following a stop condition.
 *
 *
 * Input: @wbuf: the write buffer.
 *        @wlen: the length of write buffer.
 * Output:
 * Return: return 0 if success, otherwise return error code.
 ***********************************************************************/
int touch_write(uint8_t *wbuf, uint16_t wlen)
{
    int ret = 0;
	int i =3;

	if(wlen==0 ||wbuf ==NULL)
	{
		LOG("touch_write invalid\r\n");
		return ret;
	}
	for(i = 0; i < 3; i++) 
	{
		// ret = I2cPlatformWrite(FT3308_I2C_DEV_ADDR, wbuf[0], &wbuf[1],wlen-1);
		ret = I2cPlatformWrite(wbuf,wlen);
		if (ret!=0) 
		{
			LOG("touch_write twi write fail,ret:%d\r\n", ret);
		} 
		else 
			break;
	}
    return ret;
}

/************************************************************************
 * Name: touch_write
 * Brief: The function is used to read values from touch chip with I2C BUS
 *  interface.
 *
 *  The standard procedure of reading values from touch chip consists of the
 *  two steps:
 *    1. Send I2C writ packet to transfer register addresses or commands 
 *       following a stop condition.
 *    2. Send I2C read packet to read values from touch chip.
 *  Sometimes, only read packet is sent, now step 1(write packet) is ignored 
 *  or skipped with invalid parameters of wbuf and wlen.
 *
 * Input: @wbuf: the write buffer.
 *        @wlen: the size in byte of write buffer.
 *
 * Output: @rbuf: the buffer to save reading values
 *         @rlen: the size in byte that you want to read
 *
 * Return: return 0 if success, otherwise return error code.
 ***********************************************************************/
int touch_read(uint8_t *wbuf, uint16_t wlen, uint8_t *rbuf, uint16_t rlen)
{
    int ret = 0;
	int i =3;

	if(wbuf==NULL ||rbuf==NULL ||wlen==0 ||rlen==0)
	{
		LOG("touch read invaild\r\n");
		return ret;
	}

	for(i = 0; i < 3; i++) 
	{
		// ret = I2cPlatformRead(FT3308_I2C_DEV_ADDR, wbuf[0], rbuf, rlen);
		ret = I2cPlatformRead(wbuf[0], rbuf, rlen);
		if (ret) 
		{
			LOG("touch_read twi read fail,ret:%d\r\n", ret);
		}
		else
			break;
	}

    return ret;
}

/************************************************************************
 * Name: touch_write_reg
 * Brief: Variety of touch_write() function. The function is used to write 
 *  a byte value to corresponding register address.
 *
 *
 * Input: @addr: the register address of touch chip.
 *        @value: the value that you want to write to the register address.
 * Output:
 * Return: return 0 if success, otherwise return error code.
 ***********************************************************************/
int touch_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t wbuf[2] = { 0 };

    wbuf[0] = addr;
    wbuf[1] = value;
    return touch_write(&wbuf[0], 2);
}

/************************************************************************
 * Name: touch_read_reg
 * Brief: Variety of touch_read() function. The function is used to read 
 *  a byte value from corresponding register address.
 *
 *
 * Input: @addr: the register address of touch chip.
 *        
 * Output: @value: the value read from the register address.
 *
 * Return: return 0 if success, otherwise return error code.
 ***********************************************************************/
int touch_read_reg(uint8_t addr, uint8_t *value)
{
    return touch_read(&addr, 1, value, 1);
}







uint8_t GetMode(void)
{
	uint8_t mode[2] ={0};

	touch_read_reg(MODESWITCH_REG, &mode[0]);
	if(mode[0]==4)
	{
		LOG("Current mode of the chip is test mode\r\n");
	}
	else if(mode[0]==0)
	{
		LOG("Current mode of the chip is working mode\r\n");	
	}
	
	return mode[0];
	
}
uint8_t GetGestID(void)
{
	uint8_t gestID[2] ={0};

	touch_read_reg(GEST_ID_REG, &gestID[0]);

	if(gestID[0]==0)
	{
		LOG("No gesture\n");
	}
	switch(gestID[0])
	{
		case 0x10:
		{
			LOG("Move up\n");
		}
		break;
		case 0x14:
		{
			LOG("Move right\n");
		}
		break;
		case 0x18:
		{
			LOG("Move down\n");
		}
		break;
		case 0x1c:
		{
			LOG("Move left");
		}
		break;
		case 0x48:
		{
			LOG("Zoom in\n");
		}
		break;
		case 0x49:
		{
			LOG("Zoom out\n");
		}
		break;

		default:
			break;
	}

	return gestID[0];
}

void GetTouchPoints(void)
{
	uint8_t touch_points[2] ={0};
	touch_read_reg(TD_STATUS_REG, &touch_points[0]);
	LOG("The detect point number =%d\n",touch_points[0]);
}


void TouchKeyTest(void)
{
	uint8_t chipid[3]={0};
	uint8_t focaltechid[2]={0};
	uint8_t fwversion[2]={0};

	touch_read_reg(ID_G_CIPHER_HIGH,&chipid[0]);
	LOG("CIPHER id high = 0x%02x\n",chipid[0]);
	
	touch_read_reg(ID_G_CIPHER_MID, &chipid[1]);
	LOG("CIPHER id low = 0x%02x\n",chipid[1]);
	
	touch_read_reg(FOCALTECH_ID, &focaltechid[0]);
	LOG("focaltech id = 0x%02x\n",focaltechid[0]);
	
	touch_read_reg(FIRMID, &fwversion[0]);
	LOG("fw version = 0x%02x\n",fwversion[0]);
	
	LOG("Current mode of the chip is = %d\n",GetMode());
	GetGestID(); 
	GetTouchPoints();
}




#if 1



/////////////////////////////////test




///add test flim  liangyh
#define MAX_CHANNEL_NUM         36
#define RAW_NUM_P               10


//-----------------------------------------
/*半成品阈值设置*/
const uint16_t HalfRawMin[MAX_CHANNEL_NUM] = 
{
    20000, 20000, 20000, 20000, 20000, 20000,
    20000, 20000, 20000, 20000, 20000, 20000,
    20000, 20000, 20000, 20000, 20000, 20000,
    20000, 20000, 20000, 20000, 20000, 20000,
    20000, 20000, 20000, 20000, 20000, 20000,
    20000, 20000, 20000, 20000, 20000, 20000,
};

const uint16_t HalfRawMax[MAX_CHANNEL_NUM] =
{
    32000, 32000, 32000, 32000, 32000, 32000,
    32000, 32000, 32000, 32000, 32000, 32000,
    32000, 32000, 32000, 32000, 32000, 32000,
    32000, 32000, 32000, 32000, 32000, 32000,
    32000, 32000, 32000, 32000, 32000, 32000,
    32000, 32000, 32000, 32000, 32000, 32000,
};

//threshold of cb
const uint16_t HalfCbMin[MAX_CHANNEL_NUM] =
{
    83,     73,     78,     69,     76,     69,
    86,     87,     82,     73,     75,     72,
    95,     97,     87,     84,     80,     84,
    101,    96,     84,     86,     84,     89,
    104,    112,    96,     97,     91,     89,
    119,    120,    110,    109,    108,    117,
};

const uint16_t HalfCbMax[MAX_CHANNEL_NUM] =
{
    185, 175, 180, 171, 178, 171,
    188, 189, 184, 175, 177, 174,
    197, 199, 189, 186, 182, 186,
    203, 198, 186, 188, 186, 191,
    206, 214, 198, 199, 193, 191,
    221, 222, 212, 211, 210, 219,
};

//threshold of short
#if 0
const uint16_t HalfShortMin[MAX_CHANNEL_NUM] =
{
    109, 102, 106, 97, 102, 90,
    108, 109, 104, 95, 99, 93,
    118, 119, 110, 106, 103, 106,
    123, 119, 107, 108, 106, 112,
    127, 135, 118, 120, 115, 112,
    142, 144, 133, 133, 132, 140,
};
#else
const uint16_t HalfShortMin[MAX_CHANNEL_NUM] =
{
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
};

#endif

const uint16_t HalfShortMax[MAX_CHANNEL_NUM] = 
{
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
};



/*成品阈值设置*/
//threshold of rawdata
const uint16_t RawMin[MAX_CHANNEL_NUM] =
{
    19000, 19000, 19000, 19000, 19000, 19000,
    19000, 19000, 19000, 19000, 19000, 19000,
    19000, 19000, 19000, 19000, 19000, 19000,
    19000, 19000, 19000, 19000, 19000, 19000,
    19000, 19000, 19000, 19000, 19000, 19000,
    19000, 19000, 19000, 19000, 19000, 19000,
};

const uint16_t RawMax[MAX_CHANNEL_NUM] =
{
    33000, 33000, 33000, 33000, 33000, 33000,
    33000, 33000, 33000, 33000, 33000, 33000,
    33000, 33000, 33000, 33000, 33000, 33000,
    33000, 33000, 33000, 33000, 33000, 33000,
    33000, 33000, 33000, 33000, 33000, 33000,
    33000, 33000, 33000, 33000, 33000, 33000,
};

//threshold of cb
const uint16_t CbMin[MAX_CHANNEL_NUM] =
{
    78, 69, 73, 65, 71, 65,
    81, 82, 78, 68, 71, 68,
    90, 91, 83, 80, 75, 80,
    96, 90, 79, 82, 79, 84,
    99, 107, 90, 92, 86, 84,
    114, 115, 104, 104, 103, 112,
};

const uint16_t CbMax[MAX_CHANNEL_NUM] =
{
    188, 179, 183, 175, 181, 175,
    191, 192, 188, 178, 181, 178,
    200, 201, 193, 190, 185, 190,
    206, 200, 189, 192, 189, 194,
    209, 217, 200, 202, 196, 194,
    224, 225, 214, 214, 213, 222,
};

//threshold of short
#if 0
const uint16_t ShortMin[MAX_CHANNEL_NUM] =
{
    105, 98, 102, 94, 99, 88,
    104, 105, 101, 92, 96, 91,
    114, 116, 106, 103, 100, 103,
    120, 115, 103, 105, 103, 108,
    123, 131, 115, 116, 111, 108,
    138, 139, 130, 129, 129, 136,
};
#else
const uint16_t ShortMin[MAX_CHANNEL_NUM] =
{
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
};

#endif 

const uint16_t ShortMax[MAX_CHANNEL_NUM] =
{
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500,
};

//threshold of Touch diff
#if 1
const uint16_t diff_min[MAX_CHANNEL_NUM] =
{
    2500, 2500, 2500, 2500, 2500, 2500,
    2500, 2500, 2500, 2500, 2500, 2500,
    2500, 2500, 2500, 2500, 2500, 2500,
    2500, 2500, 2500, 2500, 2500, 2500,
    2500, 2500, 2500, 2500, 2500, 2500,
    2500, 2500, 2500, 2500, 2500, 2500,
};
#else
const uint16_t diff_min[MAX_CHANNEL_NUM] =
{
    2000, 2000, 2000, 2000, 2000, 2000,
    2000, 2000, 2000, 2000, 2000, 2000,
    2000, 2000, 2000, 2000, 2000, 2000,
    2000, 2000, 2000, 2000, 2000, 2000,
    2000, 2000, 2000, 2000, 2000, 2000,
    2000, 2000, 2000, 2000, 2000, 2000,
};

#endif

const uint16_t diff_max[MAX_CHANNEL_NUM] =
{
    9999, 9999, 9999, 9999, 9999, 9999,
    9999, 9999, 9999, 9999, 9999, 9999,
    9999, 9999, 9999, 9999, 9999, 9999,
    9999, 9999, 9999, 9999, 9999, 9999,
    9999, 9999, 9999, 9999, 9999, 9999,
    9999, 9999, 9999, 9999, 9999, 9999,
};

//threshold of Uniformity
const int16_t UniformityMin[MAX_CHANNEL_NUM] = 
{
    -60, -60, -60, -60, -60, -60,
    -60, -60, -60, -60, -60, -60,
    -60, -60, -60, -60, -60, -60,
    -60, -60, -60, -60, -60, -60,
    -60, -60, -60, -60, -60, -60,
    -60, -60, -60, -60, -60, -60,
};

const int16_t UniformityMax[MAX_CHANNEL_NUM] =
{
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100,
};

const uint16_t Uniformity_base[MAX_CHANNEL_NUM] =
{
    5000, 5000, 5000, 5000, 5000, 5000,
    5000, 5000, 5000, 5000, 5000, 5000,
    5000, 5000, 5000, 5000, 5000, 5000,
    5000, 5000, 5000, 5000, 5000, 5000,
    5000, 5000, 5000, 5000, 5000, 5000,
    5000, 5000, 5000, 5000, 5000, 5000,
};




static uint16_t raw_wo_pressure[MAX_CHANNEL_NUM] = { 0 };
static uint16_t touch_diff[MAX_CHANNEL_NUM] = { 0 };


static int StartScan( void )
{
	int ret = 0;
	int i = 0;
	uint8_t val = 0;
	
	ret = touch_write_reg(0x08, 0x01);
	if (ret < 0) {
		LOG("write reg08 fails\n");
		return ret;
	}
	
	for (i = 0; i < 400; i++) {
		FT3308_DELAY_MS( 10 );
		val = 0xFF;
		ret = touch_read_reg(0x08, &val);
		if ((val == 0) && (ret == 0))
			return 0;
		else
			LOG("read reg%x,val:%x,ret:%d,retry:%d\n", 0x08, val, ret, i);
	}
	
	return -1;
}


static int fts_enter_factory_mode(void)
{
	int ret = 0;
	int i = 0;
	uint8_t val = 0xFF;
	
	for(i = 0; i < 2; i++)
	{
		ret = touch_write_reg(0x00, 0x40);
		if (ret < 0) {
			LOG("write reg0x00 fails\n");
			return ret;
		}
		FT3308_DELAY_MS(1);
	}

	// FT3308_DELAY_MS(10);

	for (i = 0; i < 20; i++)
	{
		ret = touch_read_reg(0x00, &val);
		if (val == 0x40) 
		{
			LOG("enter into factory mode %d\n",i);
			FT3308_DELAY_MS(20); 
			return 0;
		}
		FT3308_DELAY_MS(20);
	}

    LOG("reg00:%x\n",val);
	return -1;
}

static int fts_enter_work_mode(void)
{
	int ret = 0;
	int i = 0;
	uint8_t val = 0xFF;
	
	ret = touch_read_reg(0x00, &val);
	if (val == 0x00) {
		LOG("already in factory mode\n");
		return 0;
	}
	
	ret = touch_write_reg(0x00, 0x00);
	if (ret < 0) {
		LOG("write reg0x00 fails\n");
		return ret;
	}
	
	for (i = 0; i < 20; i++) {
		val = 0xFF;
		ret = touch_read_reg(0x00, &val);
		if (val == 0x00) {
			LOG("enter into work mode\n");
			FT3308_DELAY_MS(20); 
			return 0;
		}
		FT3308_DELAY_MS(20);
	}
	
	return -1;
}

static int fts_get_rawdata(uint16_t *raw)
{
	int ret = 0;
	int i = 0;
	uint8_t raw_addr = 0x35;
	uint8_t tmp_data[MAX_CHANNEL_NUM * 2] = { 0 };

	ret = StartScan();
	if (ret < 0) {
		LOG("star scan fails\n");
		return ret;
	}
	
	ret = touch_write_reg(0x34, 00);
	if (ret < 0) {
		LOG("write reg34 fails\n");
		return ret;
	}
	
	ret = touch_read(&raw_addr, 1, tmp_data, MAX_CHANNEL_NUM * 2);
	if (ret < 0) {
		LOG("read raw fails\n");
		return ret;
	}
	
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		raw[i] = (tmp_data[i * 2] << 8) + tmp_data[i * 2 + 1];
	}
	
	return 0;
}

static int fts_get_cb(uint16_t *cb, uint8_t mode)
{
	int ret = 0;
	int i = 0;
	uint8_t cb_addr = 0x39;
	uint8_t tmp_data[MAX_CHANNEL_NUM * 2] = { 0 };

	ret = StartScan();
	if (ret < 0) {
		LOG("star scan fails\n");
		return ret;
	}
	
	ret = touch_write_reg(0x33, 00);
	if (ret < 0) {
		LOG("write reg33 fails\n");
		return ret;
	}
	
	ret = touch_read(&cb_addr, 1, tmp_data, MAX_CHANNEL_NUM * 2);
	if (ret < 0) {
		LOG("read cb fails\n");
		return ret;
	}
	
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		cb[i] = (tmp_data[i * 2] << 8) + tmp_data[i * 2 + 1];
	}
	
	return 0;
}

static int fts_get_raw_base(void)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    uint16_t raw[MAX_CHANNEL_NUM] = { 0 };

    memset(raw_wo_pressure, 0, MAX_CHANNEL_NUM * sizeof(uint16_t));
    
    for (i = 0; i < 3; i++)
        StartScan();

    for (i = 0; i < 3; i++) {
        ret = fts_get_rawdata(raw);
    	if (ret < 0) {
    		LOG("get raw(base) fails\n");
    		return ret;
    	}
    	
    	for (j = 0; j < MAX_CHANNEL_NUM; j++) {
            raw[j] = raw[j] / 3;
            raw_wo_pressure[j] += raw[j];
    	}
    }

    return 0;
}

/*测试PASS的条件是返回值==0，并且bTestResult==true*/
int TestItem_RawDataTest(uint8_t *buf, bool is_half, bool *bTestResult)
{
	int ret = 0;
	int i = 0;
	uint16_t raw[MAX_CHANNEL_NUM] = { 0 };
	const uint16_t *min;
	const uint16_t *max;
	
	/*rawdata type*/
	buf[0] = 0x01;

    for (i = 0; i < 3; i++)
        StartScan();
	
	ret = fts_get_rawdata(raw);
	if (ret < 0) {
		LOG("get raw fails\n");
		return ret;
	}

	*bTestResult = true;
	/*compare*/
	if (is_half) {
		min = HalfRawMin;
		max = HalfRawMax;
	} else {
		min = RawMin;
		max = RawMax;
	}
	for ( i = 0; i < MAX_CHANNEL_NUM; i++) {
		if ((raw[i] < min[i]) || (raw[i] > max[i])) {
			LOG("ch:%d, raw:%d,min:%d,max:%d\n", i, raw[i], min[i], max[i]);
			*bTestResult = false;
		}
	}
	
	buf[1] = (*bTestResult) ? 0x01 : 0x02;
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		buf[2 + i * 2 + 1] = raw[i];
		buf[2 + i * 2] = (raw[i] >> 8);
	}

	LOG("raw:");
	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		LOG("%d ", raw[i]);
	LOG("\r\n");
	
	return 0;
}

/*测试PASS的条件是返回值==0，并且bTestResult==true*/
int TestItem_CbTest(uint8_t *buf, bool is_half, bool *bTestResult)
{
	int ret = 0;
	int i = 0;
	uint16_t cb[MAX_CHANNEL_NUM] = { 0 };
	const uint16_t *min;
	const uint16_t *max;
	
	buf[0] = 0x02;

	ret = fts_get_cb(cb, 0);
	if (ret < 0) {
		LOG("get cb fails\n");
		return ret;
	}
	
	*bTestResult = true;
	/*compare*/
	if (is_half) {
		min = HalfCbMin;
		max = HalfCbMax;
	} else {
		min = CbMin;
		max = CbMax;
	}
	for ( i = 0; i < MAX_CHANNEL_NUM; i++) {
		if ((cb[i] < min[i]) || (cb[i] > max[i])) {
			LOG("ch:%d, cb:%d,min:%d,max:%d\n", i, cb[i], min[i], max[i]);
			*bTestResult = false;
		}
	}
	
	buf[1] = (*bTestResult) ? 0x01 : 0x02;
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		buf[2 + i * 2 + 1] = cb[i];
		buf[2 + i * 2] = (cb[i] >> 8);
	}

	LOG("cb:");
	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		LOG("%d ", cb[i]);
	LOG("\r\n");
	return 0;
}

/*测试PASS的条件是返回值==0，并且bTestResult==true*/
int TestItem_ShortTest(uint8_t *buf, bool is_half, bool *bTestResult)
{
	int ret = 0;
	int i = 0;
	uint8_t reg0d_val = 0;
	uint16_t short_data[MAX_CHANNEL_NUM] = { 0 };
	const uint16_t *min;
	const uint16_t *max;
	
	buf[0] = 0x03;
	
	ret = touch_read_reg(0x0D, &reg0d_val);
	if (ret < 0) {
		LOG("read reg0D fails\n");
		return ret;
	}
	
	ret = touch_write_reg(0x0D, 0x0F);
	if (ret < 0) {
		LOG("write reg0D fails\n");
		return ret;
	}
	
	ret = touch_write_reg(0xAE, 2);
	if (ret < 0) {
		LOG("write regAE fails\n");
		return ret;
	}
	FT3308_DELAY_MS(200);
	
	ret = fts_get_cb(short_data, 2);
	if (ret < 0) {
		LOG("get cb fails\n");
		goto err_short;
	}
	
	*bTestResult = true;
	/*compare*/
	if (is_half) {
		min = HalfShortMin;
		max = HalfShortMax;
	} else {
		min = ShortMin;
		max = ShortMax;
	}
	for ( i = 0; i < MAX_CHANNEL_NUM; i++) {
		if ((short_data[i] < min[i]) || (short_data[i] > max[i])) {
			LOG("ch:%d, short:%d,min:%d,max:%d\n", i, short_data[i], min[i], max[i]);
			*bTestResult = false;
		}
	}
	
	buf[1] = (*bTestResult) ? 0x01 : 0x02;
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		buf[2 + i * 2 + 1] = short_data[i];
		buf[2 + i * 2] = (short_data[i] >> 8);
	}

	LOG("short_data:");
	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		LOG("%d ", short_data[i]);
	LOG("\r\n");
	
	ret = 0;
	
err_short:
	ret = touch_write_reg(0x0D, reg0d_val);
    if (ret < 0) {
		LOG("restore regAE fails\n");
	}
	
    
	ret = touch_write_reg(0xAE, 0x00);
	if (ret < 0) {
		LOG("write 00 to regAE fails\n");
	}
	FT3308_DELAY_MS(200);

    fts_get_raw_base();

	return ret;
}


/*测试PASS的条件是返回值==0，并且bTestResult==true*/
int TestItem_TouchDiffTest(uint8_t *buf, bool *bTestResult)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	uint16_t raw[MAX_CHANNEL_NUM] = { 0 };
	uint16_t *diff = touch_diff;

	/*rawdata type*/
	buf[0] = 0x04;
	
	for (i = 0; i < RAW_NUM_P; i++) {
		ret = fts_get_rawdata(raw);
		if (ret < 0) {
			LOG("get raw fails\n");
			return ret;
		}
		
		for (j = 0; j < MAX_CHANNEL_NUM; j++) {
			if (diff[j] < raw[j])
				diff[j] = raw[j];
		}
	}
	
	LOG("max raw:");
	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		LOG("%d ", diff[i]);
	LOG("\r\n");
	LOG("ori raw:");
	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		LOG("%d ", raw_wo_pressure[i]);
	LOG("\r\n");
	
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		if (diff[i] >= raw_wo_pressure[i])
			diff[i] = diff[i] - raw_wo_pressure[i];
		else
			diff[i] = raw_wo_pressure[i] - diff[i];
	}
	
	*bTestResult = true;
	/*compare*/
	for ( i = 0; i < MAX_CHANNEL_NUM; i++) {
		if ((diff[i] < diff_min[i]) || (diff[i] > diff_max[i])) {
			LOG("ch:%d, diff:%d,min:%d,max:%d\n", i, diff[i], diff_min[i], diff_max[i]);
			*bTestResult = false;
		}
	}
	
	buf[1] = (*bTestResult) ? 0x01 : 0x02;
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		buf[2 + i * 2 + 1] = diff[i];
		buf[2 + i * 2] = (diff[i] >> 8);
	}

	LOG("diff:");
	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		LOG("%d ", diff[i]);
	LOG("\r\n");
	
	return 0;
}

/*测试PASS的条件是返回值==0，并且bTestResult==true*/
int TestItem_TouchUinformityTest(uint8_t *buf, bool *bTestResult)
{
//	int ret = 0;
	int i = 0;
//	int j = 0;
	int16_t uniformity[MAX_CHANNEL_NUM] = { 0 };

	/*rawdata type*/
	buf[0] = 0x05;

	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		uniformity[i] = (touch_diff[i] - Uniformity_base[i]) * 100 / Uniformity_base[i];
	}
	
	*bTestResult = true;
	/*compare*/
	for ( i = 0; i < MAX_CHANNEL_NUM; i++) {
		if ((uniformity[i] < UniformityMin[i]) || (uniformity[i] > UniformityMax[i])) {
			LOG("ch:%d, uniformity:%d,min:%d,max:%d\n", i, uniformity[i], UniformityMin[i], UniformityMax[i]);
			*bTestResult = false;
		}
	}
	
	buf[1] = (*bTestResult) ? 0x01 : 0x02;
	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		buf[2 + i * 2 + 1] = uniformity[i];
		buf[2 + i * 2] = (uniformity[i] >> 8);
	}

	LOG("uniformity:");
	for (i = 0; i < MAX_CHANNEL_NUM; i++)
		LOG("%d ", uniformity[i]);
	LOG("\r\n");
	
	return 0;
}


//半成品测试
void half_product_test(uint8_t *buf) 
{
	int ret = 0;
	bool tmp_result = false;
	uint8_t result = 0;
	uint8_t *tmp_buf;
	
	ret = fts_enter_factory_mode();
	if (ret < 0) {
		LOG("enter factory mode fails\n");
		return ;
	}
	
	result |= (1 << 0);
	tmp_buf = buf;
	ret = TestItem_RawDataTest(tmp_buf, true, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("raw test fails\n");
		result &= (~(1 << 0));
	}
	
	result |= (1 << 1);
	tmp_buf = buf + 74;
	ret = TestItem_CbTest(tmp_buf, true, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("cb test fails\n");
		result &= (~(1 << 1));
	}
	
	result |= (1 << 2);
	tmp_buf = buf + 74 * 2;
	ret = TestItem_ShortTest(tmp_buf, true, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("short test fails\n");
		result &= (~(1 << 2));
	}
	
	ret = fts_enter_work_mode();
	if (ret < 0) {
		LOG("enter work mode fails\n");
	}
}

//成品非压力测试
void product_no_pressure_test(uint8_t *buf)
{
	int ret = 0;
	bool tmp_result = false;
	uint8_t result = 0;
	uint8_t *tmp_buf;
	
	ret = fts_enter_factory_mode();
	if (ret < 0) {
		LOG("enter factory mode fails\n");
		return ;
	}
	
	result |= (1 << 0);
	tmp_buf = buf;
	ret = TestItem_RawDataTest(tmp_buf, false, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("raw test fails\n");
		result &= (~(1 << 0));
	}
	
	result |= (1 << 1);
	tmp_buf = buf + 74;
	ret = TestItem_CbTest(tmp_buf, false, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("cb test fails\n");
		result &= (~(1 << 1));
	}
	
	result |= (1 << 2);
	tmp_buf = buf + 74 * 2;
	ret = TestItem_ShortTest(tmp_buf, false, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("short test fails\n");
		result &= (~(1 << 2));
	}
}

//成品压力测试
void product_pressure_test(uint8_t *buf)
{
	int ret = 0;
	bool tmp_result = false;
	uint8_t result = 0;
	uint8_t *tmp_buf;
	
	result |= (1 << 0);
	tmp_buf = buf;
	ret = TestItem_TouchDiffTest(tmp_buf, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("touch diff test fails\n");
		result &= (~(1 << 0));
	}
	
	result |= (1 << 1);
	tmp_buf = buf + 74;
	ret = TestItem_TouchUinformityTest(tmp_buf, &tmp_result);
	if ((ret != 0) || (tmp_result == false)) {
		LOG("touch diff uniformity test fails\n");
		result &= (~(1 << 1));
	}


	ret = fts_enter_work_mode();
	if (ret < 0) {
		LOG("enter work mode fails\n");
	}
}

#endif
