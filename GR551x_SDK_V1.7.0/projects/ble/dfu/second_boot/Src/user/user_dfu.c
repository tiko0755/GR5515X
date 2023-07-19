#include "user_dfu.h"
#include "user_app.h"
#include "app_log.h"
#include "dfu_port.h"

static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);


static dfu_pro_callback_t dfu_pro_call = 
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback    = dfu_programing_callback,
    .dfu_program_end_callback   = dfu_program_end_callback,
};

static void dfu_program_start_callback(void)
{
    APP_LOG_DEBUG("Start DFU OTA.");
}

static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("DFU OTA.... %d%%", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    APP_LOG_DEBUG("DFU OTA complete.");
}


void user_dfu_init(void)
{
    dfu_port_init(NULL, &dfu_pro_call);
    dfu_service_init(NULL);
}
