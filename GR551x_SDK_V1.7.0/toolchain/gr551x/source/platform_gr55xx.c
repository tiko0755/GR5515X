/**
 *******************************************************************************
 *
 * @file   platform_gr55xx.c
 *
 * @brief  Platform Initialization Routines.
 *
 *******************************************************************************
 
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

/*
 * INCLUDE FILES
 *******************************************************************************
 */
#include "gr55xx_hal_cortex.h"
#include "gr55xx.h"
#include "gr55xx_sys.h"
#include "gr55xx_ll_pwr.h"
#include "hal_flash.h"
#include "platform_sdk.h"
#include "pmu_calibration.h"
#include "boards.h"
#include "custom_config.h"
#include "patch.h"
#include "patch_tab.h"
#include "gr55xx_ll_gpio.h"
#include "gr55xx_hal.h"

// NOTE: SVC #0 is reserved for freertos, DO NOT USE IT!
#define SVC_TABLE_NUM_MAX   4

#define FLASH_CS        (LL_GPIO_PIN_2)      /* XQSPI flash CS        */
#define FLASH_CLK       (LL_GPIO_PIN_4)      /* XQSPI flash CLK       */
#define FLASH_IO_0      (LL_GPIO_PIN_7)      /* XQSPI flash IO0       */
#define FLASH_IO_1      (LL_GPIO_PIN_6)      /* XQSPI flash IO1       */
#define FLASH_IO_2      (LL_GPIO_PIN_5)      /* XQSPI flash IO2 (WP)  */
#define FLASH_IO_3      (LL_GPIO_PIN_3)      /* XQSPI flash IO3 (HOLD)*/
#define HAL_EXFLASH_IO_PULL_SET(_PIN_, _PULL_)  ll_gpio_set_pin_pull(GPIO1, _PIN_, _PULL_)

extern uint8_t nvds_put_patch(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);
extern uint8_t nvds_put_rom(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);
extern void dfu_cmd_handler_replace_for_encrypt(void);
extern void register_rwip_reset(void (*callback)(void));
extern void rwip_reset_patch(void);
extern void register_rwip_init(void (*callback)(uint32_t));
extern void rwip_init_patch(uint32_t error);

#define MAX_COMMENTS_CNT 12
typedef struct __attribute((packed))
{
    uint32_t    app_pattern;
    uint32_t    app_info_version;
    uint32_t    chip_ver;
    uint32_t    load_addr;
    uint32_t    run_addr;
    uint32_t    app_info_sum;
    uint8_t     check_img;
    uint8_t     boot_delay;
    uint8_t     sec_cfg;
    uint8_t     reserved0;
    uint8_t     comments[MAX_COMMENTS_CNT];
    uint32_t    reserved1[6];
} APP_INFO_t;

#define APP_INFO_ADDR           (APP_CODE_RUN_ADDR+0x200)
#define APP_INFO_PATTERN_VALUE  0x47525858
#define APP_INFO_VERSION        0x1
#define CHECK_SUM               (APP_INFO_PATTERN_VALUE+APP_INFO_VERSION+CHIP_VER+APP_CODE_LOAD_ADDR+APP_CODE_RUN_ADDR)

#if defined (__CC_ARM )
const APP_INFO_t BUILD_IN_APP_INFO __attribute__((at(APP_INFO_ADDR))) =
#elif defined (__ICCARM__)
__root const APP_INFO_t BUILD_IN_APP_INFO @ (APP_INFO_ADDR) =
#else
const APP_INFO_t BUILD_IN_APP_INFO __attribute__((section(".app_info"))) =
#endif
{
    .app_pattern      = APP_INFO_PATTERN_VALUE,
    .app_info_version = APP_INFO_VERSION,
    .chip_ver         = CHIP_VER,
    .load_addr        = APP_CODE_LOAD_ADDR,
    .run_addr         = APP_CODE_RUN_ADDR,
    .app_info_sum     = CHECK_SUM,
    .check_img        = BOOT_CHECK_IMAGE,
    .boot_delay       = BOOT_LONG_TIME,
    .sec_cfg          = SECURITY_CFG_VAL,
#ifdef APP_INFO_COMMENTS
    .comments         = APP_INFO_COMMENTS,
#endif
};

static uint32_t SVC_TABLE_USER_SPACE[SVC_TABLE_NUM_MAX] __attribute__((section("SVC_TABLE")));

#if (CFG_LCP_SUPPORT && (CHIP_TYPE == 0))
extern uint16_t gdx_lcp_buf_init(uint32_t buf_addr);
static uint8_t lcp_buf[280] __attribute__((section (".ARM.__at_0x00820000"), zero_init));
#endif

static void nvds_setup(void)
{

#ifdef NVDS_START_ADDR
    uint8_t err_code = nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    uint8_t err_code = nvds_init(0, NVDS_NUM_SECTOR);
#endif

    switch(err_code)
    {
        case NVDS_FAIL:
        case NVDS_STORAGE_ACCESS_FAILED:
            {
                uint32_t start_addr  = nvds_get_start_addr();
                uint32_t sector_size = hal_flash_sector_size();
                if (hal_flash_erase(start_addr, NVDS_NUM_SECTOR * sector_size))
                {
                    err_code = nvds_init(start_addr, NVDS_NUM_SECTOR);
                    if (NVDS_SUCCESS == err_code)
                    {
                        break;
                    }
                }
                /* Flash fault, cannot startup.
                 * TODO: Output log via UART or Dump an error code to flash. */
                while(1);
            }
        case NVDS_SUCCESS:
            break;
        default:
            /* Illegal NVDS Parameters.
             * Please check the start address and number of sectors. */
            while(1);
    }
}

void ble_sdk_env_init(void)
{
#if (CFG_MAX_CONNECTIONS < 3) && (CFG_MAX_ADVS < 2) && (CFG_MESH_SUPPORT < 1)
    register_rwip_reset(rwip_reset_patch);
    register_rwip_init(rwip_init_patch);
#endif

    // register the msg handler for patch
    uint16_t msg_cnt = sizeof(msg_tab) / sizeof(msg_tab_item_t);
    reg_msg_patch_tab(msg_tab, msg_cnt);

    // register the hci cmd handler for patch
    uint16_t hci_cmd_cnt = sizeof(hci_cmd_tab) / sizeof(hci_cmd_tab_item_t);
    reg_hci_cmd_patch_tab(hci_cmd_tab, hci_cmd_cnt);

    // register the gapm hci evt handler for patch
    uint16_t hci_evt_cnt = sizeof(gapm_hci_evt_tab) / sizeof(gapm_hci_evt_tab_item_t);
    reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab, hci_evt_cnt);

#if CFG_MAX_CONNECTIONS
    ble_con_env_init();
#endif

#if CFG_MAX_SCAN
    ble_scan_env_init();
#endif

#if CFG_MAX_ADVS
    ble_adv_env_init();
#endif
}

static void exflash_io_pull_config(void)
{
    /* XQSPI IO configuration needs to match Flash. 
       The default configuration can match most Flash */
    HAL_EXFLASH_IO_PULL_SET(FLASH_CS,   LL_GPIO_PULL_UP);
    HAL_EXFLASH_IO_PULL_SET(FLASH_CLK,  LL_GPIO_PULL_NO);
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_0, LL_GPIO_PULL_UP); /* MOSI */
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_1, LL_GPIO_PULL_UP); /* MISO */
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_2, LL_GPIO_PULL_UP); /* WP   */
    HAL_EXFLASH_IO_PULL_SET(FLASH_IO_3, LL_GPIO_PULL_UP); /* HOLD */
}

void platform_init(void)
{
    /* if BLE not fully power off, reset and power off it manually */
    if((AON->PWR_RET01 & AON_PWR_REG01_PWR_EN_PD_COMM_TIMER) || \
       (AON->PWR_RET01 & AON_PWR_REG01_PWR_EN_PD_COMM_CORE))
    {
        ll_pwr_enable_comm_core_reset();
        ll_pwr_enable_comm_timer_reset();
        ll_pwr_disable_comm_core_power();
        ll_pwr_disable_comm_timer_power();
        /* TODO: Reserve System Cold Fully Reset Method. */
        // hal_nvic_system_reset();
    }

    /* Clear All Wakeup Event When Cold Boot */
    ll_pwr_clear_wakeup_event(LL_PWR_WKUP_EVENT_ALL);
    for(uint8_t i = 0; i < MAX_NUMS_IRQn; i++)
    {
        NVIC_ClearPendingIRQ((IRQn_Type)(i));
    }

    #ifdef EXFLASH_WAKEUP_DELAY
    warm_boot_set_exflash_readid_delay(EXFLASH_WAKEUP_DELAY * 5);
    run_mode_t run_mode = (run_mode_t)(SYSTEM_CLOCK);
    uint16_t osc_time = ble_wakeup_osc_time_get(run_mode) + (EXFLASH_WAKEUP_DELAY * 5);
    ble_wakeup_osc_time_set(run_mode, osc_time);
    #endif

    /* enable protection. */
    platform_init_push();

    /* set sram power state. */
    mem_pwr_mgmt_init();

    /* nvds module init process. */
    nvds_setup();

    /* To choose the System clock source and set the accuracy of OSC. */
    #if CFG_LPCLK_INTERNAL_EN
    platform_clock_init_rng((mcu_clock_type_t)SYSTEM_CLOCK, RNG_OSC_CLK2, 500, 0);
    #else
    #if CFG_CRYSTAL_DELAY
    platform_set_rtc_crystal_delay(CFG_CRYSTAL_DELAY);
    #endif
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RTC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
    #endif

    /* Register the SVC Table. */
    svc_table_register(SVC_TABLE_USER_SPACE);

#if ENCRYPT_ENABLE 
    fpb_register_patch_init_func(fpb_encrypt_mode_patch_enable);
#else
    fpb_register_patch_init_func(fpb_patch_enable);
#endif

    /* platform init process. */ 
    platform_sdk_init();

#if ENCRYPT_ENABLE
    dfu_cmd_handler_replace_for_encrypt();
#endif

    #ifndef DRIVER_TEST
    /* Enable auto pmu calibration function period =3s on default. */
    system_pmu_calibration_init(30000);
    #endif

    system_pmu_deinit();
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);
    system_pmu_init((mcu_clock_type_t)SYSTEM_CLOCK);

    // recover the default setting by temperature, should be called in the end
    pmu_calibration_handler(NULL);

    /* RTC calibration function */
    #if !CFG_LPCLK_INTERNAL_EN
    /* Delayed for a while, because the GM of RTC has changed */
    #if CFG_CRYSTAL_DELAY
    delay_ms(CFG_CRYSTAL_DELAY);
    #endif
    rtc_calibration();
    #endif

    /* rng calibration */
    rng_calibration();

    #if (CFG_LCP_SUPPORT && (CHIP_TYPE == 0))
    gdx_lcp_buf_init((uint32_t)lcp_buf);
    #endif

    exflash_io_pull_config();

    /* disable protection. */
    platform_init_pop();

    return;
}

extern void system_platform_init(void);
extern int main(void);
#if defined ( __GNUC__ )
void __main(void)
{
    __asm("ldr    r1, =__etext\n");
    __asm("ldr    r2, =__data_start__\n");
    __asm("ldr    r3, =__data_end__\n");
    __asm(".L_loop1:\n");
    __asm("cmp    r2, r3\n");
    __asm("ittt   lt\n");
    __asm("ldrlt  r0, [r1], #4\n");
    __asm("strlt  r0, [r2], #4\n");
    __asm("blt    .L_loop1\n");
    __asm("ldr    r1, =__bss_start__\n");
    __asm("ldr    r2, =__bss_end__\n");
    __asm("movs   r0, 0\n");
    __asm(".L_loop3:\n");
    __asm("cmp    r1, r2\n");
    __asm("itt    lt\n");
    __asm("strlt  r0, [r1], #4\n");
    __asm("blt    .L_loop3\n");
    system_platform_init();
    main();
}
#endif

#if defined ( __CC_ARM )
//lint -e{10}
extern void $Super$$main(void);
//lint -e{10,144}
void $Sub$$main(void)
{
    system_platform_init();
    $Super$$main();
}
#endif



#if defined ( __ICCARM__ )

extern void __iar_program_start(void);
void __main(void)
{
    __iar_program_start();
}

extern void __iar_data_init3(void);
int __low_level_init(void)
{
    // call IAR table copy function.
    __iar_data_init3();
    system_platform_init();
    return 0;
}
#endif

