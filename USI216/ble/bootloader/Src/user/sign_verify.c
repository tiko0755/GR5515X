/**
 *****************************************************************************************
 *
 * @file sign_verify.c
 *
 * @brief Second boot function Implementation.
 *
 *****************************************************************************************
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
 *****************************************************************************************
 */


/*
 * INCLUDE FILES
 *****************************************************************************************
 */
//

#include "gr55xx_hal.h"
#include "gr55xx_sys.h"
#include "user_config.h"

#ifdef BOOTLOADER_SIGN_ENABLE

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

#define APP_FLASH_START_ADDR               (0x01002000)
#define APP_FLASH_END_ADDR                 (0x01100000)


#define FLASH_PAGE_SIZE         4096
#define AES_BLOCK_SIZE          16
#define SHA_BLOCK_SIZE          64
#define HMAC_BLOCK_SIZE         64
#define HMAC_SIZE               32
#define SHA256_SIZE             32
#define SIGNATURE_SIZE          256
#define RSA_KEY_SIZE            256
#define RSA_PUBLIC_KEY_SIZE     (2*RSA_KEY_SIZE + 8) /*N(256)+e(4)+RR(256)+C(4)*/
#define ECIES_PUBLIC_KEY_SIZE   64
#define ZERO_PADDING_SIZE       8
#define BALBOA_REGS_BASE_ADDR_SEC              (0xA0010000)
#define REGS_PCK_BASE_ADDR                     (BALBOA_REGS_BASE_ADDR_SEC + 0x4000)
#define ECC_U32_LENGTH         (8)

typedef enum {
    GM_DRV_OK =                  0,                     ///< Operation succeeded

    /*common error code start*/
    GM_DRV_ERROR =              -10000,                 ///< Unspecified error
    GM_DRV_ERROR_BUSY =         -10001,                 ///< Driver is busy
    GM_DRV_ERROR_TIMEOUT =      -10002,                 ///< Timeout occurred
    GM_DRV_ERROR_UNSUPPORTED =  -10003,                 ///< Operation not supported
    GM_DRV_ERROR_PARAMETER  =   -10004,                 ///< Parameter error
    GM_DRV_ERROR_POWER_OFF  =   -10005,                 ///< Start of driver specific errors
    GM_DRV_ERROR_CONFIG = -10006,                       ///< Driver config error
    GM_DRV_ERROR_RETRY      =   -10007,                 ///< Retry operation
    GM_DRV_ERROR_NOT_READY  =   -10008,                 ///< Not ready
    /*common error code end*/

    /*SPIM error code start*/
    GM_DRV_SPIM_ERROR_MODE = -10101,                    ///< SPIM mode error, Master Mode supported only
    /*SPIM error code end*/

    /*MTP error code start*/
    GM_DRV_MTP_ERROR_ADDRESS_NOT_4BYTE_ALIGN = -10204,  ///< MTP address error, Address is not 4 bytes aligned
    GM_DRV_MTP_ERROR_ADDRESS_NOT_AVAILABLE = -10205,    ///< MTP address error, Address can not accessed
    /*MTP error code end*/

    /*AES error code start*/
    GM_DRV_AES_TRANSFER_ERROR  = -10300,                ///< AES Transfer error flag.
    GM_DRV_AES_ALIGN_ERROR     = -10301,                ///< Input/output data not aligned to 16bytes
    GM_DRV_AES_INPUT_ERROR     = -10302,                ///< Input parameters is error.
    /*AES error code end*/

    /*SHA error code start*/
    GM_DRV_SHA_ERROR_STATUS    = -10400,                ///< SHA execution error
    GM_DRV_SHA_ERROR_INPUT     = -10401,                ///< SHA parameters error
    /*SHA error code end*/

    /*UART error code start*/
    GM_DRV_UART_ERROR_PARITY  = -10500,                 ///< UART parity error
    GM_DRV_UART_ERROR_FRAMING = -10501,                 ///< UART frame error
    GM_DRV_UART_ERROR_PARITY_FRAMING = -10502,          ///< UART parity frame error in the same time
    /*UART error code end*/

    /*I2C error code start*/
    GM_DRV_I2C_ERROR_NACK     = -10600,                 ///< No ACK received error
    /*I2C error code end*/

    /*AES error code start*/
    GM_DRV_PRESENT_TRANSFER_ERROR  = -10300,                ///< AES Transfer error flag.
    GM_DRV_PRESENT_ALIGN_ERROR     = -10301,                ///< Input/output data not aligned to 16bytes
    GM_DRV_PRESENT_INPUT_ERROR     = -10302,                ///< Input parameters is error.
    /*AES error code end*/

    GM_DRV_ECC_PARAMETER_ERROR    = -10800,                ///< ECC ECDSA verify algorithm error
    GM_DRV_ECC_INTERRUPT_ERROR    = -10801,                ///< ECC interrupt error flag is raised
    GM_DRV_ECC_ECDSA_SIGN_ERROR   = -10802,                ///< ECC ECDSA sign algorithm error
    GM_DRV_ECC_ECDSA_VERIFY_ERROR = -10803,                ///< ECC ECDSA verify algorithm error
    GM_DRV_ECC_MONTGOMERY_INVERSE_K_ERROR = -10804,        ///< ECC Montgomery inverse output k is not in range
    GM_DRV_ECC_NOT_IRREVERSIBLE   = -10805,                ///< ECC Montgomery/Modular irreversible
    GM_DRV_ECC_CURVE_UNSUPPORTED  = -10806,                ///< ECC Curve a!=-3 mod p(NIST - P256)
    GM_DRV_ECC_RNG_ERROR  = -10807,                        ///< ECC RNG module not found or returns error
    GM_DRV_ECC_DEVICE_BUSY  = -10808,                      ///< ECC Device is busy, check if it is released
    GM_DRV_ECC_CURVE_CRC_ERROR  = -10809,                  ///< ECC Curve CRC Failed!
    GM_DRV_ECC_POINT_NOT_ON_CURVE =  -10810,               ///< ECC Input point is not on the curve

    /*EFUSE error code start*/
    GM_DRV_EFUSE_ERROR = -10900,
   /*EFUSE error code end*/


    /*Bootloader error code start*/
    GM_BL_OK = 0,
    GM_BL_ERROR_CRC32     =         -20001,
    GM_BL_ERROR_INFO       =            -20002,
    GM_BL_ERROR_KEY       =             -20003,
    GM_BL_ERROR_FW          =           -20004,
    GM_BL_ERROR_INVALID_FW_PUBLIC_KEY    =    -20005,
    GM_BL_ERROR_ROLLBACK      =         -20006,
    GM_BL_ERROR_SIGNATURE     =         -20007,
    GM_BL_ERROR_PRODUCT_ID  =           -20008,
    GM_BL_ERROR_INITIALIZED     =       -20009,
    GM_BL_ERROR_INVALID_UPGRADE_MODE  =   -20010,
    GM_BL_ERROR_TIMEOUT        =        -20011,
    GM_BL_ERROR_FW_HASH      =          -20012,
    GM_BL_ERROR_TRIM            =       -20013,
    GM_BL_ERROR_FLASH_INIT    =         -20014,
    GM_BL_ERROR_FLASH_READ    =         -20015,
    GM_BL_ERROR_FLASH_WRITE     =       -20016,
    GM_BL_ERROR_FLASH_ERASE       =     -20017,
    GM_BL_ERROR_ECIES           =       -20018,

    GM_BL_ERROR_ECIES_BAD_INPUT_DATA  = -20019,
    GM_BL_ERROR_ECIES_INVALID_PADDING = -20020,
    GM_BL_ERROR_ECIES_VERIFY_FAILED   = -20021,

    GM_BL_ERROR_RSA_BAD_INPUT_DATA  = -20022,
    GM_BL_ERROR_RSA_INVALID_PADDING = -20023,
    GM_BL_ERROR_RSA_VERIFY_FAILED   = -20024,

    GM_BL_ERROR_VBS          =          -20030,
    GM_BL_ERROR_VBS_WRITE_FLASH    =    -20031,

    GM_BL_ERROR_UPGRADE_INIT        =   -20040,
    GM_BL_ERROR_UPGRADE_OVERFLOW    =   -20041,
    GM_BL_ERROR_UPGRADE_CRC32       =   -20042,
    GM_BL_ERROR_UPGRADE_INVALID_CMD =   -20043,
    GM_BL_ERROR_UPGRADE_NOT_CONNECT =   -20044,
    GM_BL_UPGRADE_PROGRAM_FAILED    =   -20049,
    /*Bootloader error code end*/
    GM_BL_ERROR_INVALID_RANDOM            =   -20050,

}gm_drv_ret_e;


typedef struct _gm_ecc_point
{
    uint32_t x[ECC_U32_LENGTH];
    uint32_t y[ECC_U32_LENGTH];
} gm_ecc_point_t;


/**
 * \brief An ecliptic curve description.  E: y^2 = x^3 + ax + b mod p.
 *  Currently only NIST P256 is supported.
 */
typedef struct _gm_ecc_parameter
{
    uint32_t a[ECC_U32_LENGTH];  ///< parameter a. For NIST P256 curve, must satisfy a = p - 3.
    uint32_t b[ECC_U32_LENGTH];  ///< parameter b. b = {0x5AC635D8, 0xAA3A93E7, 0xB3EBBD55, 0x769886BC, 0x651D06B0, 0xCC53B0F6, 0x3BCE3C3E, 0x27D2604B}

    uint32_t p[ECC_U32_LENGTH];  ///< paramete prime p = {0xffffffff, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff}
    uint32_t p_r_square[ECC_U32_LENGTH]; ///< R^2 mod p : {0x00000004, 0xfffffffd, 0xffffffff, 0xfffffffe, 0xfffffffb, 0xffffffff, 0x00000000, 0x00000003}
    uint32_t constp;                     ///< montgomery multiplication constant in prime field p, constp = 1

    uint32_t n[ECC_U32_LENGTH];          ///< order of the generated cylic point set: n = {0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xbce6faad, 0xa7179e84, 0xf3b9cac2, 0xfc632551}
    uint32_t n_r_square[ECC_U32_LENGTH]; ///< R^2 mod n : {0x66e12d94, 0xf3d95620, 0x2845b239, 0x2b6bec59, 0x4699799c, 0x49bd6fa6, 0x83244c95, 0xbe79eea2}
    uint32_t constn;                     ///< montgomery multiplication constant in prime field  n, constn = 0xee00bc4f
    uint32_t h;                ///< parameter h = 1

    gm_ecc_point_t G;           ///< generation point G

    uint32_t crc;              ///<curve crc value

} gm_ecc_parameter_t;

/**
 * \brief An ecliptic Diffie Hellman description
 */
typedef struct _gm_ecc_Diffie_Hellman
{
    uint32_t host_id;                  ///< host identification
    uint32_t k_secret[ECC_U32_LENGTH]; ///< our secrete value

    gm_ecc_point_t Q_us;      ///< our public value, need to send to peer: Q_us = k_secret * G
    gm_ecc_point_t Q_peer;    ///< peer's public value.
    gm_ecc_point_t Q_shared;  ///< shared secret: Q_shared = k_secret * Q_peer

    gm_ecc_parameter_t *ecc_curve; ///< pointer to ecc curve parameter, refer to \ref gm_ecc_parameter_t.

}gm_ecc_Diffie_Hellman_t;


typedef void (* gm_ecc_hardware_finish_callback_t)(uint32_t error);

typedef struct _gm_pkc_v1_config
{
    void*  reg_base;
    uint32_t irq_num;
    uint32_t clk_type;
    gm_ecc_hardware_finish_callback_t callback;
} gm_pkc_v1_config_t;

typedef struct _gm_pkc_v1_data
{
    volatile uint8_t interrupt_enable;
    volatile uint8_t interrupt_flag;
    volatile uint8_t pkc_done_flag;
    volatile uint8_t pkc_error_flag;
    volatile uint8_t pkc_overflow_flag;

    //ECC busy flag
    volatile uint8_t pkc_busy;
    volatile uint8_t current_status;

    //data pointer for interrupt handler//
    volatile gm_ecc_point_t* point_multiplication_result;

    volatile uint32_t count_for_rng;
    volatile uint32_t random_history;
    volatile uint32_t gen_count;

} gm_pkc_v1_data_t;


typedef struct device_config {
    char *name;
    //void (*init)(struct device *device);
    const void *config_info;
} device_config_t;


typedef struct device {
    volatile struct device_config *config;
    const void *driver_api;
    void *driver_data;
    uint32_t reference_num;
    uint32_t wakeup_src;  ///<  enable device wakeup if it has CPU wakeup src
    uint8_t wakeup_enable;
} device_t;

typedef struct {
    uint32_t bin_size;
    uint32_t check_sum;
    uint32_t load_addr;
    uint32_t run_addr ;
    uint32_t xqspi_xip_cmd;
    uint32_t xqspi_speed:4;           /*!< bit: 0..3  clock speed */
    uint32_t code_copy_mode:1;        /*!< bit: 4 code copy mode */
    uint32_t system_clk:3;            /*!< bit: 5..7 system clock */
    uint32_t check_image:1;           /*!< bit: 8 check image */
    uint32_t boot_delay:1;            /*!< bit: 9 boot delay time */
    uint32_t reserved:22;             /*!< bit: 22 reserved */
} bl_boot_info_t;


typedef struct _gm_sha_config
{
    /**
     * The pointer to 32 bytes array of user defined initial hash.
     * If user_hash is NULL, standard SHA2-256 initial hash value shall be used (see NIST FIPS 180-4).
     * recommend NULL.
     */
    uint8_t *user_hash;

    uint8_t hmac_key_type_sel;

    uint8_t operation_mode;//0:sha ; 1: hmac

    uint8_t *hmac_key;

    /**
     * Enable or disable SHA DMA mode, 0:disable, 1:enable(recommend 1)
     */
    uint8_t  dma_enable;

    uint8_t no_padding;

    /**
     * Enable or disable SHA interrupt mode, 0:disable, 1:enable(recommend 1)
     */
    uint8_t  interrupt_enable;

    /*
     *   request mode enable flag
     *   =1: will return without wait and check (REG(0x1A011008) & (1 << 4)) == 1
     */
    //uint8_t request_mode_enable;


    /* 0 last block; 1 not last block and will continue input in later stages
     *   under testing; don't use
     */
    uint8_t continue_input;
} gm_sha_config_t;

typedef struct bl_rsa_public_key
{
    uint8_t n[256];
    uint32_t e;
    uint8_t rr[256];
    uint32_t c;
} bl_rsa_public_key_t;

typedef struct bl_rsa_private_key
{
    uint8_t s[256];
} bl_rsa_private_key_t;

typedef struct
{
    uint32_t len;                 /*!<  size(N) in chars  */
    bl_rsa_public_key_t  *pk;
    bl_rsa_private_key_t  *sk;
} bl_rsa_context;

typedef struct {
        uint32_t reserved0;
        uint32_t fw_len;
        uint32_t checksum;
        uint32_t loadaddr;
        uint32_t runaddr;
        uint32_t reserved[7];

        uint32_t pattern;
        uint32_t paddingsize;

        uint8_t ecies[ECIES_PUBLIC_KEY_SIZE];
        uint8_t reserved1[8];
        uint8_t rsa[RSA_PUBLIC_KEY_SIZE];
        uint8_t signature[SIGNATURE_SIZE];
} bl_fw_info_t;


static gm_pkc_v1_config_t pkc_v1_config =
{
    .reg_base = (void*)REGS_PCK_BASE_ADDR,
};

static gm_pkc_v1_data_t pkc_v1_data =
{
    .interrupt_enable = 0,
    .interrupt_flag = 0,
    .pkc_done_flag = 0,
    .pkc_error_flag = 0,
    .pkc_overflow_flag = 0,

    //ECC busy flag
    .pkc_busy = 0,
    .current_status = 0,

    //data pointer for interrupt handler//
    .point_multiplication_result = NULL,
    .count_for_rng = 0,
    .random_history = 0,
    .gen_count = 0,
};

static struct device_config pkc_device_config =
{
    .name = "pkc",
    .config_info = &pkc_v1_config

};

static device_t pkc_dev =
{
    .driver_data = &pkc_v1_data,
    .config = &pkc_device_config
};

extern gm_drv_ret_e gm_drv_sha_encrypt(const gm_sha_config_t *config,
        const uint8_t *in, uint32_t byte_length, uint8_t *out);

extern int bl_rsa_rsassa_pss_verify(bl_rsa_context *ctx, const uint8_t *hash, const uint8_t *sig);


static int bl_check_fw_security(bl_boot_info_t *boot_info, const uint8_t *p_public_key_hash)
{
    int ret = GM_BL_OK;
    gm_sha_config_t sha_config = {0};
    uint8_t hash[SHA256_SIZE] = {0};
    bl_rsa_context rsa = {0};
    uint8_t *fw = (uint8_t *)boot_info->load_addr;
    uint32_t fw_size = boot_info->bin_size;
    bl_fw_info_t *fw_info = (bl_fw_info_t *)(fw + fw_size);

    //max fw size: 8*1024*1024 - 8*1024
    if ((boot_info->load_addr < APP_FLASH_START_ADDR) || ((uint32_t)(fw_info + 1) > APP_FLASH_END_ADDR)
        || (fw_size > (APP_FLASH_END_ADDR - APP_FLASH_START_ADDR)))
    {
        return GM_BL_ERROR_FW;
    }


    do {
        /*calculate rsa public key hash and compare with last 16bytes width efuse fw_public_key_hash*/
        memset(&sha_config, 0x0, sizeof(gm_sha_config_t));
        if ((ret = gm_drv_sha_encrypt(&sha_config, fw_info->rsa, RSA_PUBLIC_KEY_SIZE, hash)) < 0)
        {
            break;
        }

        if (memcmp(hash, p_public_key_hash, SHA256_SIZE>>1) != 0)
        {
            ret = GM_BL_ERROR_SIGNATURE; /*sha error, stop boot!*/
            break;
        }

        /*calculate fw hash except signature data*/
        memset(&sha_config, 0x0, sizeof(gm_sha_config_t));
        if ((ret = gm_drv_sha_encrypt(&sha_config, fw, fw_size + sizeof(bl_fw_info_t) - SIGNATURE_SIZE, hash)) < 0)
        {
            break;
        }

        /*do rsa pkcs1-v2.1 format signature check*/
        rsa.len = RSA_KEY_SIZE; /*sizeof(N) in chars*/
        rsa.pk = (bl_rsa_public_key_t*)fw_info->rsa;
        if((ret = bl_rsa_rsassa_pss_verify(&rsa, hash, fw_info->signature)) < 0)
        {
            ret = GM_BL_ERROR_SIGNATURE;
            break;
        }

    }while(0);

    return ret;
}

bool sign_verify(uint32_t fw_start_addr, uint32_t fw_size, const uint8_t *p_public_key_hash)
{
    bool result = true;
    *(volatile uint32_t *)0xa000e2a0 &= 0xfffffffe;
    *(volatile uint32_t *)0xa000e2a4 &= 0xfffffffe;
    *(volatile uint32_t *)0xa000e2ac &= 0xcfffffff;

    bl_boot_info_t  boot_info =
    {
        .bin_size  = fw_size,
        .load_addr = fw_start_addr,
        .run_addr  = fw_start_addr,
    };

    ll_cgc_disable_force_off_secu_hclk();
    ll_cgc_disable_force_off_secu_div4_pclk();
    
    memcpy((uint32_t *)0x0080008c, &pkc_dev, sizeof(device_t));

    if (bl_check_fw_security(&boot_info, p_public_key_hash) < 0)
    {
        result = false;
    }

    return result;
}


#endif

