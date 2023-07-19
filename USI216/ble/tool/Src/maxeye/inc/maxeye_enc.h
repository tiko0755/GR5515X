#ifndef __MAXEYE_ENC_H__
#define __MAXEYE_ENC_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"

/**@brief  define*/

#define MAXEYE_CLI_HEAD   0x4D

#define PENCIL_ENC_OK     0
#define PENCIL_WAIT_ENC   1

#define SN_SIZE           17
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

extern uint8_t devSn[SN_SIZE];

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
uint8_t get_xorcheck(uint8_t *buf, uint8_t len);

void maxeye_read_periheral_chipid(void);
void maxeye_read_periheral_hash(void);
void maxeye_read_periheral_sign_hash(void);
void maxeye_read_periheral_key(void);
void maxeye_read_periheral_sn(void);
void maxeye_read_periheral_mac(void);
void maxeye_write_periheral_reset(void);

void maxeye_write_write_enc_key(void);
void maxeye_write_periheral_hash(void);
uint8_t maxeye_enc_verify(void);

void maxeye_cli_cb(uint8_t *pData, uint8_t size);
#endif

