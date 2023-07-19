/**
 ****************************************************************************************
 *
 * @file scatter_common.h
 *
 * @brief decalare the symbols in scatter_common.sct.
 *
 *
 ****************************************************************************************
 */

#ifndef __SCATTER_COMMON_H__
#define __SCATTER_COMMON_H__
#include "flash_scatter_config.h"
#include <stdint.h>
#include <custom_config.h>

#if (CFG_MAX_CONNECTIONS < 3) && (CFG_MAX_ADVS < 2) && (CFG_MESH_SUPPORT < 1)
#define EM_BASE_ADDR               (0xB0008000)
#define EM_BLE_ADVDATATXBUF_OFFSET (0x13A0)
#define EM_BLE_ADVDATATXBUF_END    (0x4F28)
#define EM_NVDS_OFFSET             (0x6690)

// (EM_BASE_ADDR + EM_BLE_ADVDATATXBUF_OFFSET + 4 * 1270 + 16(reserved))
#define ENV_HEAP_ADDR              (0xB000A788)

// (ENV_HEAP_ADDR + ENV_HEAP_SIZE:2900 bytes)
#define KE_MSG_HEAP_ADDR           (0xB000B2DC)

// (KE_MSG_HEAP_ADDR + MSG_HEAP_SIZE:6700 bytes)
#define KE_MSG_HEAP_END_ADDR       (0xB000CD08)

// (EM_BASE_ADDR + EM_NVDS_OFFSET + 4096 + 16(reserved))
#define ATT_DB_HEAP_ADDR           (0xB000F6A0)

// (ATT_DB_HEAP_ADDR + ATT_DB_HEAP_SIZE:1024)
#define NON_RET_HEAP_ADDR          (0xB000FAA0)

// (NON_RET_HEAP_ADDR + ATT_DB_HEAP_SIZE:1024)
#define NON_RET_HEAP_END_ADDR      (0xB000FEA0)

#define STACK_HEAP_INIT(heaps_table)    uint8_t prf_buf[PRF_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t bond_buf[BOND_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t conn_buf[CONN_BUF_SIZE] __attribute__((aligned (32))) = {0};\
stack_heaps_table_t heaps_table = {     (uint32_t *)(ENV_HEAP_ADDR),\
                                        (uint32_t *)(ATT_DB_HEAP_ADDR),\
                                        (uint32_t *)(KE_MSG_HEAP_ADDR),\
                                        (uint32_t *)(NON_RET_HEAP_ADDR),\
                                        2900,\
                                        1024,\
                                        6700,\
                                        1024,\
                                        (uint8_t *)prf_buf,\
                                        PRF_BUF_SIZE,\
                                        (uint8_t *)bond_buf,\
                                        BOND_BUF_SIZE,\
                                        (uint8_t *)conn_buf,\
                                        CONN_BUF_SIZE}
#else
#define STACK_HEAP_INIT(heaps_table)    uint8_t prf_buf[PRF_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t bond_buf[BOND_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t conn_buf[CONN_BUF_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t env_heap_buf[ENV_HEAP_SIZE] __attribute__((aligned (32)))= {0};\
                                        uint8_t att_db_heap_buf[ATT_DB_HEAP_SIZE] __attribute__((aligned (32)))= {0};\
                                        uint8_t ke_msg_heap_buf[KE_MSG_HEAP_SIZE] __attribute__((aligned (32))) = {0};\
                                        uint8_t non_ret_heap_buf[NON_RET_HEAP_SIZE]__attribute__((aligned (32))) = {0};\
stack_heaps_table_t heaps_table = { (uint32_t *)env_heap_buf,\
                                        (uint32_t *)att_db_heap_buf,\
                                        (uint32_t *)ke_msg_heap_buf,\
                                        (uint32_t *)non_ret_heap_buf,\
                                        ENV_HEAP_SIZE,\
                                        ATT_DB_HEAP_SIZE,\
                                        KE_MSG_HEAP_SIZE,\
                                        NON_RET_HEAP_SIZE,\
                                        (uint8_t *)prf_buf,\
                                        PRF_BUF_SIZE,\
                                        (uint8_t *)bond_buf,\
                                        BOND_BUF_SIZE,\
                                        (uint8_t *)conn_buf,\
                                        CONN_BUF_SIZE}
#endif


#endif // __SCATTER_COMMON_H__
