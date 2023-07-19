#ifndef _USER_EFUSE_H_
#define _USER_EFUSE_H_

#include <stdint.h>
#include <stdbool.h>

#define USER_EFUSE_BASE_OFFSET           EFUSE_OFFSET_USER_DSVD
#define USER_EFUSE_SIZE                 (0x0020UL)

#define USER_EFUSE_ERROR_NONE                         ((uint32_t)0x00000000)              /**< No error                 */
#define USER_EFUSE_INIT_ERROR                         ((uint32_t)0x00000001)              /**< INIT error               */
#define USER_EFUSE_WRITE_ERROR                        ((uint32_t)0x00000002)              /**< WRITE error              */
#define USER_EFUSE_READ_ERROR                         ((uint32_t)0x00000004)              /**< READ error               */
#define USER_EFUSE_ERROR_INVALID_PARAM                ((uint32_t)0x00000008)              /**< Invalid parameters error */

uint32_t user_efuse_write(uint8_t offset, uint32_t * efuse_value, uint8_t size_word);
uint32_t user_efuse_read(uint8_t word_offset, uint32_t *data, uint8_t size_word);
#endif

