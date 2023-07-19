#include "user_efuse.h"
#include "string.h"
#include "gr55xx.h"
#include "gr55xx_hal.h"

uint32_t user_efuse_write(uint8_t word_offset, uint32_t * efuse_value, uint8_t size_word)
{
    uint32_t ret = USER_EFUSE_ERROR_NONE;
    efuse_handle_t efuse_handle = {0};

    /*
     * Reserved For Furture : Because the macro USER_EFUSE_BASE_OFFSET equal zero now, it will cause compiler warning.
     * But in the furture, maybe USER_EFUSE_BASE_OFFSET will be another value.
     */
    if( /*(word_offset < USER_EFUSE_BASE_OFFSET) || */(word_offset > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE)) || (efuse_value == NULL))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    if((word_offset + size_word * 4) > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    do {
        efuse_handle.p_instance = EFUSE;
        efuse_handle.init.info_mode = DISABLE;
        if(hal_efuse_init(&efuse_handle) != HAL_OK)
        {
            ret = USER_EFUSE_INIT_ERROR;
            break;
        }

        if(HAL_OK != hal_efuse_write(&efuse_handle, word_offset, efuse_value, size_word))
        {
            ret =  USER_EFUSE_WRITE_ERROR;
            break;
        }
    } while(0);
    hal_efuse_deinit(&efuse_handle);
    return ret;
}

uint32_t user_efuse_read(uint8_t word_offset, uint32_t *data, uint8_t size_word)
{
    uint32_t ret = USER_EFUSE_ERROR_NONE;
    efuse_handle_t efuse_handle = {0};

    /*
     * Reserved For Furture : Because the macro USER_EFUSE_BASE_OFFSET equal zero now, it will cause compiler warning.
     * But in the furture, maybe USER_EFUSE_BASE_OFFSET will be another value.
     */
    if( /*(word_offset < USER_EFUSE_BASE_OFFSET) || */(word_offset > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE)) || (data == NULL))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    if((word_offset + size_word * 4) > (USER_EFUSE_BASE_OFFSET + USER_EFUSE_SIZE))
    {
        return USER_EFUSE_ERROR_INVALID_PARAM;
    }

    do {
        efuse_handle.p_instance = EFUSE;
        efuse_handle.init.info_mode = DISABLE;
        if(hal_efuse_init(&efuse_handle) != HAL_OK)
        {
            ret = USER_EFUSE_INIT_ERROR;
            break;
        }

        if(HAL_OK != hal_efuse_read(&efuse_handle, word_offset, data, size_word))
        {
            return USER_EFUSE_READ_ERROR;
        }
    }while(0);
    hal_efuse_deinit(&efuse_handle);
    return ret;
}

