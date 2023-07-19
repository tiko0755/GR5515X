@ECHO OFF

SET WORK_DIR=%~dp0

SET BOOT_FILE_NAME=oppo_second_boot.bin

SET ERASE_FILE_SADDR=0x00000000
SET BLE_FILE_SADDR=0x00000000
SET FILM_FILE_SADDR=0x00028000
SET MCU_FILE_SADDR=0x00030000
SET FW_END_ADDR=0x0100000

::加签需要验证固件名区分大小写
SET MODEL_NAME=oppo_pencil
SET MODEL_VERSION=0.8

::把jflash添加到路径里
set PATH_JFLASH=C:\Program Files\SEGGER\JLink
::把Gprogrammer添加到路径里
set PATH_GPROGRAMMER=C:\Goodix\GProgrammer
set PATH=%PATH_JFLASH%;%PATH_GPROGRAMMER%;%PATH%;

:: 操作系统语言是否为英语
SET IS_OS_LANG_EN=0

:: 中文系统下，date命令获取到的时间信息格式为：2018/12/25，而英语系统则不同，获取到的时间信息格式为：12/25/2018
IF %IS_OS_LANG_EN% EQU 0 ( 
SET BUILD_TIME=%date:~2,2%%date:~5,2%%date:~8,2%
) ELSE (
SET BUILD_TIME=%date:~8,2%%date:~0,2%%date:~3,2%
)

SET FLASH_FILE_NAME=%MODEL_NAME%_3in1_%MODEL_VERSION%_Build%BUILD_TIME%.bin
ECHO generate merge image : %FLASH_FILE_NAME%

::合并固件
JFlash.exe -open%ERASE_FILE_DIR%%ERASE_FILE_NAME%,%ERASE_FILE_SADDR% -merge%BLE_FILE_DIR%%BLE_FILE_NAME%,%BLE_FILE_SADDR% -merge%FILM_FILE_DIR%%FILM_FILE_NAME%,%FILM_FILE_SADDR% -merge%MCU_FILE_DIR%%MCU_FILE_NAME%,%MCU_FILE_SADDR% -saveas%FW_FILE_DIR%%FLASH_FILE_NAME% -exit

SET GOODIX_FILE_NAME=%MODEL_NAME%_3in1_%MODEL_VERSION%_Build%BUILD_TIME%_goodix.bin
ECHO generate goodix image : %GOODIX_FILE_NAME%

::添加汇顶固件二进制尾缀
GR5xxx_console.exe generate "%FW_FILE_DIR%%FLASH_FILE_NAME%" "%FW_FILE_DIR%%GOODIX_FILE_NAME%" "0x01000000" 1024 0

SET SIGN_FILE_NAME=%MODEL_NAME%_3in1_%MODEL_VERSION%_Build%BUILD_TIME%_sign.bin
ECHO generate goodix image : %SIGN_FILE_NAME%

::固件签名
GR5xxx_encrypt_signature.exe --operation="sign" --firmware_key="%KEY_DIR%firmware.key" --signature_key="%KEY_DIR%sign.key" --signature_pub_key="%KEY_DIR%sign_pub.key" --product_json_path="%KEY_DIR%product.json" --ori_firmware="%FW_FILE_DIR%%GOODIX_FILE_NAME%" --output="%SIGN_FILE_DIR%%SIGN_FILE_NAME%" --random_output="%SIGN_FILE_DIR%random.bin" --base_addr="0x01000000" --flash_size="1024" --product_type="0"

