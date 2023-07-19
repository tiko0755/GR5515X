::=============================================================================
:: Developer should configure BIN_PATH, TOOL_PATH, TARGET and RUN_ADDR
::=============================================================================
::set the path to your Keil installation folder
@echo off
 
 
@set OUTPUT_DIR_PATH=.\build
@set TOOL_PATH=..\..\..\..\..\build\binaries\ble_tools\IAR

@if not exist  %OUTPUT_DIR_PATH% (
md %OUTPUT_DIR_PATH%
) else (
del /q/s %OUTPUT_DIR_PATH%\* >nul
)
 
::Generate system all output files
%TOOL_PATH%\ble_tools.exe  --mode=gen --cfg=..\Src\config\custom_config.h --bin=.\Debug\Exe\load_app.bin --outdir=.\build\ --app_name=load_app 
del /q/s .\build\*.tmp >nul
del /q/s .\build\info.* >nul
del /q/s .\build\header.* >nul


echo Firmware has been successfully generated!
