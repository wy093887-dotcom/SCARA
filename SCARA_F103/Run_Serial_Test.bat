@echo off
setlocal EnableExtensions
cd /d "%~dp0"

rem 课程设计串口测试启动器：只保留当前 G-code 通信路径和高频轨迹流测试。

echo SCARA_F103 course-design serial test launcher
echo Default serial setting: 115200 8N1
echo.
echo Available serial ports:
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\serial_link_check.ps1" -ListPorts
echo.

set "PORT="
set /p PORT=Enter serial port, for example COM13: 
if "%PORT%"=="" (
  echo No port entered.
  pause
  exit /b 2
)

:menu
echo.
echo Selected port: %PORT%
echo.
echo 1. Basic serial link check, safe
echo 2. G-code protocol check, safe
echo 3. Host-planned 3000-point stream, verbose TX/RX
echo 4. Host-planned 3000-point stream, quiet progress
echo 5. Read HOME microswitch inputs, safe
echo 6. List ports again
echo 0. Exit
echo.
set "CHOICE="
set /p CHOICE=Choose test: 
if "%CHOICE%"=="" goto done

if "%CHOICE%"=="1" goto link_check
if "%CHOICE%"=="2" goto gcode_check
if "%CHOICE%"=="3" goto host_verbose
if "%CHOICE%"=="4" goto host_quiet
if "%CHOICE%"=="5" goto home_sensor
if "%CHOICE%"=="6" goto list_ports
if "%CHOICE%"=="0" goto done
echo Unknown choice.
goto menu

:link_check
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\serial_link_check.ps1" -Port "%PORT%"
goto after_test

:gcode_check
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\gcode_stream_check.ps1" -Port "%PORT%"
goto after_test

:host_verbose
echo.
echo This sends 3000 planned G-code points and prints every TX/RX/MATCH line.
echo If only the control board is connected, this is fine for communication/queue testing.
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\host_planned_stream_stress.ps1" -Port "%PORT%" -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion
goto after_test

:host_quiet
echo.
echo This sends 3000 planned G-code points and shows progress/errors only.
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\host_planned_stream_stress.ps1" -Port "%PORT%" -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion -QuietLines
goto after_test

:home_sensor
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\home_sensor_check.ps1" -Port "%PORT%"
goto after_test

:list_ports
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\serial_link_check.ps1" -ListPorts
goto after_test

:after_test
echo.
echo Test finished with exit code %ERRORLEVEL%.
pause
goto menu

:done
endlocal
exit /b 0
