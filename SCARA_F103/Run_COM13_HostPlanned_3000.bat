@echo off
setlocal
cd /d "%~dp0"
echo SCARA_F103 host-planned 3000-point stream test
echo.
echo This test prints every TX/RX/MATCH line by default.
echo Add -QuietLines inside this .bat if you only want progress output.
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "tools\host_planned_stream_stress.ps1" -Port COM13 -Count 3000 -FeedMin 500 -FeedMax 1800 -EnableMotion
echo.
echo Test finished with exit code %ERRORLEVEL%.
pause
