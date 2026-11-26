@echo off
rem Latency of Horizon Receiver with Intel PresentMon (Windows' own record of when each frame reached the screen,
rem and whether it was composed or flipped straight to it). Needs administrator rights (PresentMon traces other
rem programs): right-click this file, "Run as administrator". The Ground must be plugged in and receiving.
rem Do not touch the keyboard or mouse while it runs (about 70 s): a key or a click changes the layout.
rem
rem   1. the bar drawn into the video         frames_bar.csv + pm_bar.csv
rem   2. the video on the whole screen         frames_full.csv + pm_full.csv
rem Results in %USERPROFILE%\horizon-build\latency\: python test\analyze_latency.py pairs each picture with its display.
setlocal
net session >nul 2>&1 || (echo Run this as administrator: right-click it, "Run as administrator". & pause & exit /b 1)
set PM=%USERPROFILE%\horizon-build\tools\presentmon\PresentMon-2.6.0-x64.exe
set APP=%~dp0..\target\release\horizon_receiver.exe
set OUT=%USERPROFILE%\horizon-build\latency
if not exist "%PM%" (echo PresentMon not found: %PM% & pause & exit /b 1)
if not exist "%APP%" (echo Build the app first: cargo build --release & pause & exit /b 1)
if not exist "%OUT%" mkdir "%OUT%"
del /q "%OUT%\*.csv" 2>nul

taskkill /im horizon_receiver.exe /f >nul 2>&1
echo [1/2] the bar drawn into the video, the video below it (30 s)
start "" "%APP%" --frame-log "%OUT%\frames_bar.csv"
timeout /t 8 /nobreak >nul
"%PM%" --process_name horizon_receiver.exe --output_file "%OUT%\pm_bar.csv" --timed 20 --terminate_after_timed --no_console_stats --v1_metrics --qpc_time
taskkill /im horizon_receiver.exe /f >nul 2>&1
timeout /t 3 /nobreak >nul

echo [2/2] the video on the whole screen (30 s)
start "" "%APP%" --hide-overlay --frame-log "%OUT%\frames_full.csv"
timeout /t 8 /nobreak >nul
"%PM%" --process_name horizon_receiver.exe --output_file "%OUT%\pm_full.csv" --timed 20 --terminate_after_timed --no_console_stats --v1_metrics --qpc_time
taskkill /im horizon_receiver.exe /f >nul 2>&1

echo.
echo Done: %OUT%
dir /b "%OUT%"
pause
