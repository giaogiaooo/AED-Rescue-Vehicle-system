@echo off
chcp 65001 >nul
title AED Kivy Desktop Test
cd /d "%~dp0"

echo.
echo ========================================
echo   AED Kivy Desktop Test
echo ========================================
echo.

echo [1/2] Install deps...
python -m pip install kivy roslibpy numpy opencv-python -q
echo         [OK]

echo [2/2] Launching...
echo.
python main.py
pause
