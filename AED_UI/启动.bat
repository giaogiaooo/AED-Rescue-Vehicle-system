@echo off
chcp 65001 >nul
title 智能AED救援车监控平台 v2.0
echo.
echo ========================================
echo    智能AED救援车监控平台 v2.0
echo ========================================
echo.
echo [1/3] 检查 Python 环境...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未找到 Python，请先安装 Python 3.8+
    echo 下载地址: https://www.python.org/downloads/
    pause
    exit /b 1
)
echo         Python 已就绪 ✓

echo [2/3] 检查依赖库...
pip install PyQt5 opencv-python roslibpy numpy -q
echo         依赖库已就绪 ✓

echo [3/3] 启动 UI 界面...
echo.
python main.py
pause
