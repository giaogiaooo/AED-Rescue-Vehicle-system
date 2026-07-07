@echo off
chcp 65001 >nul
title Pack AED Monitor Platform
cd /d "%~dp0"

for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') do set "BUILD_TIME=%%i"
set "APP_NAME=AED_%BUILD_TIME%"
set "OUT_EXE=C:\Users\ASUS\Desktop\%APP_NAME%.exe"

echo.
echo ========================================
echo   Pack as standalone EXE (shareable)
echo ========================================
echo.
echo   Output: %OUT_EXE%
echo.

echo [1/4] Clean old build cache...
if exist "dist" rmdir /s /q "dist" 2>nul
if exist "build" rmdir /s /q "build" 2>nul
if exist "*.spec" del /q "*.spec" 2>nul
echo         [OK] Done
echo.

echo [2/4] Install/Check pack dependencies...
python -m pip install pyinstaller -q
if %errorlevel% neq 0 (
    echo.
    echo ========================================
    echo   [FAIL] Dependency install/check failed.
    echo ========================================
    pause
    exit /b 1
)
echo         [OK] Done
echo.

echo [3/4] Building...
python -m PyInstaller ^
  --onefile ^
  --windowed ^
  --name="%APP_NAME%" ^
  --icon="%~dp0logo.ico" ^
  --distpath "C:\Users\ASUS\Desktop" ^
  --add-data "database;database" ^
  --add-data "ui;ui" ^
  --add-data "widgets;widgets" ^
  --add-data "ros;ros" ^
  --add-binary "C:\Users\ASUS\AppData\Roaming\Python\Python314\site-packages\PyQt5\Qt5\plugins\platforms\qwindows.dll;platforms" ^
  --add-binary "C:\Users\ASUS\AppData\Roaming\Python\Python314\site-packages\PyQt5\Qt5\plugins\styles\qwindowsvistastyle.dll;styles" ^
  --exclude-module _nvx_utf8validator ^
  --exclude-module _nvx_xormasker ^
  --hidden-import=roslibpy ^
  --hidden-import=cv2 ^
  --hidden-import=numpy ^
  --hidden-import=PyQt5.QtCore ^
  --hidden-import=PyQt5.QtGui ^
  --hidden-import=PyQt5.QtWidgets ^
  main.py

if %errorlevel% neq 0 (
    echo.
    echo ========================================
    echo   [FAIL] Build failed. Check errors above.
    echo ========================================
    pause
    exit /b 1
)

echo.
echo [4/4] Check output...
if exist "%OUT_EXE%" (
    echo.
    echo ========================================
    echo   [OK] Build success!
    echo   EXE: %OUT_EXE%
    echo ========================================
    echo.
    echo   Refreshing Windows icon cache...
    ie4uinit.exe -show
    del /f /s /q /a "%localappdata%\IconCache.db" 2>nul
    echo   [OK] Done
) else (
    echo   [FAIL] EXE not found: %OUT_EXE%
)
echo.
pause
