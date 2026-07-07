#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
项目名称：智能AED救援车监控平台 (Windows WebSocket版)
前置环境：pip install PyQt5 opencv-python roslibpy numpy
"""

import sys
import os
import traceback

os.environ["AUTOBAHN_USE_NVX"] = "0"

def exception_hook(exctype, value, tb):
    """全局异常拦截器：拦截所有导致程序崩溃的错误，并在控制台红色打印"""
    print("\n" + "="*50)
    print("【系统崩溃】捕获到未处理的严重异常：")
    print("".join(traceback.format_exception(exctype, value, tb)))
    print("="*50 + "\n")
    sys.exit(1)

# 将系统默认的异常处理替换为自定义钩子
sys.excepthook = exception_hook

print("[系统启动] 1. 正在导入 PyQt5...")
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

print("[系统启动] 2. 正在导入主窗口与子模块...")
try:
    from ui.main_window import MainWindow
except Exception as e:
    print("\n[错误] 导入 UI 模块失败！请务必检查每个子文件夹内是否存在 __init__.py 空文件：")
    traceback.print_exc()
    sys.exit(1)

def main():
    print("[系统启动] 3. 初始化 QApplication...")
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    
    app = QApplication(sys.argv)
    
    print("[系统启动] 4. 正在创建 MainWindow 实例...")
    window = MainWindow()
    
    print("[系统启动] 5. 正在显示窗口...")
    window.show()
    
    print("[系统启动] 6. 成功进入 Qt 事件循环！(程序运行中...)")
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()