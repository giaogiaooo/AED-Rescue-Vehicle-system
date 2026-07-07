#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sqlite3
import os
import sys
from datetime import datetime

def _get_db_path(db_name="alarms.db"):
    """获取数据库文件路径，兼容 PyInstaller 打包后的环境"""
    # 优先放在 exe 所在目录（PyInstaller onefile 解包后也是可写目录）
    if getattr(sys, 'frozen', False):
        base_dir = os.path.dirname(sys.executable)
    else:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_dir, db_name)

class DatabaseManager:
    """SQLite 数据库管理类，用于存储和分页查询报警记录"""
    
    def __init__(self, db_name="alarms.db"):
        self.db_name = _get_db_path(db_name)
        self.init_db()

    def get_connection(self):
        """获取数据库连接"""
        return sqlite3.connect(self.db_name)

    def init_db(self):
        """初始化数据库表结构"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS alarms (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                time TEXT NOT NULL,
                level TEXT NOT NULL,
                message TEXT NOT NULL
            )
        ''')
        conn.commit()
        conn.close()

    def insert_alarm(self, level, message):
        """插入一条报警记录"""
        conn = self.get_connection()
        cursor = conn.cursor()
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cursor.execute('''
            INSERT INTO alarms (time, level, message)
            VALUES (?, ?, ?)
        ''', (now_str, level, message))
        conn.commit()
        conn.close()
        return now_str

    def get_alarms(self, limit=10, offset=0):
        """分页获取报警记录 (按时间倒序)"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute('''
            SELECT time, level, message 
            FROM alarms 
            ORDER BY id DESC 
            LIMIT ? OFFSET ?
        ''', (limit, offset))
        records = cursor.fetchall()
        conn.close()
        return records

    def get_total_count(self):
        """获取报警记录总数"""
        conn = self.get_connection()
        cursor = conn.cursor()
        cursor.execute('SELECT COUNT(*) FROM alarms')
        count = cursor.fetchone()[0]
        conn.close()
        return count