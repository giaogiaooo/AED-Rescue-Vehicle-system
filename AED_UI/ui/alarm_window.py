#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QTableWidget, 
                             QTableWidgetItem, QPushButton, QLabel, QHeaderView)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont
from database.db import DatabaseManager

class AlarmHistoryWindow(QDialog):
    """历史报警记录分页查询窗口"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.db = DatabaseManager()
        
        self.limit = 15  # 每页显示 15 条
        self.offset = 0
        self.total_count = 0
        
        self.init_ui()
        self.apply_styles()
        self.load_data()

    def init_ui(self):
        self.setWindowTitle("历史报警记录")
        self.resize(750, 520)
        self.setWindowFlags(self.windowFlags() | Qt.WindowMinMaxButtonsHint)
        
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(12, 12, 12, 12)
        
        # 标题
        title = QLabel("📋 历史报警记录")
        title.setObjectName("DialogTitle")
        layout.addWidget(title)
        
        # 表格设置
        self.table = QTableWidget(0, 3)
        self.table.setObjectName("AlarmTable")
        self.table.setHorizontalHeaderLabels(["报警时间", "报警等级", "报警内容"])
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.setShowGrid(False)
        layout.addWidget(self.table)
        
        # 底部翻页控制器
        ctrl_layout = QHBoxLayout()
        self.btn_prev = QPushButton("◀ 上一页")
        self.btn_prev.setObjectName("PageBtn")
        self.btn_next = QPushButton("下一页 ▶")
        self.btn_next.setObjectName("PageBtn")
        self.lbl_page = QLabel("第 1 页")
        self.lbl_page.setObjectName("PageLabel")
        self.lbl_page.setAlignment(Qt.AlignCenter)
        
        self.btn_prev.clicked.connect(self.page_prev)
        self.btn_next.clicked.connect(self.page_next)
        
        ctrl_layout.addStretch()
        ctrl_layout.addWidget(self.btn_prev)
        ctrl_layout.addSpacing(10)
        ctrl_layout.addWidget(self.lbl_page)
        ctrl_layout.addSpacing(10)
        ctrl_layout.addWidget(self.btn_next)
        ctrl_layout.addStretch()
        
        layout.addLayout(ctrl_layout)

    def apply_styles(self):
        style = """
        QDialog {
            background-color: #0a0e1a;
        }
        #DialogTitle {
            font-size: 18px;
            font-weight: bold;
            color: #00e5ff;
            padding: 6px;
        }
        #AlarmTable {
            background-color: #0d1426;
            color: #c8d6f0;
            border: 1px solid #1e3a5f;
            border-radius: 8px;
            gridline-color: #162040;
            font-size: 12px;
            font-family: "Consolas", "Courier New", monospace;
        }
        #AlarmTable QHeaderView::section {
            background-color: #111d38;
            color: #00e5ff;
            padding: 6px;
            border: none;
            border-bottom: 2px solid #1e3a5f;
            font-weight: bold;
            font-size: 12px;
        }
        #AlarmTable::item {
            padding: 5px 8px;
            border-bottom: 1px solid #111d38;
        }
        #AlarmTable::item:selected {
            background-color: #112244;
            color: #00e5ff;
        }
        #PageBtn {
            background-color: #162040;
            color: #00e5ff;
            border: 1px solid #1e3a5f;
            border-radius: 6px;
            padding: 6px 16px;
            font-weight: bold;
            font-size: 12px;
        }
        #PageBtn:hover {
            background-color: #1e3058;
            border: 1px solid #00ccff;
        }
        #PageBtn:disabled {
            color: #445566;
            border: 1px solid #162040;
            background-color: #0d1328;
        }
        #PageLabel {
            color: #7799bb;
            font-size: 12px;
            font-weight: bold;
        }
        """
        self.setStyleSheet(style)

    def load_data(self):
        """从 SQLite 加载数据到表格"""
        self.total_count = self.db.get_total_count()
        records = self.db.get_alarms(limit=self.limit, offset=self.offset)
        
        self.table.setRowCount(0)
        for row, record in enumerate(records):
            self.table.insertRow(row)
            
            time_item = QTableWidgetItem(record[0])
            time_item.setForeground(QColor("#8899bb"))
            self.table.setItem(row, 0, time_item)
            
            level_item = QTableWidgetItem(record[1])
            level = record[1]
            if "红" in level:
                level_item.setForeground(QColor("#ff4444"))
            elif "橙" in level:
                level_item.setForeground(QColor("#ff8800"))
            elif "黄" in level:
                level_item.setForeground(QColor("#ffcc00"))
            else:
                level_item.setForeground(QColor("#8899bb"))
            self.table.setItem(row, 1, level_item)
            
            msg_item = QTableWidgetItem(record[2])
            msg_item.setForeground(QColor("#c8d6f0"))
            self.table.setItem(row, 2, msg_item)
        
        # 更新页码显示
        current_page = (self.offset // self.limit) + 1
        total_pages = max(1, (self.total_count + self.limit - 1) // self.limit)
        self.lbl_page.setText(f"第 {current_page} / {total_pages} 页 (共 {self.total_count} 条)")
        
        self.btn_prev.setEnabled(self.offset > 0)
        self.btn_next.setEnabled(self.offset + self.limit < self.total_count)

    def page_prev(self):
        if self.offset >= self.limit:
            self.offset -= self.limit
            self.load_data()

    def page_next(self):
        if self.offset + self.limit < self.total_count:
            self.offset += self.limit
            self.load_data()