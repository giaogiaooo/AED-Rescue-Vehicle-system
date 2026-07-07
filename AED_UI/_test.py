from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
import sys

app = QApplication(sys.argv)
w = QWidget()
w.setWindowTitle("测试窗口")
w.resize(300, 150)
layout = QVBoxLayout(w)
layout.addWidget(QLabel("如果看到这个窗口，说明打包成功！"))
w.show()
print("窗口已显示")
sys.exit(app.exec_())
