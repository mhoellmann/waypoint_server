#!/usr/bin/env python3
from sys import argv
from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QLineEdit
from PyQt5.QtWidgets import QApplication, QPushButton, QDesktopWidget
from PyQt5.QtCore import QSize

class InputWindow(QMainWindow):
    def __init__(self, input_text):
        QMainWindow.__init__(self)

        self.setMinimumSize(QSize(300, 100))
        self.setWindowTitle("Rename Waypoint")

        self.current_text = input_text
        self.nameLabel = QLabel(self)
        self.nameLabel.move(20, 20)

        self.input_line = QLineEdit(self)
        self.input_line.setText(self.current_text)
        self.input_line.selectAll()
        self.input_line.returnPressed.connect(self.okMethod)
        self.input_line.resize(200, 32)
        self.input_line.move(50, 20)

        okButton = QPushButton('OK', self)
        okButton.clicked.connect(self.okMethod)
        okButton.setAutoDefault(True)
        okButton.setDefault(True)
        okButton.resize(80,32)
        okButton.move(160, 60)
        okButton.setAutoDefault(True)

        cancelButton = QPushButton('Cancel', self)
        cancelButton.clicked.connect(self.cancelMethod)
        cancelButton.resize(80,32)
        cancelButton.move(60, 60)

        # center window
        qtRectangle = self.frameGeometry()
        centerPoint = QDesktopWidget().availableGeometry().center()
        print centerPoint
        qtRectangle.moveCenter(centerPoint)
        self.move(qtRectangle.topLeft())

    def okMethod(self):
        self.current_text = self.input_line.text().encode('utf8')
        self.close()

    def cancelMethod(self):
        self.close()

    def getNewName(self):
        return self.current_text

class InputApp:
    def __init__(self, waypoint_name):
        self.old_name = waypoint_name

    def create_app(self):
        if QApplication.instance() is None:
            app_instance = QApplication(argv)
            rename_popup_window = InputWindow(self.old_name)
            rename_popup_window.show()
            app_instance.exec_()
            new_name = rename_popup_window.getNewName()
            app_instance.exit()
            del(app_instance)
            return new_name

