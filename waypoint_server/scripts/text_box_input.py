#!/usr/bin/python2

from PyQt5.QtWidgets import QMainWindow, QWidget, QLabel, QLineEdit
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QSize    

class MainWindow(QMainWindow):
    def __init__(self, waypoint_name):
        QMainWindow.__init__(self)


        self.setMinimumSize(QSize(300, 100))    
        self.setWindowTitle("Rename Waypoint") 

        self.current_name = waypoint_name
        self.nameLabel = QLabel(self)
        self.line = QLineEdit(self)
        self.line.setText(self.current_name)

        self.line.move(50, 20)
        self.line.resize(200, 32)
        self.nameLabel.move(20, 20)

        renameButton = QPushButton('OK', self)
        cancelButton = QPushButton('Cancel', self)
        renameButton.clicked.connect(self.renameMethod)
        cancelButton.clicked.connect(self.cancelMethod)
        renameButton.resize(80,32)
        renameButton.move(160, 60)
        cancelButton.resize(80,32) 
        cancelButton.move(60, 60)       

    def renameMethod(self):
        self.current_name = self.line.text()
        self.close()

    def cancelMethod(self):
        self.current_name = "potato"
        self.close()

    def getNewName(self):
        return self.current_name


        