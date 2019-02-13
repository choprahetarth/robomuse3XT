from PyQt4 import QtGui,QtCore
import os
import sys

class Window(QtGui.QMainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.setGeometry(0, 0, 1366, 768)
        self.setWindowTitle("Surveillance Mode")
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
        self.setPalette(palette)
        self.home()
        
    def home(self):
        label = QtGui.QLabel(self)
        label.setText("")
        label.resize(1366,768)
        label.move(0,0)
        label.setPixmap(QtGui.QPixmap("Teamthapar.jpg"))
        label.setScaledContents(True)

        Button1 = QtGui.QPushButton("Start Surveillance",self)
        Button1.clicked.connect(self.straight)
        Button1.resize(200,100)
        Button1.move(50,620)   

        Button2 = QtGui.QPushButton("Start the Video Recording", self)
        Button2.clicked.connect(self.click)
        Button2.resize(200,100)
        Button2.move(1050,620)

        Button3 = QtGui.QPushButton("Start the Server",self)
        Button3.clicked.connect(self.startServer)
        Button3.resize(200,100)
        Button3.move(300,620)

        Button4 = QtGui.QPushButton("Stop the Server",self)
        Button4.clicked.connect(self.closeServer)
        Button4.resize(200,100)
        Button4.move(800,620)

    def click(self):
        os.system('python camera.py')

    def straight(self):
        print ("Hello")

    def startServer(self):
        os.system('sudo motion server start')

    def closeServer(self):
        os.system('sudo motion server stop')

    def close_application(self):
        print("whooaaaa so custom!!!")
        sys.exit()

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    GUI=Window()
    GUI.show()
    sys.exit(app.exec_())


        