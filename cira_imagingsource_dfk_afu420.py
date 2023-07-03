import sys
sys.path.append('C:\opt\\ros\melodic\\x64\lib\site-packages') # fmt:off
sys.path.append('C:\\CiRA-CORE\\install\\lib\\site-packages')

import PyQt5
from PyQt5.QtWidgets import QWidget, QMainWindow, QLabel, QSizePolicy, QApplication, QAction, QHBoxLayout, QProgressBar
# from PyQt5.QtCore import Qt,QEvent,QObject
from PyQt5.QtCore import *
from PyQt5.QtGui import QImage, QPixmap

import traceback
import os

import ctypes as C
import numpy as np

import rospy

import cv2
import qimage2ndarray
import ros_numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cira_msgs.srv import CiraFlowService2, CiraFlowService2Request, CiraFlowService2Response
import numpy as np
# sys.path.remove('C:\opt\\ros\melodic\\x64\lib\site-packages')

# Import PyhtonNet
import clr

dir_path = os.path.dirname(os.path.realpath(__file__))

print('add .dll path : ', dir_path + '\\redist\\dotnet\\x64')
# Add path to IC Imaging Control 3.5 .NET 64bit installation
sys.path.append(dir_path + '\\redist\\dotnet\\x64')
#sys.path.append('C:\\Users\\cira\\Documents\\IC Imaging Control 3.5\\redist\\dotnet\\x64')

# Load IC Imaging Control .NET 

clr.AddReference('TIS.Imaging.ICImagingControl35')
clr.AddReference('System')

# Import the IC Imaging Control namespace.
import TIS.Imaging
from System import TimeSpan


class QNode(QThread):
    def __init__(self, parent=None):
        super().__init__(parent)
        rospy.init_node("ic_node", anonymous=True)
        self.start()

    def run(self) -> None:
        rospy.spin()

class RosInterface(QThread):
    def __init__(self, parent=None, serialNumber="0x"):
        super().__init__(parent)
        self.img: QImage = QImage()
        self.mtx = QMutex()
        self.isNewImage = False
        self.pub = rospy.Publisher(f'/{serialNumber}/cira_image_service/image', String, queue_size=10)
        self.srv = rospy.Service(f'/{serialNumber}/cira_image_service/image', CiraFlowService2, self.processGetImage)

    def processGetImage(self, req):
        if req.flow_in.jsonstr != "getImage":
            res = CiraFlowService2Response()
            res.flow_out.jsonstr = "Wrong command"
            return res

        self.mtx.lock()
        self.isNewImage = False
        self.mtx.unlock()

        while not rospy.is_shutdown() and not self.isNewImage:
            self.msleep(10)

        self.mtx.lock()
        res = CiraFlowService2Response()
        res.flow_out.jsonstr = ""

        img = qimage2ndarray.rgb_view(self.img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        if len(img) > 0:
            res.flow_out.img = ros_numpy.msgify(Image, img, encoding='bgr8')

        self.isNewImage = True
        self.mtx.unlock()
        return res


class SinkData:
    brightnes = 0
    FrameBuffer = None


class DisplayBuffer:
    '''
    This class is needed to copy the image into a pixmap for
    displaying in the video window.
    '''
    locked = False
    pixmap = None

    def Copy( self, FrameBuffer):
        if(  int(FrameBuffer.FrameType.BitsPerPixel/8 ) == 4):
            imgcontent = C.cast(FrameBuffer.GetIntPtr().ToInt64(), C.POINTER(C.c_ubyte * FrameBuffer.FrameType.BufferSize))
            qimage = QImage(imgcontent.contents, FrameBuffer.FrameType.Width,FrameBuffer.FrameType.Height, QImage.Format_RGB32).mirrored()

            rosInterface.mtx.lock()
            if not rosInterface.isNewImage:
                rosInterface.img = qimage.copy()
                rosInterface.isNewImage = True
            rosInterface.mtx.unlock()

            self.pixmap = QPixmap(qimage)

class WorkerSignals(QObject):
    display = pyqtSignal(object)


class DisplayFilter(TIS.Imaging.FrameFilterImpl):
    '''
    This frame filter copies an incoming frame into our 
    DisplayBuffer object and signals the QApplication
    with the new buffer.
    '''
    __namespace__ = "DisplayFilterClass"
    signals = WorkerSignals()
    dispBuffer = DisplayBuffer()

    def GetSupportedInputTypes(self, frameTypes):
        frameTypes.Add( TIS.Imaging.FrameType(TIS.Imaging.MediaSubtypes.RGB32))

    def GetTransformOutputTypes(self,inType, outTypes):
        outTypes.Add(inType)
        return True

    def Transform(self, src, dest):
        dest.CopyFrom(src)
        if self.dispBuffer.locked is False:
            self.dispBuffer.locked = True
            self.dispBuffer.Copy(dest)
            self.signals.display.emit(self.dispBuffer)

        return False

####################################################################################

def SelectDevice():
    ic.LiveStop()
    ic.ShowDeviceSettingsDialog()
    if ic.DeviceValid is True:
        ic.LiveStart()
        ic.SaveDeviceStateToFile("device.xml")

def ShowProperties():
    if ic.DeviceValid is True:
        ic.ShowPropertyDialog()
        ic.SaveDeviceStateToFile("device.xml")

def SnapImage():
    '''
    Snap and save an image
    '''
    image = snapsink.SnapSingle(TimeSpan.FromSeconds(1))
    TIS.Imaging.FrameExtensions.SaveAsBitmap(image,"test.bmp")

def Close():
    if ic.DeviceValid is True:
        ic.LiveStop()
    app.quit()

def imageCallback(x,y,buffer):
    print("hallo")
    return 0

def OnDisplay(dispBuffer: DisplayBuffer):
    videowindow.setPixmap(dispBuffer.pixmap)   
    dispBuffer.locked = False   


qnode = QNode()
print("start")

app =  QApplication(sys.argv)

w = QMainWindow()
w.resize(640, 480)
w.move(300, 300)
w.setWindowTitle('Simple Camera')

# Create the menu
mainMenu = w.menuBar()
fileMenu = mainMenu.addMenu('&File')

exitAct =  QAction("&Exit",app)
exitAct.setStatusTip("Exit program")
exitAct.triggered.connect(Close)
fileMenu.addAction(exitAct)

deviceMenu = mainMenu.addMenu('&Device')
devselAct =  QAction("&Select",app)
devselAct.triggered.connect(SelectDevice)
deviceMenu.addAction(devselAct)

devpropAct =  QAction("&Properties",app)
devpropAct.triggered.connect(ShowProperties)
deviceMenu.addAction(devpropAct)

snapAct =  QAction("Snap &Image",app)
snapAct.triggered.connect(SnapImage)
deviceMenu.addAction(snapAct)

layout = QHBoxLayout()
mainwindow = QWidget()
videowindow = QLabel()
layout.addWidget(videowindow)

mainwindow.setLayout(layout)
w.setCentralWidget(mainwindow)

# Create the IC Imaging Control object.
ic = TIS.Imaging.ICImagingControl()

# Instantiate the display filter object 
# for live display
displayFilter = DisplayFilter()

# Connect the display signal handler to our filter.
displayFilter.signals.display.connect(OnDisplay)

ic.DisplayFrameFilters.Add( ic.FrameFilterCreate(displayFilter))
# Create a sing for simple image snap and save.

# Create the sink for snapping images on demand.
snapsink = TIS.Imaging.FrameSnapSink(TIS.Imaging.MediaSubtypes.RGB32)
ic.Sink = snapsink

ic.LiveDisplay = True

# Try to open the last used video capture device.
try:
    ic.LoadDeviceStateFromFile("device.xml",True)
    if ic.DeviceValid is True:
        serialNumber = ic.DeviceCurrent.GetSerialNumber()
        rosInterface = RosInterface(serialNumber=serialNumber) if serialNumber else RosInterface()
        ic.LiveStart()
except Exception as ex:
    print(ex)
    pass

w.show()

app.exec()
