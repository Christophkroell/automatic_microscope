import serial
import os
import numpy.core.multiarray
import cv2
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
is_simulation = False
if is_simulation:
    from picamera_sim import PiCamera
    import pty
else:
    from picamera import PiCamera
    import picamera.array
import time

camera = PiCamera()


class Controller:
    ser: serial.Serial

    def __int__(self, ser: serial.Serial):
        self.serial = ser

def find_serial_device(response_string: str) -> str:
    list_of_dev = os.listdir("/dev/")
    for file in list_of_dev:
        if not file.startswith('ttyUSB'):
            continue
        cur_serial_path = f"/dev/{file}"
        with serial.Serial(cur_serial_path, 115200) as ser:
            ser.write(",".encode())
            serial_input = ser.read_until(",".encode()).decode()
            if not serial_input.startswith(response_string):
                continue
            return cur_serial_path
    return ""


class SerialMonitor(QtCore.QThread):
    serial_input = QtCore.pyqtSignal(str)

    def __init__(self, serial_path):
        super(SerialMonitor, self).__init__()
        self.ser = serial.Serial(serial_path, 115200)

    def run(self):
        while True:
            new_input = self.ser.read_until(",".encode()).decode()
            self.serial_input.emit(new_input[:-1])
            #print(f"we got to do something with: {new_input}")


class CameraSettingsWidget(QtWidgets.QWidget):
    def __init__(self, camera, parent=None):
        super().__init__(parent)

        self.camera = camera
        self.initUI()

    def initUI(self):
        layout = QtWidgets.QFormLayout()

        # Add a label and spinbox for controlling the brightness
        self.brightness_label = QtWidgets.QLabel("Brightness")
        self.brightness_spinbox = QtWidgets.QSpinBox()
        self.brightness_spinbox.setRange(0, 100)
        self.brightness_spinbox.setValue(self.camera.brightness)
        self.brightness_spinbox.valueChanged.connect(self.setBrightness)
        layout.addRow(self.brightness_label, self.brightness_spinbox)

        # Add a label and spinbox for controlling the contrast
        self.contrast_label = QtWidgets.QLabel("Contrast")
        self.contrast_spinbox = QtWidgets.QSpinBox()
        self.contrast_spinbox.setRange(-100, 100)
        self.contrast_spinbox.setValue(self.camera.contrast)
        self.contrast_spinbox.valueChanged.connect(self.setContrast)
        layout.addRow(self.contrast_label, self.contrast_spinbox)

        # Add a label and spinbox for controlling the saturation
        self.saturation_label = QtWidgets.QLabel("Saturation")
        self.saturation_spinbox = QtWidgets.QSpinBox()
        self.saturation_spinbox.setRange(-100, 100)
        self.saturation_spinbox.setValue(self.camera.saturation)
        self.saturation_spinbox.valueChanged.connect(self.setSaturation)
        layout.addRow(self.saturation_label, self.saturation_spinbox)

        # Add a label and spinbox for controlling the sharpness
        self.sharpness_label = QtWidgets.QLabel("Sharpness")
        self.sharpness_spinbox = QtWidgets.QSpinBox()
        self.sharpness_spinbox.setRange(-100, 100)
        self.sharpness_spinbox.setValue(self.camera.sharpness)
        self.sharpness_spinbox.valueChanged.connect(self.setSharpness)
        layout.addRow(self.sharpness_label, self.sharpness_spinbox)

        # Add a label and spinbox for controlling the ISO
        self.iso_label = QtWidgets.QLabel("ISO")
        self.iso_spinbox = QtWidgets.QSpinBox()
        self.iso_spinbox.setRange(0, 800)
        self.iso_spinbox.setValue(self.camera.iso)
        self.iso_spinbox.valueChanged.connect(self.setISO)
        layout.addRow(self.iso_label, self.iso_spinbox)

        # Add a label and spinbox for controlling the shutter speed
        self.shutter_speed_label = QtWidgets.QLabel("Shutter Speed")
        self.shutter_speed_spinbox = QtWidgets.QSpinBox()
        self.shutter_speed_spinbox.setRange(0, 60000000)
        self.shutter_speed_spinbox.setValue(self.camera.shutter_speed)
        self.shutter_speed_spinbox.valueChanged.connect(self.setShutterSpeed)
        layout.addRow(self.shutter_speed_label, self.shutter_speed_spinbox)

        # Add a label and spinbox for controlling the exposure compensation
        self.exposure_compensation_label = QtWidgets.QLabel("Exposure Compensation")
        self.exposure_compensation_spinbox = QtWidgets.QSpinBox()
        self.exposure_compensation_spinbox.setRange(-25, 25)
        self.exposure_compensation_spinbox.setValue(self.camera.exposure_compensation)
        self.exposure_compensation_spinbox.valueChanged.connect(self.setExposureCompensation)
        layout.addRow(self.exposure_compensation_label, self.exposure_compensation_spinbox)

        # Add a label and spinbox for controlling the exposure mode
        self.exposure_mode_label = QtWidgets.QLabel("Exposure Mode")
        self.exposure_mode_combobox = QtWidgets.QComboBox()
        self.exposure_mode_combobox.addItems(
            ["off", "auto", "night", "nightpreview", "backlight", "spotlight", "sports", "snow", "beach", "verylong",
             "fixedfps", "antishake", "fireworks"])
        self.exposure_mode_combobox.setCurrentText(self.camera.exposure_mode)
        self.exposure_mode_combobox.currentTextChanged.connect(self.setExposureMode)
        layout.addRow(self.exposure_mode_label, self.exposure_mode_combobox)

        self.setLayout(layout)

    def setBrightness(self, value):
        self.camera.brightness = value

    def setContrast(self, value):
        self.camera.contrast = value

    def setSaturation(self, value):
        self.camera.saturation = value

    def setSharpness(self, value):
        self.camera.sharpness = value

    def setISO(self, value):
        self.camera.iso = value

    def setShutterSpeed(self, value):
        self.camera.shutter_speed = value

    def setExposureCompensation(self, value):
        self.camera.exposure_compensation = value

    def setExposureMode(self, value):
        self.camera.exposure_mode = value



class MicroscopeGui(QtWidgets.QMainWindow):
    def __init__(self):
        super(MicroscopeGui, self).__init__()
        is_simulation = True
        if is_simulation:
            master, slave = pty.openpty()
            path_to_controller = os.ttyname(slave)
            master, slave = pty.openpty()
            path_to_motor_FZT = os.ttyname(slave)
            master, slave = pty.openpty()
            path_to_motor_XYR = os.ttyname(slave)
            # print(f"sim serial_path: {path_to_motor_FZT}")
        else:
            path_to_motor_FZT = find_serial_device("motor_controller_FZT")
            path_to_motor_XYR = find_serial_device("motor_controller_XYR")
            path_to_controller = find_serial_device("controller")

        print(f"path to controller: {path_to_controller}")
        self.serial_motor_fzt = serial.Serial(path_to_motor_FZT, 115200)
        print(f"path to motor_FZT: {path_to_motor_FZT}")
        self.controller_thread = SerialMonitor(path_to_controller)
        self.controller_thread.serial_input.connect(self.handle_controller_input)
        self.controller_thread.start()
        camera_control_widget = CameraSettingsWidget(camera=camera)
        self.setCentralWidget(camera_control_widget)
        brightness = 50
        camera.brightness = brightness
        contrast = 0
        camera.contrast = contrast
        EV = 0
        saturation = 0
        camera.saturation = saturation
        camera.exposure_compensation = 0
        camera.sensor_mode = 2  # full field of view
        camera.resolution = (3280, 2464)  # Pi camera v2 max resolution, 8MP
        #camera.resolution = (1920, 1080)  # Pi camera v2 max resolution, 8MP
        camera.rotation = 180
        camera.annotate_text_size = 100
        camera.annotate_text = ""
        camera.iso = 0
        camera.shutter_speed = 0
        camera.framerate = 30
        camera.exposure_mode = 'auto'
        camera.awb_mode = 'auto'
        # camera.preview_fullscreen = False # optional
        # camera.preview_window = (0, 50, 1280, 960)  # optional
        # camera.start_preview()
        # with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.framerate = 30
        with picamera.array.PiRGBArray(camera, size=(640, 480)) as output:
            for frame in camera.capture_continuous(output, format="bgr", use_video_port=True):
                image = frame.array
                cv2.imshow("Video Stream", image)
                key = cv2.waitKey(1) & 0xFF
                output.truncate(0)
                if key == ord("q"):
                    break
        cv2.destroyAllWindows()

        #motor_controller_XYR

    def handle_controller_input(self, input_str):
        motor = input_str[0]
        ser = None
        max_mms = 2
        #print(f"input_str: {input_str}")
        if motor in ["F", "Z", "T", "L", "M"]:
            ser = self.serial_motor_fzt
            if motor in ["L", "M"]:
                max_mms = 256
        if motor in ["X", "Y", "R"]:
            #todo add xyr
            return
        if ser is None:
            return
        what_to_do = input_str[1]
        if what_to_do != "S":
            print(f"todo what_to_do: add {what_to_do} handler")
            return
        speed = int(input_str[2::])
        speed_mms = speed / 1000 * max_mms
        serial_string = f"{motor}S{speed_mms:3},"
        #print(f"serial_string: {serial_string}")
        ser.write(serial_string.encode())

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.serial_controller_fzt.close()
        self.serial_motor_fzt.close()

if __name__ == '__main__':
    time.sleep(5)
    app = QtWidgets.QApplication(sys.argv)
    microscope_gui = MicroscopeGui()
    microscope_gui.show()
    sys.exit(app.exec())