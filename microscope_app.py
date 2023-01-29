import serial
import os
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
from picamera import PiCamera
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


class MicroscopeGui(QtWidgets.QMainWindow):
    def __init__(self):
        super(MicroscopeGui, self).__init__()
        path_to_controller = find_serial_device("controller")
        print(f"path to controller: {path_to_controller}")
        path_to_motor_FZT = find_serial_device("motor_controller_FZT")
        self.serial_motor_fzt = serial.Serial(path_to_motor_FZT, 115200)
        print(f"path to motor_FZT: {path_to_motor_FZT}")
        self.controller_thread = SerialMonitor(path_to_controller)
        self.controller_thread.serial_input.connect(self.handle_controller_input)
        self.controller_thread.start()
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
        camera.start_preview()

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
    sys.exit(app.exec_())