import serial
import os
import sys

print(f"os.name: {os.name}")
print(f"sys.platform: {sys.platform}")
is_simulation = False
if sys.platform == "darwin":
    is_simulation = True
import pty
if is_simulation:
    from picamera_sim import PiCamera
    from picamera_sim import PiRGBArray
    from PyQt6 import QtWidgets, QtCore, QtGui
else:
    import cv2
    from picamera import PiCamera
    from picamera.array import PiRGBArray
    from PyQt5 import QtWidgets, QtCore, QtGui
import time
import enum
from functools import partial
import shelve
import datetime
from Presets.Camera import CameraResolution, CameraResolutionOptions
from Widgets.CameraWidgets import CameraSettingsWidget
from Widgets.MotorWidgets import MotorWidget
from Hardware.MotorHW import Motor, MotionType

class Controller:
    ser: serial.Serial

    def __int__(self, ser: serial.Serial):
        self.serial = ser


def find_serial_device(response_string: str, device_path="ttyUSB") -> str:
    list_of_dev = os.listdir("/dev/")
    for file in list_of_dev:
        if not file.startswith(device_path):
        #if not file.startswith('tty.bt'):
            continue
        cur_serial_path = f"/dev/{file}"
        print(f"cur_serial_path: {cur_serial_path}")
        with serial.Serial(cur_serial_path, 115200) as ser:
            ser.write(",".encode())
            serial_input = ser.read_until(",".encode()).decode()
            print(f"serial_input for {cur_serial_path} : {serial_input}")
            if not serial_input.startswith(response_string):
                continue
            return cur_serial_path
    master, slave = pty.openpty()
    cur_serial_path = os.ttyname(slave)
    #ser = serial.Serial(cur_serial_path, 115200)
    print(f"couldn't connect to {response_string}, pty.openpty() was used: {cur_serial_path}")
    return cur_serial_path


class SerialMonitor(QtCore.QThread):
    serial_input = QtCore.pyqtSignal(str)

    def __init__(self, serial_path):
        super(SerialMonitor, self).__init__()
        #self.ser = serial_controller
        print(f"connect to: {serial_path}")
        self.serial_port = serial.Serial(serial_path, 115200)

    def run(self):
        while True:
            new_input = self.serial_port.read_until(",".encode()).decode()
            self.serial_input.emit(new_input[:-1])

    def stop(self):
        self.serial_port.close()



class VideoThread(QtCore.QThread):
    change_pixmap_signal = QtCore.pyqtSignal(QtGui.QImage)
    raw_capture: PiRGBArray
    camera_resolution: CameraResolution

    def __init__(self, camera, camera_resolution):
        super(VideoThread, self).__init__()
        self.camera_resolution = camera_resolution
        self.camera = camera

    def run(self):
        camera = self.camera
        mode = self.camera_resolution.sensor_mode
        resolution = self.camera_resolution.resolution
        #resolution = (2464, 2464)
        #mode = 2
        camera.sensor_mode = mode
        camera.resolution = resolution
        print(f"camera resolution: {self.camera.resolution}, mode: {camera.sensor_mode}")
        self.raw_capture = PiRGBArray(camera, size=resolution)

        for frame in camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):
            image = frame.array
            #print(image.shape)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w, ch = image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            self.change_pixmap_signal.emit(qt_image)
            self.raw_capture.truncate(0)

    def set_resolution(self, new_resolution: CameraResolution):
        self.camera.sensor_mode = new_resolution.sensor_mode
        self.camera.resolution = new_resolution.resolution
        self.raw_capture = PiRGBArray(self.camera, size=new_resolution.resolution)





class Light:
    max_brightness: int = 256
    light_value: int
    serial_interface: serial.Serial

    def __init__(self, name, serial_id, serial_interface):
        self.name = name
        self.serial_id = serial_id
        self.serial_interface = serial_interface

    def set_light_percentage(self, light_percentage):
        self.light_value = int(self.max_brightness * light_percentage)
        serial_string = f"{self.serial_id}S{self.light_value:3},"
        print(f"send to light: {serial_string}")
        self.send_serial_command(serial_string)

    def send_serial_command(self, command: str):
        self.serial_interface.write(command.encode())


class MicroscopeGui(QtWidgets.QWidget):
    camera: PiCamera
    serial_motor_fzt: serial.Serial
    serial_motor_xyr: serial.Serial
    light1: Light
    light2: Light
    motor_x: Motor
    motor_y: Motor
    motor_rotation: Motor
    motor_focus: Motor
    motor_zoom: Motor
    motor_tilt: Motor
    dict_of_motors: {str: Motor}
    dict_of_lights: {Light}

    def __init__(self):
        super(MicroscopeGui, self).__init__()
        self.camera = PiCamera()
        #time.sleep(2)
        self.motor_layout = QtWidgets.QVBoxLayout()
        self.setup_motors()
        path_serial_controller = find_serial_device("controller", device_path="ttyhh")
        #self.controller_thread = SerialMonitor(serial_path="/dev/tty.bt_controller")
        #self.controller_thread = SerialMonitor(serial_path="/dev/rfcomm0")
        self.controller_thread = SerialMonitor(serial_path=path_serial_controller)
        self.controller_thread.serial_input.connect(self.handle_controller_input)
        self.controller_thread.start()

        used_resolution = CameraResolutionOptions.max_resolution

        camera_control_widget = CameraSettingsWidget(camera=self.camera)
        camera_control_widget.update_camera_resolution(used_resolution=used_resolution)
        main_layout = QtWidgets.QHBoxLayout()
        self.setLayout(main_layout)

        control_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_layout)

        stop_button = QtWidgets.QPushButton("stop")
        stop_button.pressed.connect(self.controller_thread.stop)
        control_layout.addWidget(stop_button)

        capture_image_button = QtWidgets.QPushButton("Capture")
        capture_image_button.pressed.connect(self.capture_image)
        control_layout.addWidget(capture_image_button)

        print(f"is_simulation: {is_simulation}")
        if not is_simulation:
            if True:
                self.video_thread = VideoThread(camera=self.camera, camera_resolution=used_resolution)
                self.video_thread.change_pixmap_signal.connect(self.update_image)
                self.video_thread.start()
                self.video_widget = QtWidgets.QLabel()
                self.video_widget.setFixedSize(900, 900)
                main_layout.addWidget(self.video_widget)
            else:
                self.camera.start_preview()
        main_layout.addWidget(camera_control_widget)
        main_layout.addLayout(self.motor_layout)

    def capture_image(self):
        date = datetime.datetime.now().strftime("%Y_%m_%d %H:%M:%S")
        if not os.path.exists("images"):
            os.mkdir("images")
        self.camera.capture(f"images/{date}.jpg")

    def setup_motors(self):
        path_serial_motor_fzt = find_serial_device("motor_controller_FZT", device_path="ttyUSB")
        path_serial_motor_xyr = find_serial_device("motor_controller_XYR", device_path="ttyACM")
        self.serial_motor_fzt = serial.Serial(path_serial_motor_fzt, 115200)
        self.serial_motor_xyr = serial.Serial(path_serial_motor_xyr, 115200)
        self.motor_focus = Motor(name="focus", serial_id="F", motion_type=MotionType.linear,
                                 serial_interface=self.serial_motor_fzt, units_per_rotation=35.3)
        self.motor_zoom = Motor(name="zoom", serial_id="Z", motion_type=MotionType.linear,
                                serial_interface=self.serial_motor_fzt, units_per_rotation=35.3)
        self.motor_tilt = Motor(name="tilt", serial_id="T", motion_type=MotionType.rotary,
                                serial_interface=self.serial_motor_fzt, units_per_rotation=360*15/140)
        self.motor_x = Motor(name="x", serial_id="X", motion_type=MotionType.linear,
                             serial_interface=self.serial_motor_xyr, units_per_rotation=35.3)
        self.motor_y = Motor(name="y", serial_id="Y", motion_type=MotionType.linear,
                             serial_interface=self.serial_motor_xyr, units_per_rotation=35.3)
        self.motor_rotation = Motor(name="rotation", serial_id="R", motion_type=MotionType.rotary,
                                    serial_interface=self.serial_motor_xyr, units_per_rotation=360*15/120)

        self.dict_of_motors = {
            "F": self.motor_focus,
            "Z": self.motor_zoom,
            "T": self.motor_tilt,
            "X": self.motor_x,
            "Y": self.motor_y,
            "R": self.motor_rotation
        }
        for motor in self.dict_of_motors.values():
            self.motor_layout.addWidget(MotorWidget(motor))

        self.light1 = Light("light_1", "L", self.serial_motor_fzt)
        self.light2 = Light("light_2", "M", self.serial_motor_fzt)

        self.dict_of_lights = {
            "L": self.light1,
            "M": self.light2
        }

    def get_all_motor_positions(self):

        pass

    def update_image(self, qt_image:QtGui.QImage):
        pixmap = QtGui.QPixmap.fromImage(qt_image.scaledToWidth(800))
        self.video_widget.setPixmap(pixmap)

    def handle_controller_input(self, serial_input):
        target_id = serial_input[0]
        if target_id in self.dict_of_motors:
            motor = self.dict_of_motors[target_id]
            request_type = serial_input[1]
            if request_type == 'S':
                speed_factor = float(serial_input[2::]) * 1e-3
                motor.set_speed_mode_multiplier(speed_factor)
                print(f"speed_factor: {speed_factor}")
            return
        if target_id in self.dict_of_lights:
            led = self.dict_of_lights[target_id]
            light_percentage = float(serial_input[2::]) * 1e-3
            led: Light
            led.set_light_percentage(light_percentage)
            return

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        #self.serial_controller.close()
        self.serial_motor_xyr.close()
        self.serial_motor_fzt.close()


if __name__ == '__main__':
    time.sleep(2)
    app = QtWidgets.QApplication(sys.argv)
    microscope_gui = MicroscopeGui()
    microscope_gui.show()
    sys.exit(app.exec())
