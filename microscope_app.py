import serial
import os
import abc

from PyQt6 import QtWidgets, QtCore, QtGui
import sys
is_simulation = True
import pty
if is_simulation:
    from picamera_sim import PiCamera
else:
    import cv2
    from picamera import PiCamera
    import picamera.array
    from picamera.array import PiRGBArray
import time
import enum


class MotorState(enum.Enum):
    UNKNOWN = enum.auto
    HOMING = enum.auto
    READY = enum.auto
    MOVING = enum.auto
    ERROR = enum.auto
    SPEEDMODE = enum.auto


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


class CameraSettingsWidget(QtWidgets.QWidget):
    framerate: int = 10
    brightness: int = 50
    contrast: int = 0
    saturation: int = 0
    sharpness: int = 0
    iso: int = 60
    shutter_speed: int = 30000
    exposure_compensation: int = 0
    exposure_mode: str = "off"
    awb_mode: str = "off"
    awb_gains: tuple = (1, 1)

    def __init__(self, camera, parent=None):
        super().__init__(parent)

        self.camera = camera
        self.initUI()

    def initUI(self):
        layout = QtWidgets.QFormLayout()

        # Add a label and spinbox for controlling the framerate
        framerate_spinbox = QtWidgets.QSpinBox()
        framerate_spinbox.setRange(1, 30)
        framerate_spinbox.valueChanged.connect(self.set_framerate)
        framerate_spinbox.setValue(self.framerate)
        layout.addRow("Framerate", framerate_spinbox)

        # Add a label and spinbox for controlling the brightness
        brightness_spinbox = QtWidgets.QSpinBox()
        brightness_spinbox.setRange(0, 100)
        brightness_spinbox.setValue(self.camera.brightness)
        brightness_spinbox.valueChanged.connect(self.setBrightness)
        layout.addRow("Brightness", brightness_spinbox)

        # Add a label and spinbox for controlling the contrast
        contrast_spinbox = QtWidgets.QSpinBox()
        contrast_spinbox.setRange(-100, 100)
        contrast_spinbox.setValue(self.camera.contrast)
        contrast_spinbox.valueChanged.connect(self.setContrast)
        layout.addRow("Contrast", contrast_spinbox)

        # Add a label and spinbox for controlling the saturation
        saturation_spinbox = QtWidgets.QSpinBox()
        saturation_spinbox.setRange(-100, 100)
        saturation_spinbox.setValue(self.camera.saturation)
        saturation_spinbox.valueChanged.connect(self.setSaturation)
        layout.addRow("Saturation", saturation_spinbox)

        # Add a label and spinbox for controlling the sharpness
        sharpness_spinbox = QtWidgets.QSpinBox()
        sharpness_spinbox.setRange(-100, 100)
        sharpness_spinbox.setValue(self.camera.sharpness)
        sharpness_spinbox.valueChanged.connect(self.setSharpness)
        layout.addRow("Sharpness", sharpness_spinbox)

        # Add a label and spinbox for controlling the ISO
        iso_spinbox = QtWidgets.QSpinBox()
        iso_spinbox.setRange(0, 800)
        iso_spinbox.setValue(self.camera.iso)
        iso_spinbox.valueChanged.connect(self.setISO)
        layout.addRow("ISO", iso_spinbox)

        # Add a label and spinbox for controlling the shutter speed
        shutter_speed_spinbox = QtWidgets.QSpinBox()
        shutter_speed_spinbox.setRange(0, 60000000)
        shutter_speed_spinbox.setValue(self.camera.shutter_speed)
        shutter_speed_spinbox.valueChanged.connect(self.setShutterSpeed)
        layout.addRow("Shutter Speed", shutter_speed_spinbox)

        # Add a label and spinbox for controlling the exposure compensation
        exposure_compensation_spinbox = QtWidgets.QSpinBox()
        exposure_compensation_spinbox.setRange(-25, 25)
        exposure_compensation_spinbox.setValue(self.camera.exposure_compensation)
        exposure_compensation_spinbox.valueChanged.connect(self.setExposureCompensation)
        layout.addRow("Exposure Compensation", exposure_compensation_spinbox)

        # Add a label and spinbox for controlling the exposure mode
        exposure_mode_combobox = QtWidgets.QComboBox()
        exposure_mode_combobox.addItems(
            ["off", "auto", "night", "nightpreview", "backlight", "spotlight", "sports", "snow", "beach", "verylong",
             "fixedfps", "antishake", "fireworks"])
        exposure_mode_combobox.setCurrentText(self.camera.exposure_mode)
        exposure_mode_combobox.currentTextChanged.connect(self.setExposureMode)
        layout.addRow("Exposure Mode", exposure_mode_combobox)

        # Add a label and spinbox for controlling the awb mode
        awb_mode_label = QtWidgets.QLabel("AWB Mode")
        awb_mode_combobox = QtWidgets.QComboBox()
        awb_mode_combobox.addItems(["off", "auto"])
        awb_mode_combobox.setCurrentText(self.camera.awb_mode)
        awb_mode_combobox.currentTextChanged.connect(self.set_awb_mode)
        layout.addRow(awb_mode_label, awb_mode_combobox)

        # Add a label and spinbox for controlling the exposure compensation
        #self.exposure_compensation_label = QtWidgets.QLabel("Exposure Compensation")
        self.awb_gains = self.camera.awb_gains
        awb_gain1_spinbox = QtWidgets.QDoubleSpinBox()
        awb_gain2_spinbox = QtWidgets.QDoubleSpinBox()
        awb_gain1_spinbox.setRange(0, 8)
        awb_gain2_spinbox.setRange(0, 8)
        awb_gain1_spinbox.setValue(self.awb_gains[0])
        awb_gain2_spinbox.setValue(self.awb_gains[1])
        awb_gain1_spinbox.valueChanged.connect(self.set_awb_gain1)
        awb_gain2_spinbox.valueChanged.connect(self.set_awb_gain2)
        gain_layout = QtWidgets.QHBoxLayout()
        gain_layout.addWidget(awb_gain1_spinbox)
        gain_layout.addWidget(awb_gain2_spinbox)

        layout.addRow("AWB gains", gain_layout)

        self.setLayout(layout)

    def set_framerate(self, value):
        self.framerate = value
        self.camera.framerate = value

    def setBrightness(self, value):
        self.brightness = value
        self.camera.brightness = value

    def setContrast(self, value):
        self.contrast = value
        self.camera.contrast = value

    def setSaturation(self, value):
        self.saturation = value
        self.camera.saturation = value

    def setSharpness(self, value):
        self.sharpness = value
        self.camera.sharpness = value

    def setISO(self, value):
        self.iso = value
        self.camera.iso = value

    def setShutterSpeed(self, value):
        self.shutter_speed = value
        self.camera.shutter_speed = value

    def setExposureCompensation(self, value):
        self.exposure_compensation = value
        self.camera.exposure_compensation = value

    def setExposureMode(self, value):
        self.exposure_mode = value
        self.camera.exposure_mode = value

    def set_awb_mode(self, value):
        self.awb_mode = value
        self.camera.awb_mode = value

    def set_awb_gain1(self, value):
        self.awb_gains = (value, self.awb_gains[1])
        self.camera.awb_gains = self.awb_gains

    def set_awb_gain2(self, value):
        self.awb_gains = (self.awb_gains[0], value)
        self.camera.awb_gains = self.awb_gains


class VideoThread(QtCore.QThread):
    change_pixmap_signal = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, camera):
        super(VideoThread, self).__init__()
        self.camera = camera

    def run(self):
        camera = self.camera
        camera.resolution = (640, 480)
        #camera.framerate = 24
        rawCapture = PiRGBArray(camera, size=(640, 480))

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w, ch = image.shape
            bytesPerLine = ch * w
            qt_image = QtGui.QImage(image.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.change_pixmap_signal.emit(qt_image)
            rawCapture.truncate(0)


class MotionControlWidget(QtWidgets.QWidget):
    def __init__(self):
        super(MotionControlWidget, self).__init__()


class MotionType:
    linear: str = "linear"
    rotary: str = "rotary"


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

    def send_serial_command(self, command:str):
        return
        self.serial_interface.write(command.encode())


class BaseMotor:
    name: str
    state: MotorState
    steps_per_rotation: int = 4096
    motion_type: MotionType
    serial_interface: serial.Serial
    unit_per_step: float

    def __init__(self, name, serial_id, motion_type, serial_interface, unit_per_step):
        self.state = MotorState.UNKNOWN
        self.name = name
        self.serial_id = serial_id
        self.type = motion_type
        self.serial_interface = serial_interface
        self.unit_per_step = unit_per_step

    def set_speed_mode_speed(self, speed: float):
        steps_per_second = speed * self.unit_per_step
        serial_string = f"{self.serial_id}S{steps_per_second:3},"
        print(f"send to motor: {serial_string}")
        self.send_serial_command(serial_string)

    def send_serial_command(self, command:str):
        return
        self.serial_interface.write(command.encode())


    def __str__(self):
        return f"name: {self.name}, id: {self.serial_id}, type: {self.type}, interface: {self.serial_interface}"


class RotaryMotor:
    home_speed_degs: float = 0
    home_direction: int = 1
    limit_min_deg: float = -0.01
    limit_max_deg: float = 100
    speed_degs: float = 0
    max_speed_degs: float = 1
    acceleration_degs2: float = 4
    deg_per_motor_rotation: float
    #deg_per_motor_rotation: float = 360/steps_per_rotation * 15/120 # 120 tilt gear 0.010986
    # deg_per_step: float = 360/steps_per_rotation * 15/140 # 140 rotary gear 0.009416852678571

    def __init__(self, name, serial_id, motion_type, serial_interface, gear_ratio):
        self.deg_per_motor_rotation = 360 * gear_ratio  # 0.0086181640625
        self.motor = BaseMotor(name, serial_id, motion_type, serial_interface, self.deg_per_motor_rotation)
        self.name = name
        self.serial_id = serial_id
        self.type = motion_type
        self.serial_interface = serial_interface

    def set_speed_mode_multiplier(self, speed_multiplier):
        if speed_multiplier == 0:
            # todo ask motor for position
            self.motor.state = MotorState.READY
        else:
            self.motor.state = MotorState.MOVING
        speed = self.max_speed_degs * speed_multiplier
        print(f"speed: {speed}")
        self.motor.set_speed_mode_speed(speed)


class LinearMotor:
    home_speed_mms: float = 0
    home_direction: int = 1
    limit_min_mm: float = -0.01
    limit_max_mm: float = 100
    speed_mms: float = 0
    max_speed_mms: float = 2
    acceleration_mms2: float = 4
    mm_per_motor_rotation: float
    #mm_per_step: float = 35.3/steps_per_rotation

    def __init__(self, name, serial_id, motion_type, serial_interface, circumference_mm=35.3):
        self.mm_per_motor_rotation = circumference_mm # 0.0086181640625
        self.motor = BaseMotor(name, serial_id, motion_type, serial_interface, self.mm_per_motor_rotation)

    def set_speed_mode_multiplier(self, speed_multiplier):
        speed = self.max_speed_mms * speed_multiplier
        print(f"speed: {speed}")
        self.motor.set_speed_mode_speed(speed)




class MicroscopeGui(QtWidgets.QWidget):
    camera: PiCamera
    serial_motor_fzt: serial.Serial
    serial_motor_xyr: serial.Serial
    light1: Light
    light2: Light
    motor_x: LinearMotor
    motor_y: LinearMotor
    motor_rotation: RotaryMotor
    motor_focus: LinearMotor
    motor_zoom: LinearMotor
    motor_tilt: RotaryMotor
    dict_of_motors: {LinearMotor}
    dict_of_lights: {Light}

    def __init__(self):
        super(MicroscopeGui, self).__init__()
        self.camera = PiCamera()
        #time.sleep(2)
        self.setup_motors()
        #path_serial_controller = find_serial_device("controller", device_path="tty.bt")
        self.controller_thread = SerialMonitor(serial_path="/dev/tty.bt_controller")
        self.controller_thread.serial_input.connect(self.handle_controller_input)
        self.controller_thread.start()

        camera_control_widget = CameraSettingsWidget(camera=self.camera)
        main_layout = QtWidgets.QHBoxLayout()
        self.setLayout(main_layout)
        stop_button = QtWidgets.QPushButton("stop")
        stop_button.pressed.connect(self.controller_thread.stop)
        main_layout.addWidget(stop_button)

        if not is_simulation:
            self.thread = VideoThread(camera=self.camera)
            self.thread.change_pixmap_signal.connect(self.update_image)
            self.thread.start()
            self.video_widget = QtWidgets.QLabel()
            self.video_widget.setFixedSize(500, 500)
            main_layout.addWidget(self.video_widget)

        main_layout.addWidget(camera_control_widget)

    def setup_motors(self):
        path_serial_motor_fzt = find_serial_device("motor_controller_FZT")
        path_serial_motor_xyr = find_serial_device("motor_controller_XYR")
        self.serial_motor_fzt = serial.Serial(path_serial_motor_fzt, 115200)
        self.serial_motor_xyr = serial.Serial(path_serial_motor_xyr, 115200)
        self.motor_focus = LinearMotor(name="focus", serial_id="F", motion_type=MotionType.linear,
                                       serial_interface=self.serial_motor_fzt)
        self.motor_zoom = LinearMotor(name="zoom", serial_id="Z", motion_type=MotionType.linear,
                                      serial_interface=self.serial_motor_fzt)
        self.motor_tilt = RotaryMotor(name="tilt", serial_id="T", motion_type=MotionType.rotary,
                                      serial_interface=self.serial_motor_fzt, gear_ratio=15/140)

        self.motor_x = LinearMotor(name="x", serial_id="X", motion_type=MotionType.linear,
                                   serial_interface=self.serial_motor_xyr)
        self.motor_y = LinearMotor(name="y", serial_id="Y", motion_type=MotionType.linear,
                                   serial_interface=self.serial_motor_xyr)
        self.motor_rotation = RotaryMotor(name="tilt", serial_id="R", motion_type=MotionType.rotary,
                                          serial_interface=self.serial_motor_xyr, gear_ratio=15/120)
        self.dict_of_motors = {
            "F": self.motor_focus,
            "Z": self.motor_zoom,
            "T": self.motor_tilt,
            "X": self.motor_x,
            "Y": self.motor_y,
            "R": self.motor_rotation
        }

        self.light1 = Light("light_1", "L", self.serial_motor_fzt)
        self.light2 = Light("light_2", "M", self.serial_motor_fzt)

        self.dict_of_lights = {
            "L": self.light1,
            "M": self.light2
        }

    def get_all_motor_positions(self):

        pass

    def update_image(self, qt_image):
        pixmap = QtGui.QPixmap.fromImage(qt_image)
        self.video_widget.setPixmap(pixmap)

    def handle_controller_input(self, serial_input):
        target_id = serial_input[0]
        if target_id in self.dict_of_motors:
            motor = self.dict_of_motors[target_id]
            request_type = serial_input[1]
            if request_type == 'S':
                speed_factor = float(serial_input[2::]) * 1e-3
                motor: LinearMotor
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
    time.sleep(5)
    app = QtWidgets.QApplication(sys.argv)
    microscope_gui = MicroscopeGui()
    microscope_gui.show()
    sys.exit(app.exec())