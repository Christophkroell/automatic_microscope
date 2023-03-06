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


class CameraResolution:
    sensor_mode: int
    resolution: tuple
    frame_rates: tuple

    def __init__(self, sensor_mode, resolution, frame_rates):
        self.sensor_mode = sensor_mode
        self.resolution = resolution
        self.frame_rates = frame_rates


class CameraResolutionOptions:
    max_resolution = CameraResolution(sensor_mode=2, resolution=(3280, 2464), frame_rates=(0.5, 15))
    half_resolution = CameraResolution(sensor_mode=4, resolution=(1640, 1232), frame_rates=(0.5, 40))
    low_resolution = CameraResolution(sensor_mode=7, resolution=(640, 480), frame_rates=(40, 90))


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


class CameraSettings:
    used_resolution: CameraResolution = CameraResolutionOptions.max_resolution
    framerate: int = 10
    brightness: int = 50
    contrast: int = 0
    saturation: int = 0
    sharpness: int = 0
    iso: int = 60
    exposure_time_ms: int = 30
    exposure_compensation: int = 0
    exposure_mode: str = "off"
    awb_mode: str = "off"
    awb_gains: tuple = (1, 1)


class CameraSettingsWidget(QtWidgets.QWidget):
    settings = CameraSettings
    framerate_spinbox: QtWidgets.QDoubleSpinBox
    exposure_time_ms_spinbox: QtWidgets.QSpinBox

    def __init__(self, camera, parent=None):
        super().__init__(parent)
        self.camera = camera
        setting_path = "settings/camera.shelve"
        if os.path.exists(setting_path):
            self.shelved_parameter = shelve.open(setting_path, writeback=True)
            self.settings = self.shelved_parameter["camera_settings"]
        else:
            self.settings = CameraSettings()
            self.shelved_parameter = shelve.open(setting_path, writeback=True)
            self.shelved_parameter["camera_settings"] = self.settings
        #self.used_resolution = used_resolution
        self.initUI()

    def initUI(self):
        layout = QtWidgets.QFormLayout()

        sensor_mode_combobox = QtWidgets.QComboBox()
        self.dict_of_sensor_modes = {"Low": CameraResolutionOptions.low_resolution,
                                     "Medium": CameraResolutionOptions.half_resolution,
                                     "High": CameraResolutionOptions.max_resolution}
        sensor_mode_combobox.addItems(self.dict_of_sensor_modes)
        sensor_mode_combobox.currentTextChanged.connect(self.sensor_mode_changed)
        self.sensor_mode_combobox = sensor_mode_combobox
        layout.addRow("Sensor", sensor_mode_combobox)

        # Add a label and spinbox for controlling the framerate
        framerate_spinbox = QtWidgets.QDoubleSpinBox()
        framerate_spinbox.setRange(1, 30)
        framerate_spinbox.valueChanged.connect(self.set_framerate)
        self.framerate_spinbox = framerate_spinbox
        layout.addRow("Framerate", framerate_spinbox)

        # Add a label and spinbox for controlling the brightness
        brightness_spinbox = QtWidgets.QSpinBox()
        brightness_spinbox.setRange(0, 100)
        brightness_spinbox.valueChanged.connect(self.setBrightness)
        layout.addRow("Brightness", brightness_spinbox)

        # Add a label and spinbox for controlling the contrast
        contrast_spinbox = QtWidgets.QSpinBox()
        contrast_spinbox.setRange(-100, 100)
        contrast_spinbox.valueChanged.connect(self.setContrast)
        layout.addRow("Contrast", contrast_spinbox)

        # Add a label and spinbox for controlling the saturation
        saturation_spinbox = QtWidgets.QSpinBox()
        saturation_spinbox.setRange(-100, 100)
        saturation_spinbox.valueChanged.connect(self.setSaturation)
        layout.addRow("Saturation", saturation_spinbox)

        # Add a label and spinbox for controlling the sharpness
        sharpness_spinbox = QtWidgets.QSpinBox()
        sharpness_spinbox.setRange(-100, 100)
        sharpness_spinbox.valueChanged.connect(self.setSharpness)
        layout.addRow("Sharpness", sharpness_spinbox)

        # Add a label and spinbox for controlling the ISO
        iso_spinbox = QtWidgets.QSpinBox()
        iso_spinbox.setRange(60, 800)
        iso_spinbox.valueChanged.connect(self.setISO)
        layout.addRow("ISO", iso_spinbox)

        # Add a label and spinbox for controlling the shutter speed
        exposure_time_ms_spinbox = QtWidgets.QSpinBox()
        #exposure_time_ms_spinbox.setRange(0, 600)
        exposure_time_ms_spinbox.valueChanged.connect(self.set_exposure_time_ms)
        self.exposure_time_ms_spinbox = exposure_time_ms_spinbox
        layout.addRow("Exposure time ms", exposure_time_ms_spinbox)

        # Add a label and spinbox for controlling the exposure compensation
        exposure_compensation_spinbox = QtWidgets.QSpinBox()
        exposure_compensation_spinbox.setRange(-25, 25)
        exposure_compensation_spinbox.valueChanged.connect(self.setExposureCompensation)
        layout.addRow("Exposure Compensation", exposure_compensation_spinbox)

        # Add a label and spinbox for controlling the exposure mode
        exposure_mode_combobox = QtWidgets.QComboBox()
        exposure_mode_combobox.addItems(
            ["off", "auto", "night", "nightpreview", "backlight", "spotlight", "sports", "snow", "beach", "verylong",
             "fixedfps", "antishake", "fireworks"])
        exposure_mode_combobox.currentTextChanged.connect(self.setExposureMode)
        layout.addRow("Exposure Mode", exposure_mode_combobox)

        # Add a label and spinbox for controlling the awb mode
        awb_mode_label = QtWidgets.QLabel("AWB Mode")
        awb_mode_combobox = QtWidgets.QComboBox()
        awb_mode_combobox.addItems(["off", "auto"])
        awb_mode_combobox.currentTextChanged.connect(self.set_awb_mode)
        layout.addRow(awb_mode_label, awb_mode_combobox)

        # Add a label and spinbox for controlling the exposure compensation
        #self.exposure_compensation_label = QtWidgets.QLabel("Exposure Compensation")
        self.settings.awb_gains = self.camera.awb_gains
        awb_gain1_spinbox = QtWidgets.QDoubleSpinBox()
        awb_gain2_spinbox = QtWidgets.QDoubleSpinBox()
        awb_gain1_spinbox.setRange(0, 8)
        awb_gain2_spinbox.setRange(0, 8)
        awb_gain1_spinbox.valueChanged.connect(self.set_awb_gain1)
        awb_gain2_spinbox.valueChanged.connect(self.set_awb_gain2)
        gain_layout = QtWidgets.QHBoxLayout()
        gain_layout.addWidget(awb_gain1_spinbox)
        gain_layout.addWidget(awb_gain2_spinbox)

        layout.addRow("AWB gains", gain_layout)

        self.setLayout(layout)

        framerate_spinbox.setValue(self.settings.framerate)
        brightness_spinbox.setValue(self.settings.brightness)
        contrast_spinbox.setValue(self.settings.contrast)
        saturation_spinbox.setValue(self.settings.saturation)
        sharpness_spinbox.setValue(self.settings.sharpness)
        iso_spinbox.setValue(self.settings.iso)
        exposure_time_ms_spinbox.setValue(self.settings.exposure_time_ms)
        exposure_compensation_spinbox.setValue(self.settings.exposure_compensation)
        exposure_mode_combobox.setCurrentText(self.settings.exposure_mode)
        awb_mode_combobox.setCurrentText(self.settings.awb_mode)
        awb_gain1_spinbox.setValue(self.settings.awb_gains[0])
        awb_gain2_spinbox.setValue(self.settings.awb_gains[1])

    def update_shelve(self):
        self.shelved_parameter["camera_settings"] = self.settings
        self.shelved_parameter.sync()

    def sensor_mode_changed(self, sensor_mode: str):
        self.update_camera_resolution(self.dict_of_sensor_modes[sensor_mode])

    def update_camera_resolution(self, used_resolution: CameraResolution):
        self.framerate_spinbox.setMinimum(used_resolution.frame_rates[0])
        self.framerate_spinbox.setMaximum(used_resolution.frame_rates[1])
        min_exposure_time_ms = int(1*1e3/used_resolution.frame_rates[1])
        max_exposure_time_ms = int(1*1e3/used_resolution.frame_rates[0])
        print(f"min_sutter: {min_exposure_time_ms}, max_exposure_time_ms: {max_exposure_time_ms}")
        self.exposure_time_ms_spinbox.setMinimum(min_exposure_time_ms)
        self.exposure_time_ms_spinbox.setMaximum(max_exposure_time_ms)
        self.camera.sensor_mode = used_resolution.sensor_mode
        self.camera.resolution = used_resolution.resolution
        self.settings.used_resolution = used_resolution
        self.update_shelve()

    def set_framerate(self, framerate):
        self.settings.framerate = framerate
        self.camera.framerate = framerate
        max_exposure_time_ms = int(1 * 1e3 / framerate)
        self.exposure_time_ms_spinbox.setMaximum(max_exposure_time_ms)
        self.update_shelve()

    def setBrightness(self, value):
        self.settings.brightness = value
        self.camera.brightness = value
        self.update_shelve()

    def setContrast(self, value):
        self.settings.contrast = value
        self.camera.contrast = value
        self.update_shelve()

    def setSaturation(self, value):
        self.settings.saturation = value
        self.camera.saturation = value
        self.update_shelve()

    def setSharpness(self, value):
        self.settings.sharpness = value
        self.camera.sharpness = value
        self.update_shelve()

    def setISO(self, value):
        self.settings.iso = value
        self.camera.iso = value
        self.update_shelve()

    def set_exposure_time_ms(self, exposure_time_ms):
        self.settings.exposure_time_ms = exposure_time_ms
        self.camera.shutter_speed = int(exposure_time_ms * 1e3)
        self.update_shelve()

    def setExposureCompensation(self, value):
        self.settings.exposure_compensation = value
        self.camera.exposure_compensation = value
        self.update_shelve()

    def setExposureMode(self, value):
        self.settings.exposure_mode = value
        self.camera.exposure_mode = value
        self.update_shelve()

    def set_awb_mode(self, value):
        self.settings.awb_mode = value
        self.camera.awb_mode = value
        self.update_shelve()

    def set_awb_gain1(self, value):
        self.settings.awb_gains = (value, self.settings.awb_gains[1])
        self.camera.awb_gains = self.settings.awb_gains
        self.update_shelve()

    def set_awb_gain2(self, value):
        self.settings.awb_gains = (self.settings.awb_gains[0], value)
        self.camera.awb_gains = self.settings.awb_gains
        self.update_shelve()


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
        camera.sensor_mode = self.camera_resolution.sensor_mode
        camera.resolution = self.camera_resolution.resolution
        #camera.framerate = 24
        self.raw_capture = PiRGBArray(camera, size=self.camera_resolution.resolution)

        for frame in camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):
            image = frame.array
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w, ch = image.shape
            bytesPerLine = ch * w
            qt_image = QtGui.QImage(image.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.change_pixmap_signal.emit(qt_image)
            self.raw_capture.truncate(0)

    def set_resolution(self, new_resolution:CameraResolution):
        self.camera.sensor_mode = new_resolution.sensor_mode
        self.camera.resolution = new_resolution.resolution
        self.raw_capture = PiRGBArray(self.camera, size=new_resolution.resolution)


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

    def send_serial_command(self, command: str):
        self.serial_interface.write(command.encode())


class BaseMotor:
    name: str
    state: MotorState
    steps_per_rotation: int = 4096
    motion_type: MotionType
    serial_interface: serial.Serial
    unit_per_step: float
    position_in_steps: float

    def __init__(self, name, serial_id, motion_type, serial_interface: serial.Serial):
        self.state = MotorState.UNKNOWN
        self.name = name
        self.serial_id = serial_id
        self.type = motion_type
        self.serial_interface = serial_interface
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_motor_position)
        #self.update_timer.start(250)

    def set_speed_mode(self, motor_rotation_per_second: float):
        print(f"motor_rotation_per_second: {motor_rotation_per_second}")
        steps_per_second = motor_rotation_per_second * self.steps_per_rotation
        serial_string = f"{self.serial_id}S{steps_per_second:.3f},"
        print(f"send to motor: {serial_string}")
        self.send_serial_command(serial_string)

    def send_serial_command(self, command: str):
        self.serial_interface.write(command.encode())

    def update_motor_position(self):
        serial_command = f"{self.serial_id}G,"
        self.send_serial_command(serial_command)
        new_input = self.serial_interface.read_until(",".encode()).decode()
        if serial_command[0:2] != new_input[0:2]:
            raise ValueError(f"Wrong return: {serial_command} != {new_input}")
        self.position_in_steps = float(new_input[2:-1])
        print(f"updated motor position_in_steps: {self.position_in_steps}")
        return

    def get_position_in_rotations(self):
        self.update_motor_position()
        return self.position_in_steps / self.steps_per_rotation

    def __str__(self):
        return f"name: {self.name}, id: {self.serial_id}, type: {self.type}, interface: {self.serial_interface}"


class MotorParameters:
    motion_type: MotionType
    motor_enabled: bool
    speed_unit: str
    position_unit: str
    home_speed: float
    home_direction: int
    limit_min: float
    limit_max: float
    speed: float
    max_speed: float
    acceleration: float
    position_in_rotation: float
    position_per_rotation: float

    def __init__(self, motion_type: MotionType, units_per_rotation):
        self.units_per_rotation = units_per_rotation
        self.direction = 1
        if motion_type == MotionType.linear:
            self.linear_motor_settings()
            return
        if motion_type == MotionType.rotary:
            self.rotary_motor_settings()
            return

    def rotary_motor_settings(self):
        self.motor_enabled: bool = True
        self.position_unit: str = "deg"
        self.speed_unit: str = f"{self.position_unit}/s"
        self.home_speed: float = 1
        self.home_direction: int = 1
        self.limit_min: float = -90
        self.limit_max: float = 90
        self.speed: float = 1
        self.max_speed: float = 1
        self.acceleration: float = 4
        self.position_per_rotation: float = 0
        self.position_in_rotation: float = 0

    def linear_motor_settings(self):
        self.motor_enabled: bool = True
        self.position_unit: str = "mm"
        self.speed_unit: str = f"{self.position_unit}/s"
        self.home_speed: float = 1
        self.home_direction: int = 1
        self.limit_min: float = -0.01
        self.limit_max: float = 100
        self.speed: float = 0
        self.max_speed: float = 2
        self.acceleration: float = 4
        self.position_per_rotation: float = 0
        self.position_in_rotation: float = 0


class Motor:
    position: float = 0

    def __init__(self, name, serial_id, motion_type, serial_interface, units_per_rotation):
        self.name = name
        if not os.path.exists("settings"):
            os.mkdir("settings")
        shelve_path = f"settings/{name}.shelve"
        if os.path.exists(shelve_path):
            self.shelved_parameter = shelve.open(shelve_path, writeback=True)
            self.parameter = self.shelved_parameter["parameter"]
            self.parameter: MotorParameters
            # self.parameter_changed("motor_enabled", True)
        else:
            self.parameter = MotorParameters(motion_type=motion_type, units_per_rotation=units_per_rotation)
            self.shelved_parameter = shelve.open(shelve_path, writeback=True)
            self.shelved_parameter["parameter"] = self.parameter
        self.base = BaseMotor(name, serial_id, motion_type, serial_interface)

    def set_speed_mode_multiplier(self, speed_multiplier):
        units_per_rotation = self.parameter.units_per_rotation
        max_speed = self.parameter.max_speed
        direction = self.parameter.direction
        if speed_multiplier == 0:
            # todo ask motor for position
            self.base.state = MotorState.READY
        else:
            self.base.state = MotorState.MOVING
        speed = max_speed * speed_multiplier * units_per_rotation * direction
        print(f"speed: {speed}{self.parameter.speed_unit}")
        self.base.set_speed_mode(speed)

    def get_max_speed(self) -> float:
        return self.parameter.max_speed

    def set_max_speed(self, max_speed):
        self.parameter.max_speed = max_speed

    def get_position(self):
        self.position = self.base.get_position_in_rotations() / self.parameter.units_per_rotation
        print(f"steps_per_second: {self.position}")
        return self.position

    def parameter_changed(self, parameter_name, value):
        print(f"parameter_changed: {parameter_name}: {value}")
        self.parameter.__setattr__(parameter_name, value)
        self.shelved_parameter["parameter"] = self.parameter
        self.shelved_parameter.sync()


class MotorWidget(QtWidgets.QWidget):
    setup_widget: QtWidgets.QWidget
    def __init__(self, motor: Motor):
        super(MotorWidget, self).__init__()
        self.motor = motor
        self.name = motor.base.name

        main_layout = QtWidgets.QHBoxLayout()
        self.setup_widget = QtWidgets.QWidget()
        self.setLayout(main_layout)
        open_settings_button = QtWidgets.QPushButton(self.name)
        open_settings_button.pressed.connect(self.setup_widget.show)
        main_layout.addWidget(open_settings_button)
        self.setup_widget_startup()

    def setup_widget_startup(self):
        self.setup_widget.setWindowTitle(self.name)
        setting_layout = QtWidgets.QFormLayout()
        self.setup_widget.setLayout(setting_layout)
        motor = self.motor

        motor_enabled_checkbox = QtWidgets.QCheckBox()
        motor_enabled_changed = partial(motor.parameter_changed, "motor_enabled")
        motor_enabled_checkbox.setChecked(motor.parameter.motor_enabled)
        motor_enabled_checkbox.stateChanged.connect(motor_enabled_changed)
        setting_layout.addRow(f"Motor enabled", motor_enabled_checkbox)

        position_in_rotation_spinbox = QtWidgets.QDoubleSpinBox()
        position_in_rotation_spinbox.setValue(motor.parameter.position_in_rotation)
        position_in_rotation_changed = partial(motor.parameter_changed, "position_in_rotation")
        position_in_rotation_spinbox.valueChanged.connect(position_in_rotation_changed)
        setting_layout.addRow(f"Position {motor.parameter.position_unit}", position_in_rotation_spinbox)

        direction_spinbox = QtWidgets.QSpinBox()
        direction_spinbox.setSingleStep(2)
        direction_spinbox.setMinimum(-1)
        direction_spinbox.setMaximum(1)
        direction_spinbox.setValue(motor.parameter.direction)
        direction_changed = partial(motor.parameter_changed, "direction")
        direction_spinbox.valueChanged.connect(direction_changed)
        setting_layout.addRow("Direction", direction_spinbox)

        max_speed_spinbox = QtWidgets.QDoubleSpinBox()
        max_speed_spinbox.setValue(motor.get_max_speed())
        max_speed_spinbox.valueChanged.connect(motor.set_max_speed)
        max_speed_changed = partial(motor.parameter_changed, "max_speed")
        max_speed_spinbox.valueChanged.connect(max_speed_changed)
        setting_layout.addRow(f"Max speed {motor.parameter.speed_unit}", max_speed_spinbox)

    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        self.setup_widget.show()


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

        used_resolution = CameraResolutionOptions.half_resolution

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
                self.thread = VideoThread(camera=self.camera, camera_resolution=used_resolution)
                self.thread.change_pixmap_signal.connect(self.update_image)
                self.thread.start()
                self.video_widget = QtWidgets.QLabel()
                self.video_widget.setFixedSize(used_resolution.resolution[0], used_resolution.resolution[1])
                main_layout.addWidget(self.video_widget)
            else:
                self.camera.start_preview()
        main_layout.addWidget(camera_control_widget)
        main_layout.addLayout(self.motor_layout)

    def capture_image(self):
        date = datetime.datetime.now().strftime("%Y_%m_%d %H:%M:%S")
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