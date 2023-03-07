import enum
import serial
import sys
import shelve
import os
is_simulation = False
if sys.platform == "darwin":
    is_simulation = True

if is_simulation:
    from PyQt6 import QtCore
else:
    from PyQt5 import QtCore

class MotorState(enum.Enum):
    UNKNOWN = enum.auto
    HOMING = enum.auto
    READY = enum.auto
    MOVING = enum.auto
    ERROR = enum.auto
    SPEEDMODE = enum.auto

class MotionType:
    linear: str = "linear"
    rotary: str = "rotary"

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