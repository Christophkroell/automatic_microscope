import sys
from functools import partial
is_simulation = False
if sys.platform == "darwin":
    is_simulation = True

if is_simulation:
    from PyQt6 import QtWidgets, QtGui
else:
    from PyQt5 import QtWidgets, QtGui
from Hardware.MotorHW import Motor

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
