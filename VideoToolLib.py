from PyQt5 import QtWidgets, QtCore, QtGui


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
        self.exposure_time_ms_spinbox = QtWidgets.QSpinBox()
        self.exposure_time_ms_spinbox.setRange(0, 600)
        self.exposure_time_ms_spinbox.setValue(self.camera.shutter_speed)
        self.exposure_time_ms_spinbox.valueChanged.connect(self.set_exposure_time_ms)
        layout.addRow(self.shutter_speed_label, self.exposure_time_ms_spinbox)

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

    def set_exposure_time_ms(self, value):
        self.camera.shutter_speed = value

    def setExposureCompensation(self, value):
        self.camera.exposure_compensation = value

    def setExposureMode(self, value):
        self.camera.exposure_mode = value