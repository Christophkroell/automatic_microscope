import sys
is_simulation = False
if sys.platform == "darwin":
    is_simulation = True

if is_simulation:
    from picamera_sim import PiCamera
    from picamera_sim import PiRGBArray
    from PyQt6 import QtWidgets, QtCore, QtGui
else:
    import cv2
    from picamera import PiCamera
    from picamera.array import PiRGBArray
    from PyQt5 import QtWidgets, QtCore, QtGui


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