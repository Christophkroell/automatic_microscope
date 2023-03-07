
class CameraResolution:
    sensor_mode: int
    resolution: tuple
    frame_rates: tuple

    def __init__(self, sensor_mode, resolution, frame_rates):
        self.sensor_mode = sensor_mode
        self.resolution = resolution
        self.frame_rates = frame_rates

class CameraResolutionOptions:
    max_resolution = CameraResolution(sensor_mode=2, resolution=(2464, 2464), frame_rates=(0.5, 15))
    half_resolution = CameraResolution(sensor_mode=4, resolution=(1640, 1232), frame_rates=(0.5, 40))
    low_resolution = CameraResolution(sensor_mode=7, resolution=(640, 480), frame_rates=(40, 90))

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