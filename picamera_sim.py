
class PiCamera:
    framerate: int = 10
    brightness: int = 50
    contrast: int = 0
    saturation: int = 0
    sharpness: int = 0
    iso: int = 60
    shutter_speed: int = 100000
    exposure_compensation: int = 0
    exposure_mode: str = "off"
    awb_mode: str = "off"
    awb_gains: tuple = (1, 1)

    def start_preview(self):
        pass

    def capture(self, file_path):
        print(f"save file to: {file_path}")

    def __setattr__(self, key, value):
        print(f"camer: {key} : {value}")

class PiRGBArray:
    pass