import pty

class Serial():
    def __init__(self, serial_path, baud):
        self.serial_path = serial_path
        self.buad = baud

    def read_until(self):
        pass
