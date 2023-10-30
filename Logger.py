""" Simple custom logger
        * 0: INFO, WARNING, ERROR
        * 1: WARNING, ERROR
        * 2: ERROR
"""
class Logger:
    BLUE = '\033[94m'
    ORANGE = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    END = '\033[0m'

    def __init__(self, log_level):
        self.log_level = log_level
    
    def INFO(self, msg):
        if self.log_level <= 0:
            log = f"{self.BLUE}{self.BOLD}[INFO] {self.END}{msg}"
            print(log)

    def WARNING(self, msg):
        if self.log_level <= 1:
            log = f"{self.ORANGE}{self.BOLD}[WARNING] {self.END}{msg}"
            print(log)

    def ERROR(self, msg):
        if self.log_level <= 2:
            log = f"{self.RED}{self.BOLD}[ERROR] {self.END}{msg}"
            print(log)

