
class port:
    def __init__(self):
        pass

    class A:
        def __init__(self):
            self.device = None
            self.config = None
            self.ignore_conf = 0
            

    class B:
        def __init__(self):
            self.device = None
            self.config = None
            self.ignore_conf = 0

    class C:
        def __init__(self):
            self.device = None
            self.config = None
            self.ignore_conf = 0

    class D:
        def __init__(self):
            self.device = None
            self.config = None 
            self.ignore_conf = 0           

    class E:
        def __init__(self):
            self.device = None
            self.config = None 
            self.ignore_conf = 0         

class hub_class:
    def __init__(self):
        self.hub_program_cycle = 100
        self.enable_imu = False
        self.imu_mode = ""
        self.enable_battery_management = False
        self.enable_button = False
        self.enable_speaker = False
        self.speaker_volume = 60
        self.opening = True
        self.ignore_conf = 0


portA = port.A()
portB = port.B()
portC = port.C()
portD = port.D()
portE = port.E()
hub = hub_class()


class device_class:
    def __init__(self):
        self.qos = "best-effort"
        self.port = ""

    
class motor_class(device_class):
    def __init__(self):
        super().__init__()
        self.wise = "clock"
        self.run_mode = "set-power"

class color_sensor_class(device_class):
    def __init__(self):
        super().__init__()
        self.enable_lights = False
        self.light_qos = "best-effort"

class ultrasonic_sensor_class(device_class):
    def __init__(self):
        super().__init__()
        self.enable_lights = False
        self.light_qos = "best-effort"

class force_sensor_class(device_class):
    def __init__(self):
        super().__init__()
