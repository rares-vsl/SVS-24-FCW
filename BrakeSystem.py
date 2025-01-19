import threading
import time

import carla


class BrakeSystem(object):
    def __init__(self):
        self.stop_flag = False

    def delayer_stop(self):
        time.sleep(2)
        self.stop_flag = False

    def is_active(self):
        return self.stop_flag
    
    
    def stop_vehicle(self, vehicle, constant_velocity_enabled):
        control = carla.VehicleControl()
        
        self.stop_flag = True
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = True

        if constant_velocity_enabled:
            vehicle.disable_constant_velocity()
            
        vehicle.apply_control(control)
        threading.Thread(target=self.delayer_stop).start()