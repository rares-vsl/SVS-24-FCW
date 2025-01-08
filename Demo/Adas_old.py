import carla, math, paho.mqtt.client as mqtt
from enum import Enum

# Enum state del sistema adas
class Fcw_state(Enum):
    IDLE = 4
    WARNING = 3
    ACTION = 2
    ESCAPE = 1

class Forward_collision_warning_mqtt:
        def __init__(self,
                    world,
                    attached_vehicle,
                    get_asphalt_friction_coefficient,
                    action_listener, 
                    debug = True,
                    visual_debug_pixel_life_time = 0.25,  
                    mqtt_broker = 'broker.emqx.io',
                    mqtt_port = 1883,
                    mqtt_topic = "carla/fcw_state",
                    min_fcw_state = Fcw_state.IDLE,
                    vehicle_vertical_dimension = 0.75,
                    vehicle_horizontal_dimension = 0.9,
                    min_ttc = 0.5,
                    average_reaction_time = 2.5,
                    velocity_threshold = 2.77,
                    max_radiant_steer_angle = 1, # circa 57 gradi
                    steer_tollerance = 0.02,
                    radar_range = 285,
                    radar_pitch = 5          
        ):
            
            # Inizializzazione parametri
            self.__world = world
            self.__attached_vehicle = attached_vehicle
            self.__get_asphalt_friction_coefficient = get_asphalt_friction_coefficient
            self.__action_listener = action_listener
            self.__debug = debug
            self.__visual_debug_pixel_life_time = visual_debug_pixel_life_time
            self.__mqtt_topic = mqtt_topic
            self.__min_fcw_state = min_fcw_state
            self.__vehicle_vertical_dimension = vehicle_vertical_dimension
            self.__vehicle_horizontal_dimension = vehicle_horizontal_dimension
            self.__min_ttc = min_ttc
            self.__average_reaction_time = average_reaction_time
            self.__velocity_threshold = velocity_threshold
            self.__max_radiant_steer_angle = max_radiant_steer_angle
            self.__steer_tollerance = steer_tollerance
            self.__radar_range = radar_range
            self.__radar_pitch = radar_pitch

            # Creazione client mqtt
            self.__mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            self.__mqttc.connect(mqtt_broker, mqtt_port)

            # Creazione del radar
            rad_bp = world.get_blueprint_library().find('sensor.other.radar')
            rad_bp.set_attribute('horizontal_fov', str(125))
            rad_bp.set_attribute('vertical_fov', str(115 + self.__radar_pitch))
            rad_bp.set_attribute('range', str(self.__radar_range))
            rad_bp.set_attribute('points_per_second', str(2000))
            rad_location = carla.Location(x=2.25, z=0.9)
            rad_rotation = carla.Rotation(pitch = self.__radar_pitch)
            rad_transform = carla.Transform(rad_location,rad_rotation)

            # Aggancio al veicolo
            self.__radar = world.spawn_actor(rad_bp,rad_transform,attach_to=self.__attached_vehicle, attachment_type=carla.AttachmentType.Rigid)

            # Avvio client mqtt
            self.__mqttc.loop_start()

            # Settaggio del listener
            self.__radar.listen(lambda radar_data: self.__forward_collision_callback(radar_data))

        # Debug visivo
        def __radar_visual_debug(self, detection, radar_data, red, green, blue):
            if self.__debug:
                current_rot = radar_data.transform.rotation
                azi = math.degrees(detection.azimuth)
                alt = math.degrees(detection.altitude)
                
                fw_vec = carla.Vector3D(x=detection.depth - 0.25)
                carla.Transform(
                    carla.Location(),
                    carla.Rotation(
                        pitch=current_rot.pitch + alt,
                        yaw=current_rot.yaw + azi,
                        roll=current_rot.roll)).transform(fw_vec)
                
                self.__world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=self.__visual_debug_pixel_life_time,
                    persistent_lines=False,
                    color=carla.Color(red, green, blue))
                
        # Algoritmo
        def __forward_collision_callback(self, radar_data):
            asphalt_friction_coefficient = self.__get_asphalt_friction_coefficient()
            asphalt_friction_deceleration = 9.81 * asphalt_friction_coefficient
            control = self.__attached_vehicle.get_control()
            if -self.__steer_tollerance < control.steer < self.__steer_tollerance:
                radiant_steer_angle = control.steer * self.__max_radiant_steer_angle
            else:
                radiant_steer_angle = 0
            for detection in radar_data:
                vehicle_velocity = self.__get_attached_vehicle_velocity()
                if vehicle_velocity > self.__velocity_threshold:
                    if detection.velocity < 0 and self.__check_horizontal_collision(detection.azimuth, detection.depth, radiant_steer_angle) and self.__check_vertical_collision(detection.altitude, detection.depth):
                        projected_depth = self.__get_projected_depth(detection.azimuth, detection.altitude, detection.depth, radiant_steer_angle)
                        max_depth = self.__get_max_depth(detection.azimuth, radiant_steer_angle)
                        if projected_depth <= max_depth:
                            projected_velocity = self.__get_projected_velocity(detection.azimuth, detection.altitude, detection.velocity, radiant_steer_angle) 
                            breaking_distance = self.__get_breaking_distance(projected_velocity, asphalt_friction_deceleration)
                            reacting_distance = max(0, projected_depth - breaking_distance)
                            ttc = reacting_distance / projected_velocity
                            fcw_state = Fcw_state.IDLE
                            if ttc < self.__min_ttc:  
                                if vehicle_velocity < projected_velocity - self.__velocity_threshold:
                                    fcw_state = Fcw_state.ESCAPE
                                    self.__radar_visual_debug(detection, radar_data, 0,0,1)
                                else:
                                    fcw_state = Fcw_state.ACTION
                                self.__radar_visual_debug(detection, radar_data, 1,0,0)
                            elif ttc < self.__min_ttc + self.__average_reaction_time:
                                fcw_state = Fcw_state.WARNING
                                self.__radar_visual_debug(detection, radar_data, 1,1,0)
                            else:
                                self.__radar_visual_debug(detection, radar_data, 0,1,0)
                            if fcw_state.value < self.__min_fcw_state.value:
                                self.__min_fcw_state = fcw_state    
                                self.__publish_message()
                                if fcw_state == Fcw_state.ACTION:
                                    self.__action_listener()
                else:
                    self.__min_fcw_state = Fcw_state.IDLE


        # Controllo dei segno
        def __are_sign_equals(self, a, b):
            return (a > 0 and b > 0) or (a < 0 and b < 0)
        
        # Convertitori
        def __get_radiant_radar_pitch(self):
            return math.radians(self.__radar_pitch)

        def __check_horizontal_collision(self, azimuth, depth, radiant_steer_angle):
            if self.__are_sign_equals(azimuth, radiant_steer_angle):
                return True
            return abs(depth * math.sin(azimuth)) < self.__vehicle_horizontal_dimension

        def __check_vertical_collision(self, altitude, depth):
            return abs(depth * math.sin(altitude + self.__get_radiant_radar_pitch())) < self.__vehicle_vertical_dimension 

        def __get_projected_velocity(self, azimuth, altitude, velocity, radiant_steer_angle):
            return abs(math.cos(azimuth - radiant_steer_angle) * math.cos(altitude + self.__get_radiant_radar_pitch()) * velocity)

        def __get_projected_depth(self, azimuth, altitude, depth, radiant_steer_angle):
            return abs(math.cos(azimuth - radiant_steer_angle) * math.cos(altitude + self.__get_radiant_radar_pitch()) * depth)

        def __get_breaking_distance(self, projected_velocity, asphalt_friction_deceleration):
            return (1/2) * (projected_velocity**2) / asphalt_friction_deceleration

        def __get_attached_vehicle_velocity(self):
            velocity_vector = self.__attached_vehicle.get_velocity()
            velocity = (velocity_vector.x**2 + velocity_vector.y**2 + velocity_vector.z**2)**0.5
            if self.__attached_vehicle.get_control().reverse:
                velocity = -velocity
            return velocity

        def __get_max_depth(self, azimuth, radiant_steer_angle):
            if radiant_steer_angle == 0:
                return self.__radar_range
            if self.__are_sign_equals(azimuth, radiant_steer_angle):
                stering_range = abs(self.__vehicle_horizontal_dimension / math.tan(radiant_steer_angle))
            else:
                stering_range = abs((self.__vehicle_horizontal_dimension / 2) / math.tan(radiant_steer_angle))
            return min(stering_range, self.__radar_range)

        # Invio messaggi Mqtt
        def __publish_message(self):
            status = self.__mqttc.publish(self.__mqtt_topic, self.__min_fcw_state.name)
            if status[0] == 0:
                print(f"Send `{self.__min_fcw_state.name}` to topic `{self.__mqtt_topic}`")
            else:
                print(f"Failed to send message to topic {self.__mqtt_topic}")

        # Destroy
        def destroy(self):
            self.__mqttc.loop_stop()
            self.__mqttc.disconnect()
            self.__radar.destroy()
            