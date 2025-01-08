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
                    vehicle_half_vertical_dimension = 0.75,
                    vehicle_half_horizontal_dimension = 0.9,
                    vehicle_wheelbase = 2.8,
                    min_ttc = 0.5,
                    average_reaction_time = 2.5,
                    velocity_th = 2.77,
                    escape_ratio_th = 0.5,
                    max_radiant_steer_angle = 1.22, # circa 70 gradi
                    steer_tollerance = 0.02,
                    radar_range = 250,
                    climb_inconsistencies_th = 5,
                    max_radiant_slope = 0.2,
                    detected_point_th = 20   
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
            self.__vehicle_half_vertical_dimension = vehicle_half_vertical_dimension
            self.__vehicle_half_horizontal_dimension = vehicle_half_horizontal_dimension
            self.__vehicle_wheelbase = vehicle_wheelbase
            self.__min_ttc = min_ttc
            self.__average_reaction_time = average_reaction_time
            self.__velocity_th = velocity_th
            self.__escape_ratio_th = escape_ratio_th
            self.__max_radiant_steer_angle = max_radiant_steer_angle
            self.__steer_tollerance = steer_tollerance
            self.__radar_range = radar_range
            self.__climb_inconsistencies_th = climb_inconsistencies_th
            self.__detected_point_th = detected_point_th
            self.__max_radiant_slope = max_radiant_slope

            # Creazione client mqtt
            self.__mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            self.__mqttc.connect(mqtt_broker, mqtt_port)

            # Creazione del radar
            rad_bp = world.get_blueprint_library().find('sensor.other.radar')
            rad_bp.set_attribute('horizontal_fov', str(25))
            rad_bp.set_attribute('vertical_fov', str(25))
            rad_bp.set_attribute('range', str(self.__radar_range))
            rad_bp.set_attribute('points_per_second', str(50000))
            rad_bp.set_attribute('sensor_tick', str(0.01))
            rad_location = carla.Location(x=2.25, z=0.9)
            rad_rotation = carla.Rotation()
            rad_transform = carla.Transform(rad_location,rad_rotation)

            # Aggancio al veicolo
            self.__radar = world.spawn_actor(rad_bp,rad_transform,attach_to=self.__attached_vehicle, attachment_type=carla.AttachmentType.Rigid)

            # Avvio client mqtt
            self.__mqttc.loop_start()

            # Settaggio del listener
            self.__radar.listen(lambda radar_data: self.__forward_collision_callback(radar_data))
                
        # Algoritmo
        def __forward_collision_callback(self, radar_data):
            asphalt_friction_coefficient = self.__get_asphalt_friction_coefficient()
            asphalt_friction_deceleration = 9.81 * asphalt_friction_coefficient
            control = self.__attached_vehicle.get_control()
            if -self.__steer_tollerance < control.steer < self.__steer_tollerance:
                radiant_steer_angle = control.steer * self.__max_radiant_steer_angle
            else:
                radiant_steer_angle = 0
            detected_escape_list = []
            detected_action_list = []
            detected_warning_list = []
            detected_idle_list = []
            vehicle_velocity = self.__get_attached_vehicle_velocity()
            if vehicle_velocity > self.__velocity_th:
                for detection in radar_data:
                    if detection.velocity < 0 and self.__check_horizontal_collision(detection.azimuth, detection.depth, radiant_steer_angle) and self.__check_vertical_collision(detection.altitude, detection.depth):
                        projected_depth = self.__get_projected_depth(detection.azimuth, detection.altitude, detection.depth, radiant_steer_angle)
                        max_depth = self.__get_max_depth(detection.azimuth, radiant_steer_angle)
                        if projected_depth <= max_depth:
                            projected_velocity = self.__get_projected_velocity(detection.azimuth, detection.altitude, detection.velocity, radiant_steer_angle) 
                            breaking_distance = self.__get_breaking_distance(projected_velocity, asphalt_friction_deceleration)
                            reacting_distance = max(0, projected_depth - breaking_distance)
                            ttc = reacting_distance / projected_velocity
                            if ttc < self.__min_ttc:  
                                if abs(vehicle_velocity) < projected_velocity * self.__escape_ratio_th:
                                    detected_escape_list.append((detection, projected_depth))
                                else:
                                    detected_action_list.append((detection, projected_depth))
                            elif ttc < self.__min_ttc + self.__average_reaction_time:
                                detected_warning_list.append((detection, projected_depth))
                            else:
                                detected_idle_list.append((detection, projected_depth))
                self.__analize_detection(detected_escape_list, radar_data, Fcw_state.ESCAPE, 0, 0, 1) 
                self.__analize_detection(detected_action_list, radar_data, Fcw_state.ACTION, 1, 0, 0, self.__action_listener)
                self.__analize_detection(detected_warning_list, radar_data, Fcw_state.WARNING, 1, 1, 0)
                self.__analize_detection(detected_idle_list, radar_data, Fcw_state.IDLE, 0, 1, 0)  

            else:
                self.__min_fcw_state = Fcw_state.IDLE

        # Support 
        def __analize_detection(self, detected_point_list, radar_data, fwd_state, red, green, blue, listener = lambda: None):
            if len(detected_point_list) > self.__detected_point_th:
                if self.__check_climb(detected_point_list):
                    red, green, blue = 1,1,1
                elif fwd_state.value < self.__min_fcw_state.value:
                    self.__min_fcw_state = fwd_state
                    if fwd_state != Fcw_state.IDLE:
                        self.__publish_message(fwd_state.name)
                    listener()
                for detection_plus_projected_distance in detected_point_list:
                    self.__radar_visual_debug(detection_plus_projected_distance[0], radar_data, red, green, blue)

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

        # Controllori
        def __are_sign_equals(self, a, b):
            return (a > 0 and b > 0) or (a < 0 and b < 0)

        def __check_horizontal_collision(self, azimuth, depth, radiant_steer_angle):
            return abs(depth * math.sin(azimuth - radiant_steer_angle)) < self.__vehicle_half_horizontal_dimension

        def __check_vertical_collision(self, altitude, depth):
            return abs(depth * math.sin(altitude)) < self.__vehicle_half_vertical_dimension 

        def __check_climb(self, detections_plus_projected_distance):
            if len(detections_plus_projected_distance) > self.__climb_inconsistencies_th + 2:
                climb_inconsistencies_counter = 0
                for i in range(2, len(detections_plus_projected_distance)):
                    if not self.__check_radiant_slope_validity(detections_plus_projected_distance[i], detections_plus_projected_distance[i-1]):
                        if not self.__check_radiant_slope_validity(detections_plus_projected_distance[i], detections_plus_projected_distance[i-2]):
                            climb_inconsistencies_counter +=1
                            if climb_inconsistencies_counter >  self.__climb_inconsistencies_th:
                                return False
                        else:
                           climb_inconsistencies_counter = 0
                    else: 
                       climb_inconsistencies_counter = 0  
                return True
            return False
        
        def __check_radiant_slope_validity(self, detection_plus_projected_distance1, detection_plus_projected_distance2):
            cathetus = detection_plus_projected_distance1[1] - detection_plus_projected_distance2[1]
            if cathetus < 0:
                return False  
            point1 = detection_plus_projected_distance1[0]
            altitude_depth1 = abs(math.cos(point1.azimuth) * point1.depth)
            point2 = detection_plus_projected_distance2[0]
            altitude_depth2 = abs(math.cos(point2.azimuth) * point2.depth)
            if altitude_depth2 > altitude_depth1:
                return False 
            included_altitude_angle = abs(point1.altitude - point2.altitude)
            hypotenuse = math.sqrt(altitude_depth1**2 + altitude_depth2**2 - 2 * altitude_depth1 * altitude_depth2 * math.cos(included_altitude_angle))
            if cathetus > hypotenuse:
                return False
            return math.acos(cathetus / hypotenuse) < self.__max_radiant_slope
        
        # Convertitori
        def __get_projected_velocity(self, azimuth, altitude, velocity, radiant_steer_angle):
            return abs(math.cos(azimuth - radiant_steer_angle) * math.cos(altitude) * velocity)

        def __get_projected_depth(self, azimuth, altitude, depth, radiant_steer_angle):
            return abs(math.cos(azimuth - radiant_steer_angle) * math.cos(altitude) * depth)

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
            radius = abs(self.__vehicle_wheelbase / math.tan(radiant_steer_angle))
            if self.__are_sign_equals(azimuth, radiant_steer_angle):
                cathetus = radius - 2 * self.__vehicle_half_horizontal_dimension
            else:
                cathetus = radius - self.__vehicle_half_horizontal_dimension
            stering_range = math.sqrt(radius**2 - cathetus**2)
            return min(stering_range, self.__radar_range)

        # Invio messaggi Mqtt
        def __publish_message(self, message):
            status = self.__mqttc.publish(self.__mqtt_topic, message)
            if status[0] == 0:
                print(f"Send `{message}` to topic `{self.__mqtt_topic}`")
            else:
                print(f"Failed to send message to topic {self.__mqtt_topic}")

        # Destroy
        def destroy(self):
            self.__mqttc.loop_stop()
            self.__mqttc.disconnect()
            self.__radar.destroy()
            