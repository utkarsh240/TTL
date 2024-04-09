import math

gravity = -9.81

class Car:
    def _init_(self, car):
        self.is_player_controlled = True
        self.cg_height = 0.55
        self.inertia_scale = 1
        self.brake_power = 12000
        self.e_brake_power = 10000
        self.weight_transfer = 0.24
        self.max_steer_angle = 0.797
        self.corner_stiffness_front = 5.0
        self.corner_stiffness_rear = 5.2
        self.air_resistance = 2.5
        self.rolling_resistance = 8.0
        self.e_brake_grip_ratio_front = 0.9
        self.total_tire_grip_front = 2.5
        self.e_brake_grip_ratio_rear = 0.4
        self.total_tire_grip_rear = 2.5
        self.steer_speed = 2.5
        self.steer_adjust_speed = 1
        self.speed_steer_correction = 295
        self.speed_turning_stability = 11.8
        self.axle_distance_correction = 1.7
        self.trail_acceleration_threshold = 25
        self.e_brake_speed_threshold = 40
        self.axle_front_pos = (0, 0.86)
        self.tire_front_left_pos = (-0.64, 0.861)
        self.tire_front_right_pos = (0.64, 0.861)
        self.axle_rear_pos = (0, -0.865)
        self.tire_rear_left_pos = (-0.64, -0.865)
        self.tire_rear_right_pos = (0.64, -0.865)
        self.speed_factor = 0.065
        self.rotation_factor = 0.05
        self.mass = 1500
        self.linear_drag = 0.1
        self.angular_drag = 0.5

        self.heading_angle = 0
        self.absolute_velocity = 0
        self.angular_velocity = 0
        self.steer_direction = 0
        self.steer_angle = 0
        self.position = (0, 0)
        self.origin_position = (0, 0)
        self.rotation = 0
        self.direction = 0
        self.wheel_rotation = 0
        self.distance = 0
        self.velocity = (0, 0)
        self.acceleration = (0, 0)
        self.local_velocity = (0, 0)
        self.local_acceleration = (0, 0)
        self.center_of_gravity = (0, -0.231)

        self.inertia = 1
        self.wheel_base = 1
        self.track_width = 1

        self.steer = 0
        self.throttle = 0
        self.brake = 0
        self.e_brake = 0
        self.e_brake_trigger_speed = 0

        self.active_brake = 0
        self.active_throttle = 0

        self.front_wheel_drive = False
        self.rear_wheel_drive = True

        self.axle_front = None
        self.axle_rear = None
        self.engine = None

        self.car = car

        self.object_contact = False
        self.object_contact_obstacle = None
        self.object_contact_reverse_direction = 0

        self.last_current_time = None

        self.position = (self.car.position.z, self.car.position.x)
        self.origin_position = (self.car.position.z, self.car.position.x)

        self.axle_front = Axle(self, (self.car.wheel_front_left, self.car.wheel_front_right))
        self.axle_rear = Axle(self, (self.car.wheel_rear_left, self.car.wheel_rear_right))

        self.axle_front.position = self.axle_front_pos
        self.axle_front.tire_left.position = self.tire_front_left_pos
        self.axle_front.tire_right.position = self.tire_front_right_pos
        self.axle_rear.position = self.axle_rear_pos
        self.axle_rear.tire_left.position = self.tire_rear_left_pos
        self.axle_rear.tire_right.position = self.tire_rear_right_pos

        self.engine = Engine(self)

        self.axle_front.distance_to_cg = abs(self.center_of_gravity[1] - self.axle_front.position[1])
        self.axle_rear.distance_to_cg = abs(self.center_of_gravity[1] - self.axle_rear.position[1])
        self.axle_front.distance_to_cg *= self.axle_distance_correction
        self.axle_rear.distance_to_cg *= self.axle_distance_correction

        self.wheel_base = self.axle_front.distance_to_cg + self.axle_rear.distance_to_cg
        self.inertia = self.mass * self.inertia_scale

        self.heading_angle = self.rotation

        self.axle_front.setup(self.wheel_base)
        self.axle_rear.setup(self.wheel_base)

        self.track_width = abs(self.axle_rear.tire_left.position[0] - self.axle_rear.tire_right.position[0])

    def notify_contact(self, contact_direction, obstacle):
        if contact_direction == 1 and not self.object_contact and (self.object_contact_obstacle is None or obstacle != self.object_contact_obstacle):
            self.begin_contact(obstacle)
        if contact_direction == -1 and obstacle == self.object_contact_obstacle:
            self.end_contact()

    def begin_contact(self, obstacle):
        self.velocity = (self.velocity[0] / 5, self.velocity[1] / 5)
        self.object_contact_reverse_direction = -self.direction
        self.object_contact_obstacle = obstacle
        self.object_contact = True

    def end_contact(self):
        self.object_contact_reverse_direction = 0
        self.object_contact_obstacle = None
        self.object_contact = False

    def speed_kilometers_per_hour(self):
        return math.sqrt(self.velocity[0] ** 2 + self.velocity[1] ** 2) * 18 / 5

    def setup(self, four_wheel_drive):
        if four_wheel_drive:
            self.front_wheel_drive = True
            self.rear_wheel_drive = True
        else:
            self.front_wheel_drive = False
            self.rear_wheel_drive = True

    def update(self, steer, throttle, e_brake=0, dt=1/60):
        self.position = (self.car.position.z, self.car.position.x)
        self.rotation = self.car.euler_angles.y
        self.heading_angle = self.rotation

        sin_val = math.sin(self.heading_angle)
        cos_val = math.cos(self.heading_angle)

        # Get local velocity
        self.local_velocity = (cos_val * self.velocity[0] + sin_val * self.velocity[1],
                               cos_val * self.velocity[1] - sin_val * self.velocity[0])

        # Direction
        self.direction = 1 if self.local_velocity[0] >= 0 else -1

        # Prevent overlap => block
        blocked = 0.0
        if self.object_contact and self.object_contact_reverse_direction != 0:
            if self.direction != self.object_contact_reverse_direction:
                self.velocity = (0, 0)
                self.local_velocity = (0, 0)
            if math.copysign(1, throttle) == self.object_contact_reverse_direction:
                self.end_contact()
            else:
                blocked = 1.0

        # Weight transfer
        transfer_x = self.weight_transfer * self.local_acceleration[0] * self.cg_height / self.wheel_base
        transfer_y = self.weight_transfer * self.local_acceleration[1] * self.cg_height / self.track_width * 20

        # Weight on each axle
        weight_front = self.mass * (self.axle_front.weight_ratio * -gravity - transfer_x)
        weight_rear = self.mass * (self.axle_rear.weight_ratio * -gravity + transfer_x)

        # Weight on each tire
        self.axle_front.tire_left.active_weight = weight_front - transfer_y
        self.axle_front.tire_right.active_weight = weight_front + transfer_y
        self.axle_rear.tire_left.active_weight = weight_rear - transfer_y
        self.axle_rear.tire_right.active_weight = weight_rear + transfer_y

        # Velocity of each tire
        self.axle_front.tire_left.angular_velocity = self.axle_front.distance_to_cg * self.angular_velocity
        self.axle_front.tire_right.angular_velocity = self.axle_front.distance_to_cg * self.angular_velocity
        self.axle_rear.tire_left.angular_velocity = -self.axle_rear.distance_to_cg * self.angular_velocity
        self.axle_rear.tire_right.angular_velocity = -self.axle_rear.distance_to_cg * self.angular_velocity

        # Slip angle
        self.axle_front.slip_angle = math.atan2(self.local_velocity[1] + self.axle_front.angular_velocity(), abs(self.local_velocity[0])) - math.copysign(1, self.local_velocity[0]) * self.steer_angle
        self.axle_rear.slip_angle = math.atan2(self.local_velocity[1] + self.axle_rear.angular_velocity(), abs(self.local_velocity[0]))

        # Brake and Throttle power
        self.active_brake = min(self.brake * self.brake_power + self.e_brake * self.e_brake_power, self.brake_power)
        self.active_throttle = ((1 - blocked) * throttle * self.engine.torque()) * (self.engine.gear_ratio() * self.engine.effective_gear_ratio())

        # Torque of each tire (front wheel drive)
        if self.front_wheel_drive:
            self.axle_front.tire_left.torque = self.active_throttle / self.axle_front.tire_left.radius
            self.axle_front.tire_right.torque = self.active_throttle / self.axle_front.tire_right.radius

        # Torque of each tire (rear wheel drive)
        if self.rear_wheel_drive:
            self.axle_rear.tire_left.torque = self.active_throttle / self.axle_rear.tire_left.radius
            self.axle_rear.tire_right.torque = self.active_throttle / self.axle_rear.tire_right.radius

        # Grip and Friction of each tire
        self.axle_front.tire_left.grip = self.total_tire_grip_front * (1.0 - self.e_brake * (1.0 - self.e_brake_grip_ratio_front))
        self.axle_front.tire_right.grip = self.total_tire_grip_front * (1.0 - self.e_brake * (1.0 - self.e_brake_grip_ratio_front))
        self.axle_rear.tire_left.grip = self.total_tire_grip_rear * (1.0 - self.e_brake * (1.0 - self.e_brake_grip_ratio_rear))
        self.axle_rear.tire_right.grip = self.total_tire_grip_rear * (1.0 - self.e_brake * (1.0 - self.e_brake_grip_ratio_rear))

        self.axle_front.tire_left.friction_force = max(min(-self.corner_stiffness_front * self.axle_front.slip_angle, self.axle_front.tire_left.grip), -self.axle_front.tire_left.grip) * self.axle_front.tire_left.active_weight
        self.axle_front.tire_right.friction_force = max(min(-self.corner_stiffness_front * self.axle_front.slip_angle, self.axle_front.tire_right.grip), -self.axle_front.tire_right.grip) * self.axle_front.tire_right.active_weight
        self.axle_rear.tire_left.friction_force = max(min(-self.corner_stiffness_rear * self.axle_rear.slip_angle, self.axle_rear.tire_left.grip), -self.axle_rear.tire_left.grip) * self.axle_rear.tire_left.active_weight
        self.axle_rear.tire_right.friction_force = max(min(-self.corner_stiffness_rear * self.axle_rear.slip_angle, self.axle_rear.tire_right.grip), -self.axle_rear.tire_right.grip) * self.axle_rear.tire_right.active_weight

        # Forces
        torque = 0
        if self.front_wheel_drive and self.rear_wheel_drive:
            torque = (self.axle_front.torque() + self.axle_rear.torque()) / 2
        elif self.front_wheel_drive:
            torque = self.axle_front.torque()
        elif self.rear_wheel_drive:
            torque = self.axle_rear.torque()

        traction_force_x = torque - self.active_brake * math.copysign(1, self.local_velocity[0])
        traction_force_y = 0

        drag_force_x = -self.rolling_resistance * self.local_velocity[0] - self.air_resistance * self.local_velocity[0] * abs(self.local_velocity[0])
        drag_force_y = -self.rolling_resistance * self.local_velocity[1] - self.air_resistance * self.local_velocity[1] * abs(self.local_velocity[1])

        total_force_x = drag_force_x + traction_force_x
        total_force_y = drag_force_y + traction_force_y + math.cos(self.steer_angle) * self.axle_front.friction_force() + self.axle_rear.friction_force()

        # Adjust Y force so it levels out the car heading at high speeds
        if self.absolute_velocity > 10:
            total_force_y *= (self.absolute_velocity + 1) / (21 - self.speed_turning_stability)

        # If we are not pressing gas, add artificial drag - helps with simulation stability
        if throttle == 0:
            self.velocity = (
                self.velocity[0] * (1 - 0.005),
                self.velocity[1] * (1 - 0.005)
            )

        # Acceleration
        self.local_acceleration = (
            total_force_x / self.mass,
            total_force_y / self.mass
        )

        self.acceleration = (
            cos_val * self.local_acceleration[0] - sin_val * self.local_acceleration[1],
            sin_val * self.local_acceleration[0] + cos_val * self.local_acceleration[1]
        )

        # Velocity and speed
        self.velocity = (
            self.velocity[0] + self.acceleration[0] * dt,
            self.velocity[1] + self.acceleration[1] * dt
        )

        self.absolute_velocity = math.sqrt(self.velocity[0] ** 2 + self.velocity[1] ** 2)

        # Angular torque of car
        angular_torque = (self.axle_front.friction_force() * self.axle_front.distance_to_cg) - (self.axle_rear.friction_force() * self.axle_rear.distance_to_cg)
        # Car will drift away at low speeds
        if self.absolute_velocity < 0.5 and self.active_throttle == 0:
            self.local_acceleration = (0, 0)
            self.absolute_velocity = 0
            self.velocity = (0, 0)
            angular_torque = 0
            self.angular_velocity = 0
            self.acceleration = (0, 0)
        angular_acceleration = angular_torque / self.inertia

        # Update
        self.angular_velocity += angular_acceleration * dt
        ...

    def update_control(self, steer, throttle, e_brake=0, dt=1/60):
        # Update control variables
        if self.is_player_controlled:
            self.steer = steer
            self.throttle = throttle
            if self.e_brake == 0 and e_brake > 0:
                self.e_brake_trigger_speed = self.speed_kilometers_per_hour()
            elif e_brake == 0:
                self.e_brake_trigger_speed = 0
            self.e_brake = e_brake

            # Apply filters to our steer direction
            self.steer_direction = self.smooth_steering(steer, dt)
            self.steer_direction = self.speed_adjusted_steering(self.steer_direction)

            # Calculate the current angle the tires are pointing
            self.steer_angle = self.steer_direction * self.max_steer_angle

            # Set front axle tires rotation
            self.axle_front.tire_left.rotation = self.steer_angle
            self.axle_front.tire_right.rotation = self.steer_angle

        # Calculate weight center of four tires
        # This is just to draw that red dot over the car to indicate what tires have the most weight
        pos = (0, 0)
        if math.sqrt(self.local_acceleration[0] ** 2 + self.local_acceleration[1] ** 2) > 1:
            wfl = max(0, (self.axle_front.tire_left.active_weight - self.axle_front.tire_left.resting_weight))
            wfr = max(0, (self.axle_front.tire_right.active_weight - self.axle_front.tire_right.resting_weight))
            wrl = max(0, (self.axle_rear.tire_left.active_weight - self.axle_rear.tire_left.resting_weight))
            wrr = max(0, (self.axle_rear.tire_right.active_weight - self.axle_rear.tire_right.resting_weight))

            pos = (
                self.axle_front.tire_left.position[0] * wfl + self.axle_front.tire_right.position[0] * wfr +
                self.axle_rear.tire_left.position[0] * wrl + self.axle_rear.tire_right.position[0] * wrr,
                self.axle_front.tire_left.position[1] * wfl + self.axle_front.tire_right.position[1] * wfr +
                self.axle_rear.tire_left.position[1] * wrl + self.axle_rear.tire_right.position[1] * wrr
            )

            weight_total = wfl + wfr + wrl + wrr
        ...

    def smooth_steering(self, steer_input, dt):
        # Smooth steering input
        self.steer_input = steer_input
        self.steer_input_smoothed = self.steer_input_smoothed 
