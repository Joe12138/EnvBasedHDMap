import numpy as np
import utils
from vehicle.behavior import IDMVehicle
from vehicle.control import MDPVehicle
from vehicle.planner import planner


class InteractionVehicle(IDMVehicle):
    """
    Use Interaction human driving trajectories.
    """
    # Longitudinal policy parameters
    ACC_MAX = 5.0  # [m/s2]  """Maximum acceleration."""
    COMFORT_ACC_MAX = 3.0  # [m/s2]  """Desired maximum acceleration."""
    COMFORT_ACC_MIN = -3.0  # [m/s2] """Desired maximum deceleration."""
    DISTANCE_WANTED = 1.0  # [m] """Desired jam distance to the front vehicle."""
    TIME_WANTED = 0.5  # [s]  """Desired time gap to the front vehicle."""
    DELTA = 4.0  # [] """Exponent of the velocity term."""

    # Lateral policy parameters [MOBIL]
    POLITENESS = 0.1  # in [0, 1]
    LANE_CHANGE_MIN_ACC_GAIN = 0.2  # [m/s2]
    LANE_CHANGE_MAX_BRAKING_IMPOSED = 2.0  # [m/s2]
    LANE_CHANGE_DELAY = 1.0  # [s]

    def __init__(self, road, position, heading=0, velocity=0, target_lane_index=None, target_velocity=None, route=None,
                 enable_lane_change=False, timer=None, vehicle_id=None, v_length=None, v_width=None,
                 interaction_traj=None):
        super(InteractionVehicle, self).__init__(road, position, heading, velocity, target_lane_index, target_velocity,
                                                 route, enable_lane_change, timer)
        self.interaction_traj = interaction_traj
        self.traj = np.array(self.position)
        self.vehicle_id = vehicle_id
        self.sim_steps = 0
        self.overtaken = False
        self.appear = True if self.position[0] != 0 else False  # this relates to the data process method
        self.velocity_history = []
        self.heading_history = []
        self.crash_history = []
        self.overtaken_history = []

        # vehicle length [m]
        self.LENGTH = v_length
        self.WIDTH = v_width

    @classmethod
    def create(cls, road, vehicle_id, position, v_length, v_width, interaction_traj, heading=0, velocity=15):
        """
        Create a new Interaction vehicle.

        :param road: the road where the vehicle is driving
        :param vehicle_id: Interaction vehicle ID
        :param position: the position where the vehicle start on the road
        :param v_length: vehicle length
        :param v_width: vehicle width
        :param interaction_traj: interaction trajectory
        :param heading: initial heading
        :param velocity: initial velocity in [m/s]
        :return: A vehicle with Interaction position and velocity
        """

        v = cls(road, position, heading, velocity, vehicle_id=vehicle_id, v_length=v_length, v_width=v_width,
                interaction_traj=interaction_traj)

        return v

    def act(self, action=None):
        """
        Execute an action when Interaction vehicle is overridden.
        :return:
        """
        if not self.overtaken:
            return

        if self.crashed:
            return

        action = {}
        front_vehicle, rear_vehicle = self.road.neighbour_vehicles(self)

        # Lateral: MOBIL
        self.follow_road()
        if self.enable_lane_change:
            self.change_lane_policy()

        action["steering"] = self.steering_control(self.target_lane_index)

        # Longitudinal: IDM
        action["acceleration"] = self.acceleration(ego_vehicle=self, front_vehicle=front_vehicle,
                                                   rear_vehicle=rear_vehicle)
        action["acceleration"] = np.clip(action["acceleration"], -self.ACC_MAX, self.ACC_MAX)

        # THIS NEED TO BE THOUGHT AGAIN
        print("Lan Wenxing")
        self.check_collision()

        self.action = action

    def step(self, dt):
        """
        Update the state of an Interaction vehicle.

        If the front vehicle is too close, use IDM model to override the Interaction vehicle.
        :param dt:
        :return:
        """
        self.appear = True if self.interaction_traj[self.sim_steps][0] != 0 else False
        self.timer += dt
        self.sim_steps += 1

        self.heading_history.append(self.heading)
        self.velocity_history.append(self.velocity)
        self.crash_history.append(self.crashed)
        self.overtaken_history.append(self.overtaken)

        # Check if the vehicle need to overtake
        front_vehicle, rear_vehicle = self.road.neighbour_vehicles(self)
        if front_vehicle is not None and isinstance(front_vehicle, InteractionVehicle) and front_vehicle.overtaken:
            gap = self.road.dist_between_vehicles_on_same_lane(self, front_vehicle)
            desired_gap = self.desired_gap(self, front_vehicle)
        elif front_vehicle is not None and (isinstance(front_vehicle, HumanLikeVehicle) or isinstance(front_vehicle,
                                                                                                      MDPVehicle)):
            gap = self.road.dist_between_vehicles_on_same_lane(self, front_vehicle)
            desired_gap = self.desired_gap(self, front_vehicle)
        else:
            gap = 100
            desired_gap = 50

        if gap >= desired_gap and not self.overtaken:
            self.position = self.interaction_traj[self.sim_steps][:2]

            self.heading = self.interaction_traj[self.sim_steps][4]

            # This is only suitable to the lateral road
            self.velocity = self.interaction_traj[self.sim_steps][2]
            self.target_velocity = self.velocity
            self.lane_index = self.road.network.get_closest_lane_index(self.position)
            self.lane = self.road.network.get_lane(self.lane_index)
        elif int(self.interaction_traj[self.sim_steps][3][0]) == -1 and self.overtaken:
            self.position = self.interaction_traj[self.sim_steps][:2]
            self.velocity = self.interaction_traj[self.sim_steps][2]
        else:
            self.overtaken = True

            target_lane = self.interaction_traj[self.sim_steps][3]
            self.target_lane_index = (target_lane[1], target_lane[2], target_lane[3])

            super(InteractionVehicle, self).step(dt)

        self.traj = np.append(self.traj, self.position, axis=0)

    def check_collision(self, other):
        """
        Check for collision with another vehicle.

        :param other: the other vehicle
        :return:
        """
        if not self.COLLISIONS_ENABLED or not other.COLLISIONS_ENABLED or self.crashed or other is self:
            return

        # Fast spherical pre-check
        if np.linalg.norm(np.array(other.position) - np.array(self.position)) > self.LENGTH:
            return

        # if both vehicles are Interaction vehicles and have not been overridden
        if isinstance(self, InteractionVehicle) and not self.overtaken and isinstance(other,
                                                                                      InteractionVehicle) and not other.overtaken:
            return

            # Accurate rectangular check
        if utils.rotated_rectangles_intersect((self.position, 0.9 * self.LENGTH, 0.9 * self.WIDTH, self.heading),
                                              (other.position, 0.9 * other.LENGTH, 0.9 * other.WIDTH,
                                               other.heading)) and self.appear:
            self.velocity = other.velocity = min([self.velocity, other.velocity], key=abs)
            self.crashed = other.crashed = True


class HumanLikeVehicle(IDMVehicle):
    """
    Create a human-like (IRL) driving agent.
    """
    TAU_A = 0.1  # [s]
    TAU_DS = 0.1  # [s]
    PURSUIT_TAU = 1.5 * TAU_DS  # [s]
    KP_A = 1 / TAU_A
    KP_HEADING = 1 / TAU_DS
    KP_LATERAL = 1 / 0.1  # [1/s]
    MAX_STEERING_ANGLE = np.pi / 3  # [rad]
    MAX_VELOCITY = 30  # [m/s]

    def __init__(self, road, position,
                 heading=0,
                 velocity=0,
                 acc=0,
                 target_lane_index=None,
                 target_velocity=15,  # Speed reference
                 route=None,
                 timer=None,
                 vehicle_id=None, v_length=None, v_width=None, interaction_traj=None, human=False, IDM=False):
        super(HumanLikeVehicle, self).__init__(road, position, heading, velocity, target_lane_index, target_velocity,
                                               route, timer)

        self.interaction_traj = interaction_traj
        self.traj = np.array(self.position)
        self.sim_steps = 0
        self.vehicle_id = vehicle_id
        self.planned_trajectory = None
        self.human = human
        self.IDM = IDM
        self.velocity_history = []
        self.heading_history = []
        self.crash_history = []
        self.acc = acc
        self.steering_noise = None
        self.acc_noise = None

        self.LENGTH = v_length  # Vehicle length [m]
        self.WIDTH = v_width  # Vehicle width [m]

    @classmethod
    def create(cls, road, vehicle_id, position, v_length, v_width, interaction_traj, heading=0, velocity=0, acc=0,
               target_velocity=15, human=False, IDM=False):
        """
        Create a human-like (IRL) driving vehicle in replace of a NGSIM vehicle.
        """
        v = cls(road, position, heading, velocity, acc, target_velocity=target_velocity,
                vehicle_id=vehicle_id, v_length=v_length, v_width=v_width, interaction_traj=interaction_traj,
                human=human, IDM=IDM)

        return v

    def trajectory_planner(self, target_point, target_speed, time_horizon):
        """
        Plan a trajectory for the human-like (IRL) vehicle.
        """
        x, x_velocity, x_acc = self.position[0], self.velocity * np.cos(self.heading), self.acc * np.cos(self.heading) # Longitudinal
        y, y_velocity, y_acc = self.position[1], self.velocity * np.sin(self.heading), self.acc * np.sin(self.heading)  # Lateral
        target_area, speed, T = target_point, target_speed, time_horizon

        if not self.human:
            target_area += np.random.normal(0, 0.1)

        path = planner(x, x_velocity, x_acc, y, y_velocity, y_acc, target_area, speed, T)

        self.planned_trajectory = np.array([[x, y] for x, y in zip(path[0].x, path[0].y)])

        # print(self.planned_trajectory)
        # print()

        if self.IDM:
            self.planned_trajectory = None

        # if constant velocity:
        # time = np.arange(0, T*10, 1)
        # path_x = self.position[0] + self.velocity * np.cos(self.heading) * time/10
        # path_y = self.position[1] + self.velocity * np.sin(self.heading) * time/10
        # self.planned_trajectory = np.array([[x, y] for x, y in zip(path_x, path_y)])

    def act(self, step):
        if self.planned_trajectory is not None:
            self.action = {'steering': self.steering_control(self.planned_trajectory, step),
                           'acceleration': self.velocity_control(self.planned_trajectory, step)}

            print(self.action)
        elif self.IDM:
            super(HumanLikeVehicle, self).act()
        else:
            return

    def steering_control(self, trajectory, step):
        """
        Steer the vehicle to follow the given trajectory.

        1. Lateral position is controlled by a proportional controller yielding a lateral velocity command
        2. Lateral velocity command is converted to a heading reference
        3. Heading is controlled by a proportional controller yielding a heading rate command
        4. Heading rate command is converted to a steering angle

        :param trajectory: the trajectory to follow
        :return: a steering wheel angle command [rad]
        """
        target_coords = trajectory[step]
        # Lateral position control
        lateral_velocity_command = self.KP_LATERAL * (target_coords[1] - self.position[1])

        # Lateral velocity to heading
        heading_command = np.arcsin(np.clip(lateral_velocity_command / utils.not_zero(self.velocity), -1, 1))
        heading_ref = np.clip(heading_command, -np.pi/4, np.pi/4)
        # Heading control
        heading_rate_command = self.KP_HEADING*utils.wrap_to_pi(self.heading-heading_ref)

        print("self.heading = {}, heading_command = {}".format(self.heading, heading_command))

        # Heading rate to steering angle
        steering_angle = np.arctan(self.LENGTH / utils.not_zero(self.velocity) * heading_rate_command)
        steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        return steering_angle

    def velocity_control(self, trajectory, step):
        """
        Control the velocity of the vehicle.

        Using a simple proportional controller.

        :param trajectory: the trajectory to follow
        :return: an acceleration command [m/s2]
        """
        target_velocity = np.linalg.norm((trajectory[step]-trajectory[step-1])/0.1)
        acceleration = self.KP_A * (target_velocity - self.velocity)

        return acceleration

    def step(self, dt):
        self.sim_steps += 1
        self.heading_history.append(self.heading)
        self.velocity_history.append(self.velocity)
        self.crash_history.append(self.crashed)

        super(HumanLikeVehicle, self).step(dt)

        self.traj = np.append(self.traj, self.position, axis=0)

    def calculate_human_likeness(self):
        original_traj = self.interaction_traj[:self.sim_steps + 1, :2]
        ego_traj = self.traj.reshape(-1, 2)
        ADE = np.mean([np.linalg.norm(original_traj[i] - ego_traj[i]) for i in
                       range(ego_traj.shape[0])])  # Average Displacement Error (ADE)
        FDE = np.linalg.norm(original_traj[-1] - ego_traj[-1])  # Final Displacement Error (FDE)
        # print("FDE = {}".format(FDE))
        return FDE
