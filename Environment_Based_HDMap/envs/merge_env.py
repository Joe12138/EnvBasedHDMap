import sys

import numpy as np

from envs.common.abstract import AbstractEnv
from Interaction_data.data_process import InteractionDataset
from vehicle.human_driving import *
from road.road_network import *
from road.lane import *


class MergeEnv(AbstractEnv):
    def __init__(self, dataset_file_path: str, osm_file_path: str, vehicle_id: int, case_id: int, IDM: bool = False,
                 lat_origin: float = 0.0, lon_origin: float = 0.0):
        self.interaction_data = InteractionDataset(dataset_file_path=dataset_file_path)
        self.hd_map = HDMap(osm_file_path=osm_file_path, lat_origin=lat_origin, lon_origin=lon_origin)
        self.min_x, self.min_y, self.max_x, self.max_y = self.set_visible_area()
        self.vehicle_id = vehicle_id
        self.case_id = case_id
        self.human = False
        self.IDM = IDM

        # the number of the frames in the whole trajectory of the ego vehicle
        self.duration = len(self.interaction_data.id_traj_dict[(self.case_id, self.vehicle_id)].trajectory)

        # the distance threshold which is about neighbour vehicles
        self.dist_threshold = 100

        # Get all neighbour vehicles and add them into the SnapShot Object
        self.get_neighbour_vehicles(self.dist_threshold)

        # Get all surrounding vehicles at all time
        self.surrounding_vehicles_id_set = self.get_all_neighbour_vehicle()

        self.trajectory_set = self.get_trajectory_set()
        self.ego_length = self.trajectory_set["ego"]["length"]
        self.ego_width = self.trajectory_set["ego"]["width"]
        self.ego_trajectory = self.trajectory_set["ego"]["trajectory"]

        self.run_step = 0

        super(MergeEnv, self).__init__()

    def get_neighbour_vehicles(self, dist_threshold: float):
        """
        For each frame, find the neighbor vehicles around the ego vehicle within the "dist_threshold" distance
        """
        traj_obj = self.interaction_data.id_traj_dict[(self.case_id, self.vehicle_id)]

        for frame_id, v_obj in traj_obj.time_state_dict.items():
            self.interaction_data.get_neighbour_vehicle(ego_vehicle_obj=v_obj, dist_threshold=dist_threshold)

    def get_all_neighbour_vehicle(self) -> set:
        """
        Get all neighbour vehicles at all time steps.

        @return Set: all neighbour vehicles id in a set.
        """
        neighbour_vehicle_set = set()

        for key, snapshot_obj in self.interaction_data.id_snapshot_dict.items():
            for vehicle_obj in snapshot_obj.neighbor_vehicle_list:
                if vehicle_obj.id != self.vehicle_id:
                    neighbour_vehicle_set.add(vehicle_obj.id)

        return neighbour_vehicle_set

    def set_visible_area(self):
        """
        Find the minimum and maximum x and y coordinate.
        """
        min_x = 10e9
        min_y = 10e9
        max_x = -10e9
        max_y = -10e9

        for point in self.hd_map.lanelet_map.pointLayer:
            min_x = min(point.x, min_x)
            min_y = min(point.y, min_y)
            max_x = max(point.x, max_x)
            max_y = max(point.y, max_y)

        return 1000-max_x, min_y, 1000-min_x, max_y

    def get_trajectory_set(self):
        """
        Convert the class object of the trajectory to list object.
        """
        record_trajectory = {"ego": {"length": 0, "width": 0, "trajectory": []}}

        ego_trajectory_obj = self.interaction_data.id_traj_dict[(self.case_id, self.vehicle_id)]
        record_trajectory["ego"]["length"] = ego_trajectory_obj.v_length
        record_trajectory["ego"]["width"] = ego_trajectory_obj.v_width

        ego_trajectory = []
        for vehicle_state_obj in ego_trajectory_obj.trajectory:
            x = vehicle_state_obj.x
            y = vehicle_state_obj.y
            speed = vehicle_state_obj.speed
            psi_rad = vehicle_state_obj.psi_rad

            lane_result = self.hd_map.position_to_lane((x, y))
            if lane_result is None:
                raise Exception("Something Wrong in hd_map position_to_lane method.")

            ego_trajectory.append([x, y, speed, lane_result, psi_rad])

        record_trajectory["ego"]["trajectory"] = self.trajectory_smoothing(ego_trajectory)
        ego_time_list = list(ego_trajectory_obj.time_state_dict.keys())

        for veh_id in self.surrounding_vehicles_id_set:
            trajectory_list = []
            trajectory_obj = self.interaction_data.id_traj_dict[(self.case_id, veh_id)]
            v_length = trajectory_obj.v_length
            v_width = trajectory_obj.v_width

            for frame_id in ego_time_list:
                try:
                    vehicle_state_obj = trajectory_obj.time_state_dict[frame_id]
                    x = vehicle_state_obj.x
                    y = vehicle_state_obj.y
                    speed = vehicle_state_obj.speed
                    psi_rad = vehicle_state_obj.psi_rad

                    lane_result = self.hd_map.position_to_lane((x, y))
                    if lane_result is None:
                        raise Exception("Something Wrong in hd_map position_to_lane method.")

                    trajectory_list.append([x, y, speed, lane_result, psi_rad])
                except KeyError:
                    trajectory_list.append([0, 0, 0, [-1, 0, 0, 0], 0])
            record_trajectory[veh_id] = {"length": v_length,
                                         "width": v_width,
                                         "trajectory": self.trajectory_smoothing(trajectory_list)}

        return record_trajectory

    @staticmethod
    def trajectory_smoothing(trajectory):
        trajectory = np.array(trajectory)
        x = trajectory[:, 0]
        y = trajectory[:, 1]
        speed = trajectory[:, 2]
        lane = trajectory[:, 3]
        psi_rad = trajectory[:, 4]

        # window_length = 21 if len(x[np.nonzero(x)]) >= 21 else len(x[np.nonzero(x)]) if len(x[np.nonzero(x)])%2 != 0 else len(x[np.nonzero(x)])-1
        # x[np.nonzero(x)] = signal.savgol_filter(x[np.nonzero(x)], window_length=window_length,
        #                                         polyorder=3)  # window size used for filtering, order of fitted polynomial
        # y[np.nonzero(y)] = signal.savgol_filter(y[np.nonzero(y)], window_length=window_length, polyorder=3)
        # speed[np.nonzero(speed)] = signal.savgol_filter(speed[np.nonzero(speed)], window_length=window_length,
        #                                                 polyorder=3)

        return [[float(x), float(y), float(s), l, float(r)] for x, y, s, l, r in zip(x, y, speed, lane, psi_rad)]

    @classmethod
    def default_config(cls) -> dict:
        # min_x, min_y, max_x, max_y = cls.set_visible_area(cls)
        cfg = super(MergeEnv, cls).default_config()
        cfg.update({
            "observation": {"type": "Kinematics"},
            "vehicles_count": 10,
            "show_trajectories": True,
            "screen_width": 640,
            "screen_height": 320
        })
        # print(cfg)
        return cfg

    def reset(self, human=False, reset_time=1):
        """
        Reset the environment at a given time (scene) and specify whether use human target
        """

        self.human = human
        self._create_road()
        self._create_vehicles(reset_time)
        self.steps = 0

        return super().reset()

    def _create_road(self):
        """
        Create a road composed of INTERACTION road network.
        """
        net = RoadNetwork()

        for draw_lane_index, draw_lane_obj in self.hd_map.draw_lane_dict.items():
            for key, coord in draw_lane_obj.index_coord_dict.items():
                left_way_list = coord[0]
                right_way_list = coord[1]

                way_type = draw_lane_obj.index_type_dict[key]

                lane = Lane(left_way_list=left_way_list,
                            right_way_list=right_way_list,
                            left_way_type=way_type[0],
                            right_way_type=way_type[1],
                            draw_lane_id=draw_lane_index,
                            index=key)
                net.add_lane(draw_lane_obj.start_str, draw_lane_obj.end_str, lane)

        self.road = Road(network=net,
                         np_random=self.np_random,
                         record_history=self.config["show_trajectories"])

    def _create_vehicles(self, reset_time):
        ego_trajectory = self.ego_trajectory[reset_time:]
        ego_acc = (self.ego_trajectory[reset_time][2]-self.ego_trajectory[reset_time-1][2])/0.1

        self.vehicle = HumanLikeVehicle.create(self.road,
                                               self.vehicle_id,
                                               ego_trajectory[0][:2],
                                               self.ego_length,
                                               self.ego_width,
                                               np.array(ego_trajectory),
                                               acc=ego_acc,
                                               velocity=ego_trajectory[0][2],
                                               human=self.human,
                                               IDM=self.IDM,
                                               heading=ego_trajectory[0][4])

        self.road.vehicles.append(self.vehicle)

        for veh_id in self.surrounding_vehicles_id_set:
            if veh_id == 1 or veh_id == 2:
                continue
            other_trajectory = self.trajectory_set[veh_id]["trajectory"][reset_time:]
            if len(other_trajectory) == 0:
                continue

            self.road.vehicles.append(InteractionVehicle.create(
                self.road,
                veh_id,
                other_trajectory[0][:2],
                self.trajectory_set[veh_id]["length"],
                self.trajectory_set[veh_id]["width"],
                np.array(other_trajectory),
                velocity=other_trajectory[0][2],
                heading=other_trajectory[0][4]
            ))

    def step(self, action=None):
        """
        Perform a MDP step
        """
        if self.road is None or self.vehicle is None:
            raise NotImplementedError("The road and vehicle must be initialized in the environment implementation")

        features = self._simulate(action)
        obs = self.observation.observe()
        terminal = self._is_terminal()

        on_road_result = self.road.network.get_closest_lane_index(self.vehicle.position)

        info = {
            "velocity": self.vehicle.velocity,
            "crashed": self.vehicle.crashed,
            "offroad": False if on_road_result is None else True,
            "action": action,
            "time": self.time
        }

        return obs, features, terminal, info

    def _simulate(self, action):
        """
        Perform several steps of simulation with the planned trajectory
        """
        trajectory_features = []
        T = action[2] if action is not None else 3

        for i in range(int(T * self.SIMULATION_FREQUENCY) - 1):
            if i == 0:
                if action is not None:  # sampled goal
                    # lateral coordinate, speed, 1
                    self.vehicle.trajectory_planner(action[0], action[1], action[2])
                else:  # human goal
                    self.vehicle.trajectory_planner(self.vehicle.interaction_traj[self.vehicle.sim_steps + T * 10][1],
                                                    (self.vehicle.interaction_traj[self.vehicle.sim_steps + T * 10][0] -
                                                     self.vehicle.interaction_traj[self.vehicle.sim_steps + T * 10 - 1][
                                                         0]) / 0.1, T)
                self.run_step = 1

            self.road.act(self.run_step)
            self.road.step(1 / self.SIMULATION_FREQUENCY)
            print("run_step = {}".format(self.run_step))
            print(self.vehicle.position)
            print(self.vehicle.planned_trajectory[self.run_step])
            print()
            self.time += 1
            self.run_step += 1
            features = self._features()
            trajectory_features.append(features)

            self._automatic_rendering()

            # Stop at terminal states
            if self.done or self._is_terminal():
                break

        self.enable_auto_render = False

        human_likeness = features[-1]
        interaction = np.max([feature[-2] for feature in trajectory_features])
        trajectory_features = np.sum(trajectory_features, axis=0)
        trajectory_features[-1] = human_likeness


        return trajectory_features

    def _features(self):
        """
        Hand-crafted features
        :return: the array of the defined features
        """
        # ego motion
        a = self.vehicle.traj
        ego_longitudial_positions = self.vehicle.traj.reshape(-1, 2)[self.time - 3:, 0]
        ego_longitudial_speeds = (ego_longitudial_positions[1:] - ego_longitudial_positions[
                                                                  :-1]) / 0.1 if self.time >= 3 else [0]
        ego_longitudial_accs = (ego_longitudial_speeds[1:] - ego_longitudial_speeds[:-1]) / 0.1 if self.time >= 3 else [
            0]
        ego_longitudial_jerks = (ego_longitudial_accs[1:] - ego_longitudial_accs[:-1]) / 0.1 if self.time >= 3 else [0]

        ego_lateral_positions = self.vehicle.traj.reshape(-1, 2)[self.time - 3:, 1]
        ego_lateral_speeds = (ego_lateral_positions[1:] - ego_lateral_positions[:-1]) / 0.1 if self.time >= 3 else [0]
        ego_lateral_accs = (ego_lateral_speeds[1:] - ego_lateral_speeds[:-1]) / 0.1 if self.time >= 3 else [0]

        # travel efficiency
        ego_speed = abs(ego_longitudial_speeds[-1])

        # comfort
        ego_longitudial_acc = ego_longitudial_accs[-1]
        ego_lateral_acc = ego_lateral_accs[-1]
        ego_longitudial_jerk = ego_longitudial_jerks[-1]

        # time headway front (THWF) and time headway behind (THWB)
        THWFs = [100]
        THWBs = [100]
        for v in self.road.vehicles:
            if v.position[0] > self.vehicle.position[0] and abs(
                    v.position[1] - self.vehicle.position[1]) < self.vehicle.WIDTH and self.vehicle.velocity >= 1:
                THWF = (v.position[0] - self.vehicle.position[0]) / self.vehicle.velocity
                THWFs.append(THWF)
            elif v.position[0] < self.vehicle.position[0] and abs(
                    v.position[1] - self.vehicle.position[1]) < self.vehicle.WIDTH and v.velocity >= 1:
                THWB = (self.vehicle.position[0] - v.position[0]) / v.velocity
                THWBs.append(THWB)

        THWF = np.exp(-min(THWFs))
        THWB = np.exp(-min(THWBs))

        # avoid collision

        on_road_result = self.road.network.get_closest_lane_index(self.vehicle.position)
        collision = 1 if self.vehicle.crashed or on_road_result is None else 0

        # interaction (social) impact
        social_impact = 0
        for v in self.road.vehicles:
            if isinstance(v, InteractionVehicle) and v.overtaken and v.velocity != 0:
                social_impact += np.abs(v.velocity - v.velocity_history[-1]) / 0.1 if v.velocity - v.velocity_history[
                    -1] < 0 else 0

        # ego vehicle human-likeness
        ego_likeness = self.vehicle.calculate_human_likeness()

        # feature array
        fetures = np.array([ego_speed, abs(ego_longitudial_acc), abs(ego_lateral_acc), abs(ego_longitudial_jerk),
                            THWF, THWB, collision, social_impact, ego_likeness])

        return fetures

    def _is_terminal(self):
        """
        The episode is over if the ego vehicle crashed or go off road or the time is out.
        """
        on_road_result = self.road.network.get_closest_lane_index(self.vehicle.position)

        return self.vehicle.crashed or self.time >= self.duration or (on_road_result is None)

    def sampling_space(self):
        """
        The target sampling space (longitudinal speed and lateral offset)
        """
        lane_center = self.vehicle.lane.start[1]
        current_y = self.vehicle.position[1]
        current_x = self.vehicle.position[0]
        current_speed = self.vehicle.velocity

        lane_offset_list = [current_y, lane_center]
        draw_lane_obj = self.hd_map.draw_lane_dict[self.vehicle.lane.draw_lane_id]
        if draw_lane_obj.left_lane_id is not None:
            position = self.road.get_y_coordinate(
                self.hd_map.draw_lane_dict[draw_lane_obj.left_lane_id].index_center_dict, current_x)
            lane_offset_list.append(position[1])
        if draw_lane_obj.right_lane_id is not None:
            position = self.road.get_y_coordinate(
                self.hd_map.draw_lane_dict[draw_lane_obj.right_lane_id].index_center_dict, current_x)
            lane_offset_list.append(position[1])

        min_speed = current_speed - 5 if current_speed > 5 else 0
        max_speed = current_speed + 5
        target_speeds = np.linspace(min_speed, max_speed, 10)

        return np.array(lane_offset_list), target_speeds