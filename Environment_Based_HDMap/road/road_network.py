from road.lane import Lane
from osm_parser.hd_map import HDMap
import numpy as np
from vehicle.human_driving import InteractionVehicle


class RoadNetwork(object):
    def __init__(self):
        self.graph = {}

    def add_node(self, node):
        """
        A node represents an symbolic intersection in the road network.
        :param node: the node label
        :return: No return object
        """
        if node not in self.graph.keys():
            self.graph[node] = []

    def add_lane(self, _from, _to, lane):
        """
        A lane is encoded as an edge in the road network.
        :param _from: the node which the lane starts
        :param _to: the node at which the lane ends
        :param lane: Lane object
        :return: No return object
        """
        if _from not in self.graph.keys():
            self.graph[_from] = {}
        if _to not in self.graph[_from].keys():
            self.graph[_from][_to] = []

        self.graph[_from][_to].append(lane)

    def get_lane(self, lane_index):
        """
         Get the geometry corresponding to a given index in the road network.
        :param start_node: the label of the node which the lane starts
        :param end_node: the label of the node which the lane ends
        :param index: the index of the lane in the lane list
        :return: Lane object
        """
        # if lane_index is None:
        #     return None
        # else:
        start_node, end_node, index = lane_index[0], lane_index[1], lane_index[2]
        if index is None and len(self.graph[start_node][end_node]) == 1:
            index = 0
        return self.graph[start_node][end_node][index]

    def get_closest_lane_index(self, position: tuple):
        """
        Get the index of the lane closest to a world position.
        :param position: a world position [m]
        :return: the index of the closest lane including label of the start and end node and the index of the lane in
        the lane list
        """
        for _from, to_dict in self.graph.items():
            for _to, lane_list in to_dict.items():
                for _id, lane in enumerate(lane_list):
                    result = HDMap.position_in_rectangle(p=position,
                                                         a=lane.left_way_list[0],
                                                         b=lane.right_way_list[0],
                                                         c=lane.right_way_list[1],
                                                         d=lane.left_way_list[1])
                    if result:
                        return _from, _to, _id

        return None
        # raise KeyError("The position is not in the lane of the road network!")

    def get_lanes_heading(self):
        """
        Update the lane heading when the road network is complete.
        :return:
        """
        for _from, to_dict in self.graph.items():
            for _to, lane_list in to_dict.items():
                for _id, lane in enumerate(lane_list):
                    left_way_list = lane.left_way_list
                    result = self.refer_left(lane)

                    if result:
                        lane.heading = np.arctan2(left_way_list[1][1]-left_way_list[0][1],
                                                  left_way_list[1][0]-left_way_list[0][0])
                    else:
                        lane.heading = np.arctan2(lane.right_way_list[1][1]-lane.right_way_list[0][1],
                                                  lane.right_way_list[1][0]-lane.right_way_list[0][0])

    def refer_left(self, lane) -> bool:
        middle_point = (
            (lane.left_way_list[0][0] + lane.left_way_list[1][0]) / 2,
            (lane.left_way_list[0][1] + lane.left_way_list[1][1]) / 2
        )
        lane_width = HDMap.point_to_line_distance(point=middle_point,
                                                  line_point1=lane.right_way_list[0],
                                                  line_point2=lane.right_way_list[1])
        left_middle_point = (middle_point[0], middle_point[1] - lane_width / 2)
        # right_middle_point = (middle_point[0], middle_point[1]+lane_width/2)
        result = self.get_closest_lane_index(left_middle_point)
        if result is None:
            return True
        else:
            return False

    def all_side_lanes(self, lane_index):
        """
        :param lane_index: the index of a lane.
        :return: all indexes of lanes belonging to the same road.
        """
        return self.graph[lane_index[0]][lane_index[1]]


class Road(object):
    """
    A road is a set of lanes, and a set of vehicles driving on these lanes.
    """

    def __init__(self, network: RoadNetwork = None, vehicles=None, np_random=None, record_history=False):
        """
        New road

        :param network: the road network describing the lanes.
        :param vehicles: the vehicles driving on the road
        :param np.random.RandomState np_random: a random number generator for vehicle behaviour
        :param record_history: whether the recent trajectories of vehicles should be recorded for display
        """
        self.network = network or []
        self.vehicles = vehicles or []
        self.np_random = np_random if np_random else np.random
        self.record_history = record_history

    @staticmethod
    def is_front(ego_vehicle_x: float, other_vehicle_x: float, positive_direction: str) -> bool:
        if positive_direction == "left" or positive_direction == "Left" or positive_direction == "L":
            if ego_vehicle_x < other_vehicle_x:
                return False
            else:
                return True
        else:
            if ego_vehicle_x < other_vehicle_x:
                return True
            else:
                return False

    def neighbour_vehicles(self, vehicle, lane_index=None):
        """
        Find the preceding and following vehicles of a given vehicle.
        Positive direction is judged by the heading of the ego vehicle.

        :param vehicle: the vehicle whose neighbours must be found.
        :param lane_index: the lane on which to look for preceding and following vehicles.
                    It doesn't have to be the current vehicle lane but can also be another lane, in which case the
                    vehicle is projected on it considering its local coordinates in the lane.

                    In this method, we temply don't have this function.
        :return: its preceding vehicle, its following vehicle.
        """
        lane_index = vehicle.lane_index

        if not lane_index:
            return None, None

        ego_vehicle_x = vehicle.position[0]
        ego_vehicle_heading = vehicle.heading

        front_dist = 10e9
        rear_dist = 10e9
        v_front = v_rear = None
        for v in self.vehicles:
            v_lane_index = v.lane_index
            if v_lane_index[0] == lane_index[0] and v_lane_index[1] == lane_index[1]:
                if -1.5 * np.pi < ego_vehicle_heading < -0.5 * np.pi or 0.5 * np.pi < ego_vehicle_heading < 1.5 * \
                        np.pi or 2.5 * np.pi < ego_vehicle_heading < 3.5 * np.pi:
                    front_result = self.is_front(ego_vehicle_x, v.position[0], "Left")
                    difference = abs(ego_vehicle_x - v.position[0])
                    if front_result:
                        if difference < front_dist:
                            front_dist = difference
                            v_front = v
                    else:
                        if difference < rear_dist:
                            rear_dist = difference
                            v_rear = v

        return v_front, v_rear

    def dist_between_vehicles_on_same_lane(self, ego_vehicle, other_vehicle):
        """
        Compute the distance between two vehicles which are on the same lanes.
        :param ego_vehicle:
        :param other_vehicle:
        :return:
        """
        ego_lane_index = ego_vehicle.lane_index
        other_lane_index = other_vehicle.lane_index

        # if ego_vehicle is None or other_vehicle is None:
        #     return 10e9

        if ego_lane_index[0] == other_lane_index[0] and ego_lane_index[1] == other_lane_index[1]:
            if ego_lane_index[2] == other_lane_index[2]:
                lane = self.network.graph[ego_lane_index[0]][ego_lane_index[1]][ego_lane_index[2]]
                delta_vector = np.array([
                    ego_vehicle.position[0]-other_vehicle.position[0],
                    ego_vehicle.position[1]-other_vehicle.position[1]
                ])
                refer_vector = self.get_refer_vector(lane)

                dist = np.dot(delta_vector, refer_vector)/np.linalg.norm(refer_vector)

                return dist
            else:
                if ego_lane_index[2] < other_lane_index[2]:
                    small_vehicle = ego_vehicle
                    large_vehicle = other_vehicle
                else:
                    small_vehicle = other_vehicle
                    large_vehicle = ego_vehicle

                dist = 0

                for index in range(small_vehicle.lane_index[2], large_vehicle.lane_index[2]+1):
                    lane = self.network.graph[small_vehicle.lane_index[0]][small_vehicle.lane_index[1]][index]
                    if index == small_vehicle.lane_index[2]:
                        pos = small_vehicle.position

                        end_pos = np.array(lane.right_way_list[1])
                        delta_vector = end_pos-pos
                        refer_vector = self.get_refer_vector(lane)

                        dist += np.dot(delta_vector, refer_vector) / np.linalg.norm(refer_vector)
                    elif index == large_vehicle.lane_index[2]:
                        pos = large_vehicle.position

                        end_pos = np.array(lane.right_way_list[1])
                        delta_vector = end_pos - pos
                        refer_vector = self.get_refer_vector(lane)

                        dist += np.dot(delta_vector, refer_vector) / np.linalg.norm(refer_vector)
                    else:
                        dist += lane.length

                return dist
        else:
            # raise Exception("Sorry! These two vehicle are not in the same lane.")
            return 10e9

    def get_refer_vector(self, lane):
        refer_left = self.network.refer_left(lane)
        if refer_left:
            refer_vector = np.array(lane.left_way_list[1]) - np.array(lane.left_way_list[0])
        else:
            refer_vector = np.array(lane.right_way_list[1]) - np.array(lane.right_way_list[0])
        return refer_vector

    def act(self, step):
        """
        Decide the actions of each entity on the road.
        :param step:
        :return:
        """
        for vehicle in self.vehicles:
            if isinstance(vehicle, InteractionVehicle):
                vehicle.act()
            else:
                vehicle.act(step)

    def step(self, dt):
        """
        Step the dynamics of each entity on the road.

        :param dt: time step [s]
        :return:
        """
        for vehicle in self.vehicles:
            vehicle.step(dt)

        for vehicle in self.vehicles:
            for other in self.vehicles:
                vehicle.check_collision(other)

    def close_vehicles_to(self, vehicle, distance, count=None, sort=False, see_behind=True):
        vehicles = [v for v in self.vehicles
                    if np.linalg.norm(v.position - vehicle.position) < distance
                    and v is not vehicle
                    and (see_behind or -2*vehicle.LENGTH < self.dist_between_vehicles_on_same_lane(vehicle, v))]
        if sort:
            vehicles = sorted(vehicles, key=lambda v: abs(self.dist_between_vehicles_on_same_lane(vehicle, v)))
        if count:
            vehicles = vehicles[:count]

        return vehicles



