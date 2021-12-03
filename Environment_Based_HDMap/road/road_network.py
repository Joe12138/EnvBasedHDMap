from road.lane import Lane
from osm_parser.hd_map import HDMap


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

    def get_lane(self, start_node, end_node, index: int):
        """
         Get the geometry corresponding to a given index in the road network.
        :param start_node: the label of the node which the lane starts
        :param end_node: the label of the node which the lane ends
        :param index: the index of the lane in the lane list
        :return: Lane object
        """
        if index is None and len(self.graph[start_node][end_node]) == 1:
            index = 0
        return self.graph[start_node][end_node][index]

    def get_lane_index(self, position: tuple):
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
        raise KeyError("The position is not in the lane of the road network!")


class Road(object):
    """
    A road is a set of lanes, and a set of vehicles driving on these lanes.
    """

    def __init__(self, network=None, vehicles=None, record_history=False):
        pass

