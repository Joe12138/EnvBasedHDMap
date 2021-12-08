import numpy as np


class Lane(object):
    """
    A lane on the road, described by its four corner coordinates.
    """
    def __init__(self, left_way_list: list, right_way_list: list, left_way_type: str, right_way_type: str,
                 speed_limit=20):
        self.left_way_list = left_way_list
        self.right_way_list = right_way_list
        self.left_way_type = left_way_type
        self.right_Way_type = right_way_type

        self.start = np.array([(self.left_way_list[0][0]+self.right_way_list[0][0])/2,
                               (self.left_way_list[0][1]+self.right_way_list[0][1])/2])

        self.heading = None
        self.length = self.get_lane_length()
        self.speed_limit = speed_limit

    def get_lane_length(self):
        """
        Get the length of the lane.

        :return: the length of the lane.
        """
        start_pos = ((self.left_way_list[0][0]+self.right_way_list[0][0])/2,
                     (self.left_way_list[0][1]+self.right_way_list[0][1])/2)
        end_pos = ((self.left_way_list[1][0]+self.right_way_list[1][0])/2,
                     (self.left_way_list[1][1]+self.right_way_list[1][1])/2)

        return np.linalg.norm([start_pos[0]-end_pos[0], start_pos[1]-end_pos[1]])
