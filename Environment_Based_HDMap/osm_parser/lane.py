class Lane(object):
    """
    The information of the Lane object.
    """
    def __init__(self, l_id: int, left_way_id: int, right_way_id: int):
        """
        Initialize the lane object by setting its id, left and right way id.
        """
        self.id = l_id
        self.left_way_id = left_way_id
        self.right_way_id = right_way_id

    def __str__(self):
        return "Lane id = {}, (left_way_id, right_way_id) = ({}, {})".format(self.id, self.left_way_id,
                                                                             self.right_way_id)

    def __repr__(self):
        return self.__str__()

    def to_json(self):
        return {"id": self.id, "left_way_id": self.left_way_id, "right_way_id": self.right_way_id}


class CompleteLane(object):
    """
    The information of the CompleteLane object.

    Construct the complete lane according to the information provided by .osm_parser file.
    """
    def __init__(self, start_str: str, end_str: str):
        self.start_str = start_str
        self.end_str = end_str

        self.lane_list = []
        self.left_way_list = []
        self.right_way_list = []

        self.left_node_list = None
        self.right_node_list = None

        self.left_way_type_list = None
        self.right_way_type_list = None

        self.lane_coordinate_dict = {}

    def get_node_list(self, id_way_dict: dict, direction: str) -> list:
        """
        Get the node_id list of the left or right way in this complete lane.

        return: a list of the node_id of the left or right way
        """
        node_list = []

        if direction == "left" or direction == "Left" or direction == "L":
            way_list = self.left_way_list
        else:
            way_list = self.right_way_list

        for way_id in way_list:
            ref_node_list = id_way_dict[way_id].ref_node_list

            if way_id == way_list[0]:
                node_list.extend(ref_node_list)
            else:
                node_list.extend(ref_node_list[1:])

        return node_list

    def get_way_type_list(self, id_way_dict: dict, direction: str) -> list:
        """
        Get the way_type including "solid" and "dashed" list of the left or right way in this complete lane.

        return: a list of the way type of the left or right way.
        """
        way_type_list = []

        if direction == "left" or direction == "Left" or direction == "L":
            way_list = self.left_way_list
        else:
            way_list = self.right_way_list

        for way_id in way_list:
            way_obj = id_way_dict[way_id]

            if way_obj.type == "line_thin" or way_obj.type == "line_thick":
                way_type_list.append(way_obj.subtype)
            else:
                way_type_list.append("solid")

        return way_type_list


class DrawLane(object):
    """
    The Lane Object which is used to draw on the map. That means that the DrawLane Object is after interpolation.
    """
    def __init__(self, start_str: str, end_str: str):
        self.start_str = start_str
        self.end_str = end_str

        self.id = None

        # Dict: key = index, value = coordinate list [[left way], [right way]]
        self.index_coord_dict = {}

        # Dict: key = index, value = (start_point_coordinate, end_point_coordinate)
        self.index_center_dict = {}

        # Dict: key = index, value = type string (left way type, right way type)
        self.index_type_dict = {}

        # Dict: key = index, value = width (float object)
        self.index_width_dict = {}

        self.left_lane_id = None
        self.right_lane_id = None
        self.right_lane_id = None

    def to_json(self):
        return {"start_str": self.start_str, "end_str": self.end_str, "id": self.id,
                "index_coord_dict": self.index_coord_dict, "index_type_dict": self.index_type_dict}


