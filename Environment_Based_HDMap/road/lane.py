class Lane(object):
    """
    A lane on the road, described by its four corner coordinates.
    """
    def __init__(self, left_way_list: list, right_way_list: list, left_way_type: str, right_way_type: str):
        self.left_way_list = left_way_list
        self.right_way_list = right_way_list
        self.left_way_type = left_way_type
        self.right_Way_type = right_way_type

