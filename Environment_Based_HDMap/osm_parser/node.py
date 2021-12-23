class Node(object):
    """
    The Node object.
    """
    def __init__(self, n_id: int, x: float, y: float, visible: bool=True):
        """
        Initialization of the Node by setting the x, y coordinate and its visibility attribute.
        """
        self.id = n_id
        self.x = x
        self.y = y
        self.visible = visible

        # The id list of the ways which the node owned.
        self.way_list = []

    def get_coord(self):
        return self.x, self.y

    def __str__(self):
        return "node id = {}, (x, y) = ({}, {}), visible = {}, way_list = {}".format(self.id, self.x, self.y,
                                                                                     self.visible, self.way_list)

    def __repr__(self):
        return self.__str__()

    def to_json(self):
        return {"id": self.id, "x": self.x, "y": self.y, "way_list": self.way_list}

