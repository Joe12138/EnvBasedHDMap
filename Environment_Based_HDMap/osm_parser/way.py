class Way(object):
    """
    The information of Way object.
    """
    def __init__(self, w_id: int, way_type: str, subtype: str, ref_node_list: list):
        """
        Initialization of the way by setting its id, type and subtype of the way, reference node id list.
        """
        self.id = w_id
        self.type = way_type
        self.subtype = subtype
        self.ref_node_list = ref_node_list

        # The id list of the lanes which the way owned.
        self.lane_list = []

    def __str__(self):
        return "way id = {}, type = {}, subtype = {}, ref_node_list = {}, lane_list = {}".format(self.id, self.type,
                                                                                                 self.subtype,
                                                                                                 self.ref_node_list,
                                                                                                 self.lane_list)

    def __repr__(self):
        return self.__str__()

    def to_json(self):
        return {"id": self.id, "way_type": self.type, "subtype": self.subtype, "ref_node_list": self.ref_node_list,
                "lane_list": self.lane_list}