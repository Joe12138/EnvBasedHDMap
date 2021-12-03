import lanelet2
import copy
from osm_parser.node import Node
from osm_parser.way import Way
from osm_parser.lane import Lane, CompleteLane, DrawLane
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


class HDMap(object):
    def __init__(self, osm_file_path: str, lat_origin: float = 0.0, lon_origin: float = 0.0):
        self.projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(lat_origin, lon_origin))
        self.lanelet_map = lanelet2.io.load(osm_file_path, self.projector)

        # self.id_node_dict: dict: key=node id, value=Node object
        self.id_node_dict = self.get_node()

        # self.id_way_dict: dict: key=way_id, value=Way object
        self.id_way_dict = self.get_way()

        # self.id_lane_dict: dict: key=lane_id, value=Lane object
        self.id_lane_dict = self.get_lane()
        self.change_order_for_road()

        # self.regulatory_element_dict = self.get_regulatory_element()

        self.max_interval = self.get_interval_parameter()

        self.complete_lane_dict = self.get_complete_lane()

        self.draw_lane_dict = self.complete_lane_to_draw_lane()

    def get_node(self) -> dict:
        """
        Get node's id and coordinate from the file.
        """
        node_coord_dict = {}

        for p in self.lanelet_map.pointLayer:
            node = Node(n_id=int(p.id), x=float(p.x), y=float(p.y))
            node_coord_dict[int(p.id)] = node

        return node_coord_dict

    def get_way(self) -> dict:
        """
        Get way's information from the file.
        """
        way_dict = {}

        for ls in self.lanelet_map.lineStringLayer:
            ref_node_list = []

            way_type = ls.attributes["type"]
            way_subtype = None

            if way_type == "line_thin" or way_type == "line_thick":
                way_subtype = ls.attributes["subtype"]

            for ref_p in ls:
                ref_node_list.append(int(ref_p.id))

                self.id_node_dict[int(ref_p.id)].way_list.append(int(ls.id))

            way = Way(w_id=int(ls.id), way_type=way_type, subtype=way_subtype, ref_node_list=ref_node_list)
            way_dict[int(ls.id)] = way

        return way_dict

    def get_lane(self):
        """
        Get Lane's information from the file.
        """
        id_lane_dict = {}
        for lane in self.lanelet_map.laneletLayer:
            lane_obj = Lane(l_id=int(lane.id), left_way_id=int(lane.leftBound.id), right_way_id=int(lane.rightBound.id))

            left_way_obj = self.id_way_dict[lane_obj.left_way_id]
            right_way_obj = self.id_way_dict[lane_obj.right_way_id]

            # if left_way_obj.type == "virtual" or right_way_obj == "virtual":
            #     continue
            #
            # if left_way_obj.type == "traffic_sign" or right_way_obj.type == "traffic_sign":
            #     continue

            id_lane_dict[int(lane.id)] = lane_obj

            self.id_way_dict[lane_obj.left_way_id].lane_list.append(lane_obj.id)
            self.id_way_dict[lane_obj.right_way_id].lane_list.append(lane_obj.id)

        return id_lane_dict

    def change_order_for_lane(self, lane_id: int):
        """
        In .osm file, the direction of each way is not unified. Therefore, we need to change the order of reference
        nodes of each way such that their direction are unified. In this method, the direction is from small to large
        according to the "x" coordinate of the reference node.

        Change the order of reference node for the lane which the lane id is "lane_id".
        """
        lane_obj = self.id_lane_dict[lane_id]
        left_way_list = self.id_way_dict[lane_obj.left_way_id].ref_node_list
        right_way_list = self.id_way_dict[lane_obj.right_way_id].ref_node_list

        left_way_dict = {}
        right_way_dict = {}

        for p in left_way_list:
            left_way_dict[p] = self.id_node_dict[p].x

        for p in right_way_list:
            right_way_dict[p] = self.id_node_dict[p].x

        left_way = sorted(left_way_dict.items(), key=lambda x: x[1])
        right_way = sorted(right_way_dict.items(), key=lambda x: x[1])

        left = [p[0] for p in left_way]
        right = [p[0] for p in right_way]

        self.id_way_dict[lane_obj.left_way_id].ref_node_list = copy.deepcopy(left)
        self.id_way_dict[lane_obj.right_way_id].ref_node_list = copy.deepcopy(right)

    def change_order_for_road(self):
        """
        Change the order of reference nodes of ways for the whole road.
        """
        for lane in self.id_lane_dict.keys():
            self.change_order_for_lane(lane)

    def get_regulatory_element(self) -> dict:
        """
        Get regulatory information from the file.
        """
        regulatory_element = {}
        for r_element in self.lanelet_map.regulatoryElementLayer:
            regulatory_element[int(r_element.id)] = (r_element.attributes["subtype"], r_element.attributes["sign_type"])

        return regulatory_element

    def get_interval_parameter(self) -> float:
        """
        Get the estimation parameter which is to decide which two points are one-to-one corresponding or not.
        The word "estimation" means that we only consider the points of the begin and end of the ways.

        Assume there are two points A(x1, y1) and B(x2, y2)
        if abs(x1-x2) > this parameter:
            A and B are one-to-one corresponding points.
        else:
            A and B are not one-to-one corresponding points.
        """
        max_interval = 10e-9
        for lane in self.id_lane_dict.items():
            left_way_list = self.id_way_dict[lane[1].left_way_id].ref_node_list
            right_way_list = self.id_way_dict[lane[1].right_way_id].ref_node_list

            first_node_interval = abs(self.id_node_dict[left_way_list[0]].x-self.id_node_dict[right_way_list[0]].x)
            last_node_interval = abs(self.id_node_dict[left_way_list[-1]].x-self.id_node_dict[right_way_list[-1]].x)

            if first_node_interval > max_interval:
                max_interval = first_node_interval

            if last_node_interval > max_interval:
                max_interval = last_node_interval

        return max_interval+0.1

    def need_interpolation(self, lane_id: int) -> bool:
        """
        To check that the lane needs interpolate or not?

        Why do this?
        Since for a lane in .osm file, the number of the reference nodes of two ways which are owned the same lane is
        not one-to-one corresponding.
        """
        lane_obj = self.id_lane_dict[lane_id]
        left_way_list = self.id_way_dict[lane_obj.left_way_id].ref_node_list
        right_way_list = self.id_way_dict[lane_obj.right_way_id].ref_node_list

        if len(left_way_list) != len(right_way_list):
            return False
        else:
            for i in range(len(left_way_list)):
                left_node = self.id_node_dict[left_way_list[i]].x
                right_node = self.id_node_dict[right_way_list[i]].x

                if abs(left_node-right_node) >= self.max_interval:
                    return False

        return True

    def linear_interpolation_for_lane_with_small_interval(self, lane_id: int, interval: float = 1):
        lane_obj = self.id_lane_dict[lane_id]
        left_way_list = self.id_way_dict[lane_obj.left_way_id].ref_node_list
        right_way_list = self.id_way_dict[lane_obj.right_way_id].ref_node_list

        left_original_list = [[], []]
        right_original_list = [[], []]

        total_x_list = []

        for index in range(len(left_way_list)):
            left_node_obj = self.id_node_dict[left_way_list[index]]
            left_original_list[0].append(left_node_obj.x)
            left_original_list[1].append(left_node_obj.y)
            total_x_list.append(left_node_obj.x)

        for index in range(len(right_way_list)):
            right_node_obj = self.id_node_dict[right_way_list[index]]
            right_original_list[0].append(right_node_obj.x)
            right_original_list[1].append(right_node_obj.y)
            total_x_list.append(right_node_obj.x)

        left_way_func = interpolate.interp1d(x=left_original_list[0],
                                             y=left_original_list[1],
                                             kind="linear",
                                             fill_value="extrapolate")

        right_way_func = interpolate.interp1d(x=right_original_list[0],
                                              y=right_original_list[1],
                                              kind="linear",
                                              fill_value="extrapolate")

        final_x_list = []
        total_x_list = sorted(total_x_list)
        for index in range(len(total_x_list) - 1):
            temp_array = np.linspace(total_x_list[index], total_x_list[index + 1],
                                     num=int(np.ceil((total_x_list[index + 1] - total_x_list[index]) / interval)))
            if index == 0:
                final_x_list.extend(list(temp_array))
            else:
                final_x_list.extend(list(temp_array[1:]))

        final_left_y_list = left_way_func(final_x_list)
        fina_right_y_list = right_way_func(final_x_list)

        final_left_node_list = []
        final_right_node_list = []

        for i in range(len(final_x_list)):
            final_left_node_list.append((final_x_list[i], final_left_y_list[i]))
            final_right_node_list.append((final_x_list[i], fina_right_y_list[i]))

        return final_left_node_list, final_right_node_list

    def linear_interpolation_for_lane(self, lane_id: int):
        """
        Do linear interpolation operation for the reference nodes' coordinates of two ways of the same lane.
        """
        result = self.need_interpolation(lane_id=lane_id)

        lane_obj = self.id_lane_dict[lane_id]
        left_way_list = self.id_way_dict[lane_obj.left_way_id].ref_node_list
        right_way_list = self.id_way_dict[lane_obj.right_way_id].ref_node_list

        if not result:
            left_index = 0
            right_index = 0

            left_interp_x_list = []
            right_interp_x_list = []

            left_original_list = [[], []]
            right_original_list = [[], []]

            while left_index < len(left_way_list) and right_index < len(right_way_list):
                left_node_coord = self.id_node_dict[left_way_list[left_index]].get_coord()
                right_node_coord = self.id_node_dict[right_way_list[right_index]].get_coord()

                if abs(left_node_coord[0]-right_node_coord[0]) < self.max_interval:
                    left_index += 1
                    right_index += 1

                    left_original_list[0].append(left_node_coord[0])
                    left_original_list[1].append(left_node_coord[1])

                    right_original_list[0].append(right_node_coord[0])
                    right_original_list[1].append(right_node_coord[1])
                else:
                    if left_node_coord[0] < right_node_coord[0]:
                        right_interp_x_list.append(left_node_coord[0])
                        left_index += 1

                        left_original_list[0].append(left_node_coord[0])
                        left_original_list[1].append(left_node_coord[1])
                    else:
                        left_interp_x_list.append(right_node_coord[0])
                        right_index += 1

                        right_original_list[0].append(right_node_coord[0])
                        right_original_list[1].append(right_node_coord[1])

            final_left_node_list = self.get_final_way_list(left_original_list, left_interp_x_list)
            final_right_node_list = self.get_final_way_list(right_original_list, right_interp_x_list)
        else:
            final_left_node_list = []
            final_right_node_list = []

            for i in range(len(left_way_list)):
                final_left_node_list.append((self.id_node_dict[left_way_list[i]].x,
                                             self.id_node_dict[left_way_list[i]].y))
                final_right_node_list.append((self.id_node_dict[right_way_list[i]].x,
                                              self.id_node_dict[right_way_list[i]].y))

        return final_left_node_list, final_right_node_list

    @staticmethod
    def get_final_way_list(original_list: list, inter_x_list: list):
        """
        Get complete reference nodes list by doing interpolation operation.
        """
        final_node_list = []
        for i in range(len(original_list[0])):
            final_node_list.append((original_list[0][i], original_list[1][i]))

        if len(inter_x_list) > 0:
            inter_y_list = np.interp(inter_x_list, original_list[0], original_list[1])

            for j in range(len(inter_x_list)):
                final_node_list.append((inter_x_list[j], inter_y_list[j]))

            final_node_list = sorted(final_node_list, key=lambda x: x[0])

        return final_node_list

    def get_complete_lane(self):
        """
        Get complete lane according to the lane relationship in the .osm file.

        In .osm file, the short lanes which are owned to the same complete lane share the same start or end node
        coordinate. According to this feature, we can reconstruct complete lanes from these short lanes.
        """
        complete_lane_dict = {}
        lane_list = copy.deepcopy(list(self.id_lane_dict.keys()))

        lane_index = 0
        lane_symbol = "a"

        while len(lane_list) > 0:
            lane_id = lane_list[0]
            lane_obj = self.id_lane_dict[lane_id]

            complete_lane = self.get_complete_way(left_way_id=lane_obj.left_way_id,
                                                  right_way_id=lane_obj.right_way_id,
                                                  lane_id=lane_id,
                                                  lane_index=[lane_symbol+str(lane_index), lane_symbol+str(lane_index+1)])

            for lane in complete_lane.lane_list:
                try:
                    lane_list.remove(lane)
                except ValueError:
                    continue

            complete_lane_dict[lane_index] = complete_lane
            lane_index += 2

        return complete_lane_dict

    def get_complete_way(self, left_way_id: int, right_way_id: int, lane_id: int, lane_index: list):
        """
        Get complete way starting from start left_way_id and right_way_id.
        """
        left_way_list = self.id_way_dict[left_way_id].ref_node_list
        right_way_list = self.id_way_dict[right_way_id].ref_node_list

        # <=== left_way_id
        # <=== right_way_id
        left_complete_lane_list, left_complete_way_list = self.get_half_lane(start_left_node=left_way_list[0],
                                                                             start_right_node=right_way_list[0],
                                                                             left_way_id=left_way_id,
                                                                             right_way_id=right_way_id,
                                                                             lane_id=lane_id,
                                                                             direction="Left")
        # left_way_id ===>
        # right_way_id ===>
        right_complete_lane_list, right_complete_way_list = self.get_half_lane(start_left_node=left_way_list[-1],
                                                                               start_right_node=right_way_list[-1],
                                                                               left_way_id=left_way_id,
                                                                               right_way_id=right_way_id,
                                                                               lane_id=lane_id,
                                                                               direction="Right")

        complete_lane = CompleteLane(start_str=lane_index[0], end_str=lane_index[1])
        complete_lane.lane_list = left_complete_lane_list + right_complete_lane_list[1:]
        complete_lane.left_way_list = left_complete_way_list[0] + right_complete_way_list[0][1:]
        complete_lane.right_way_list = left_complete_way_list[1] + right_complete_way_list[1][1:]

        complete_lane.left_way_type_list = complete_lane.get_way_type_list(self.id_way_dict, "left")
        complete_lane.right_way_type_list = complete_lane.get_way_type_list(self.id_way_dict, "right")

        return complete_lane

    def get_half_lane(self, start_left_node: int, start_right_node: int, left_way_id: int, right_way_id: int,
                      lane_id: int, direction: str = "Left"):
        """
        Get left or right half lane according to the given short lane.
        """
        lane_list = [lane_id]

        way_list = [[left_way_id], [right_way_id]]

        list_index = -1
        insert_pos = 0

        if direction != "Left" and direction != "left" and direction != "L":
            list_index = 0
            insert_pos = -1

        while True:
            left_way_list = self.id_node_dict[start_left_node].way_list

            do_loop = False

            for way_id in left_way_list:
                if way_id == left_way_id:
                    continue
                else:
                    left_lane_list = self.id_way_dict[way_id].lane_list

                    for l_id in left_lane_list:
                        if l_id == lane_id:
                            continue
                        else:
                            neighbor_lane_obj = self.id_lane_dict[l_id]

                            neighbor_left_node_list = self.id_way_dict[neighbor_lane_obj.left_way_id].ref_node_list
                            neighbor_right_node_list = self.id_way_dict[neighbor_lane_obj.right_way_id].ref_node_list

                            if neighbor_left_node_list[list_index] == start_left_node:
                                if neighbor_right_node_list[list_index] == start_right_node:
                                    if direction == "Left" or direction == "left" or direction == "L":
                                        lane_list.insert(insert_pos, l_id)
                                        way_list[0].insert(insert_pos, neighbor_lane_obj.left_way_id)
                                        way_list[1].insert(insert_pos, neighbor_lane_obj.right_way_id)
                                    else:
                                        lane_list.append(l_id)
                                        way_list[0].append(neighbor_lane_obj.left_way_id)
                                        way_list[1].append(neighbor_lane_obj.right_way_id)

                                    start_left_node = neighbor_left_node_list[insert_pos]
                                    start_right_node = neighbor_right_node_list[insert_pos]

                                    left_way_id = neighbor_lane_obj.left_way_id
                                    right_way_id = neighbor_lane_obj.right_way_id
                                    lane_id = neighbor_lane_obj.id

                                    do_loop = True
                            elif neighbor_right_node_list[list_index] == start_left_node:
                                if neighbor_left_node_list[list_index] == start_right_node:
                                    if direction == "Left" or direction == "left" or direction == "L":
                                        lane_list.insert(insert_pos, l_id)
                                        way_list[0].insert(insert_pos, neighbor_lane_obj.right_way_id)
                                        way_list[1].insert(insert_pos, neighbor_lane_obj.left_way_id)
                                    else:
                                        lane_list.append(l_id)
                                        way_list[0].append(neighbor_lane_obj.right_way_id)
                                        way_list[1].append(neighbor_lane_obj.left_way_id)

                                    start_left_node = neighbor_right_node_list[insert_pos]
                                    start_right_node = neighbor_left_node_list[insert_pos]

                                    left_way_id = neighbor_lane_obj.right_way_id
                                    right_way_id = neighbor_lane_obj.left_way_id
                                    lane_id = neighbor_lane_obj.id

                                    do_loop = True
            if not do_loop:
                break
        return lane_list, way_list

    def complete_lane_to_draw_lane(self) -> dict:
        """
        Convert CompleterLane object to DrawLane object.
        """
        draw_lane_dict = {}
        for complete_lane_index, complete_lane_obj in self.complete_lane_dict.items():
            draw_lane_obj = DrawLane(start_str=complete_lane_obj.start_str,
                                     end_str=complete_lane_obj.end_str)

            # key_tuple = (complete_lane_obj.start_str, complete_lane_obj.end_str)
            # draw_lane_obj.lane_index_dict[key_tuple] = complete_lane_index
            draw_lane_obj.id = complete_lane_index

            key_index = 0
            for index in range(len(complete_lane_obj.lane_list)):
                single_lane_id = complete_lane_obj.lane_list[index]
                left_way_list, right_way_list = self.linear_interpolation_for_lane(single_lane_id)
                # print(left_way_list)
                # left_way_list, right_way_list = self.linear_interpolation_for_lane_with_small_interval(single_lane_id, interval=1)
                # print(left_way_list)
                # print("**************************************")

                if complete_lane_obj.left_way_list[index] == self.id_lane_dict[single_lane_id].left_way_id:
                    left_way_type = "solid" if complete_lane_obj.left_way_type_list[index] == "solid" else "dashed"
                    right_way_type = "solid" if complete_lane_obj.right_way_type_list[index] == "solid" else "dashed"
                else:
                    left_way_type = "solid" if complete_lane_obj.right_way_type_list[index] == "solid" else "dashed"
                    right_way_type = "solid" if complete_lane_obj.left_way_type_list[index] == "solid" else "dashed"

                    temp_way_list = copy.deepcopy(left_way_list)
                    left_way_list = copy.deepcopy(right_way_list)
                    right_way_list = copy.deepcopy(temp_way_list)

                if left_way_list[0][1] < right_way_list[0][1] or left_way_list[-1][1] < right_way_list[-1][1]:
                    temp_list = copy.deepcopy(left_way_list)
                    left_way_list = copy.deepcopy(right_way_list)
                    right_way_list = copy.deepcopy(temp_list)

                    temp_type_list = copy.deepcopy(left_way_type)
                    left_way_type = copy.deepcopy(right_way_type)
                    right_way_type = copy.deepcopy(temp_type_list)

                for i in range(len(left_way_list)-1):
                    coordinate_list = [[left_way_list[i], left_way_list[i+1]], [right_way_list[i], right_way_list[i+1]]]

                    start_node_coord = ((left_way_list[i][0]+right_way_list[i][0])/2,
                                        (left_way_list[i][1]+right_way_list[i][1])/2)

                    end_node_coord = ((left_way_list[i+1][0]+right_way_list[i+1][0])/2,
                                      (left_way_list[i+1][1]+right_way_list[i+1][1])/2)

                    width = self.point_to_line_distance(point=left_way_list[i],
                                                        line_point1=right_way_list[i],
                                                        line_point2=right_way_list[i+1])

                    width += self.point_to_line_distance(point=left_way_list[i+1],
                                                         line_point1=right_way_list[i],
                                                         line_point2=right_way_list[i+1])

                    draw_lane_obj.index_coord_dict[key_index] = coordinate_list
                    draw_lane_obj.index_type_dict[key_index] = (left_way_type, right_way_type)
                    draw_lane_obj.index_center_dict[key_index] = (start_node_coord, end_node_coord)
                    draw_lane_obj.index_width_dict[key_index] = width/2

                    key_index += 1

            draw_lane_dict[complete_lane_index] = draw_lane_obj

        return draw_lane_dict

    def position_to_lane(self, position: tuple):
        """
        Get the lane index according to the position of the vehicle.
        """
        for dict_id, draw_lane_obj in self.draw_lane_dict.items():
            for index, coord in draw_lane_obj.index_coord_dict.items():
                if self.position_in_rectangle(p=position,
                                              a=coord[0][0],
                                              b=coord[1][0],
                                              c=coord[0][1],
                                              d=coord[1][1]):
                    return [dict_id, draw_lane_obj.start_str, draw_lane_obj.end_str, index]
        return None

    @staticmethod
    def point_to_line_distance(point: tuple, line_point1: tuple, line_point2: tuple):
        point = np.array(point)
        line_point1 = np.array(line_point1)
        line_point2 = np.array(line_point2)

        vec1 = line_point1 - point
        vec2 = line_point2 - point
        distance = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)
        return distance

    @staticmethod
    def position_in_rectangle(p: tuple, a: tuple, b: tuple, c: tuple, d: tuple) -> bool:
        """
        Check if a point is inside a normal rectangle.

        a-------d
        |       |
        |       |
        b-------c
        @param p: a point (x, y)
        @param a:
        @param b:
        @param c:
        @param d:
        """
        ab = [b[0] - a[0], b[1] - a[1]]
        ap = [p[0] - a[0], p[1] - a[1]]

        cd = [d[0] - c[0], d[1] - c[1]]
        cp = [p[0] - c[0], p[1] - c[1]]

        ad = [d[0] - a[0], d[1] - a[1]]
        cb = [b[0] - c[0], b[1] - c[1]]

        if np.cross(ab, ap) * np.cross(cd, cp) >= 0 and np.cross(ad, ap) * np.cross(cb, cp) >= 0:
            return True
        else:
            return False

    def draw_with_original_data_by_matplotlib(self):
        for draw_lane_id, draw_lane_obj in self.draw_lane_dict.items():
            for index, coord in draw_lane_obj.index_coord_dict.items():
                way_type = draw_lane_obj.index_type_dict[index]
                left_way_type = "-" if way_type[0] == "solid" else "--"
                right_way_type = "-" if way_type[1] == "solid" else "--"
                left_color = "black" if left_way_type == "-" else "orange"
                right_color = "black" if right_way_type == "-" else "orange"
                plt.plot([coord[0][0][0], coord[0][1][0]],
                         [coord[0][0][1], coord[0][1][1]],
                         color=left_color,
                         linestyle=left_way_type)
                plt.plot([coord[1][0][0], coord[1][1][0]],
                         [coord[1][0][1], coord[1][1][1]],
                         color=right_color,
                         linestyle=right_way_type)

        plt.savefig("/home/joe/Desktop/DR_CHN_Merging_ZS0_small.pdf")
        # plt.show()

    def draw_with_center_by_matplotlib(self):
        for draw_lane_id, draw_lane_obj in self.draw_lane_dict.items():
            for index, center in draw_lane_obj.index_center_dict.items():
                width = draw_lane_obj.index_width_dict[index]
                way_type = draw_lane_obj.index_type_dict[index]
                left_way_type = "-" if way_type[0] == "solid" else "--"
                right_way_type = "-" if way_type[1] == "solid" else "--"
                left_color = "black" if left_way_type == "-" else "orange"
                right_color = "black" if right_way_type == "-" else "orange"

                plt.plot([center[0][0]+width/2, center[1][0]+width/2],
                         [center[0][1]+width/2, center[1][1]+width/2],
                         color=left_color,
                         linestyle=left_way_type)
                plt.plot([center[0][0]-width/2, center[1][0]-width/2],
                         [center[0][1]-width/2, center[1][1]-width/2],
                         color=right_color,
                         linestyle=right_way_type)

        plt.savefig("/home/joe/Desktop/DR_CHN_Merging_ZS0_center.pdf")

    def get_draw_lane_info(self):
        lane_start_draw_dict = {}
        for draw_lane_id, draw_lane_obj in self.draw_lane_dict.items():
            coord = draw_lane_obj.index_coord_dict[len(draw_lane_obj.index_coord_dict)-1]
            left_y = int(coord[0][0][1]*10)
            right_y = int(coord[1][0][1]*10)

            left_draw = False
            right_draw = False

            if left_y not in lane_start_draw_dict.keys():
                lane_start_draw_dict[left_y] = True
                left_draw = True
            else:
                if not lane_start_draw_dict[left_y]:
                    left_draw = True
                    lane_start_draw_dict[left_y] = True

            if right_y not in lane_start_draw_dict.keys():
                lane_start_draw_dict[right_y] = True
                right_draw = True
            else:
                if not lane_start_draw_dict[right_y]:
                    right_draw = True
                    lane_start_draw_dict[right_y] = True


            for index, center in draw_lane_obj.index_center_dict.items():
                width = draw_lane_obj.index_width_dict[index]
                way_type = draw_lane_obj.index_type_dict[index]
                left_way_type = "-" if way_type[0] == "solid" else "--"
                right_way_type = "-" if way_type[1] == "solid" else "--"
                left_color = "black" if left_way_type == "-" else "orange"
                right_color = "black" if right_way_type == "-" else "orange"

                if left_draw:
                    plt.plot([center[0][0]+width/2, center[1][0]+width/2],
                             [center[0][1]+width/2, center[1][1]+width/2],
                             color=left_color,
                             linestyle=left_way_type)

                if right_draw:
                    plt.plot([center[0][0]-width/2, center[1][0]-width/2],
                             [center[0][1]-width/2, center[1][1]-width/2],
                             color=right_color,
                             linestyle=right_way_type)
            # plt.show()

        plt.savefig("/home/joe/Desktop/DR_CHN_Merging_ZS0_center.pdf")


if __name__ == '__main__':
    osm_file = "/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/maps/DR_CHN_Merging_ZS0.osm"
    hd_map = HDMap(osm_file_path=osm_file)
    # hd_map.draw_with_original_data_by_matplotlib()
    # hd_map.draw_with_center_by_matplotlib()
    # hd_map.get_draw_lane_info()
    import json
    for key, draw_lane_obj in hd_map.draw_lane_dict.items():
        with open("/home/joe/Desktop/coord_"+str(key)+".json", "w", encoding="UTF-8") as f:
            f.write(json.dumps(hd_map.draw_lane_dict[key].index_coord_dict))
    print("Hello World!")