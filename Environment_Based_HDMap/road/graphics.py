import copy
import time

import pygame
from road.lane import Lane
from road.road_network import RoadNetwork, Road
import numpy as np
from osm_parser.hd_map import HDMap
from vehicle.graphics import VehicleGraphics
from Interaction_data.data_process import InteractionDataset
from vehicle.human_driving import InteractionVehicle, HumanLikeVehicle


class Color(object):
    BLACK = (0, 0, 0)
    GREY = (100, 100, 100)
    GREEN = (50, 200, 0)
    YELLOW = (200, 200, 0)
    WHITE = (255, 255, 255)
    RED = (200, 50, 0)


class WorldSurface(pygame.Surface):
    """
    A pygame Surface implementing a local system so that we can move and zoom in the displayed area.
    """

    INITIAL_SCALING = 10.0
    INITIAL_CENTERING = [0.5, 0.5]
    SCALING_FACTOR = 5
    MOVING_FACTOR = 0.1

    def __init__(self, width: int, height: int, min_x: float, min_y: float, max_x: float, max_y: float):
        super(WorldSurface, self).__init__((width, height))
        self.scaling = min(self.INITIAL_SCALING, min(width/(max_x-min_x), height/(max_y-min_y)))
        self.scaling = 3
        self.origin = np.array([min_x, min_y])
        self.centering_position = self.INITIAL_CENTERING

    def dist2pix(self, length: float):
        """
        Convert a distance [m] to pixels [px]
        :param length: the input distance [m]
        :return: the corresponding size [px]
        """
        return int(length*self.scaling)

    def pos2pix(self, x: float, y: float):
        """
        Convert two world coordinates [m] into a position in the surface [px]
        :param x: x world coordinate [m]
        :param y: y world coordinate [m]
        :return: the coordinates of the corresponding pixel [px]
        """
        return self.dist2pix(x-self.origin[0]), self.dist2pix(y-self.origin[1])

    def move_display_window_to(self, position):
        """
        Set the origin of the displayed area to center on a given world position.

        :param position: a world position [m]
        :return: No return object
        """
        self.origin = position-np.array([
            self.centering_position[0]*self.get_width()/self.scaling,
            self.centering_position[1]*self.get_height()/self.scaling
        ])

    def handle_event(self, event):
        """
        Handle pygame events for moving and zooming in the displayed area.

        :param event: a pygame event
        """
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_l:
                self.scaling *= 3 / self.SCALING_FACTOR
            if event.key == pygame.K_o:
                self.scaling *= self.SCALING_FACTOR/3
            if event.key == pygame.K_a:
                self.centering_position[0] -= self.MOVING_FACTOR
            if event.key == pygame.K_d:
                self.centering_position[0] += self.MOVING_FACTOR
            if event.key == pygame.K_w:
                self.centering_position[1] -= self.MOVING_FACTOR
            if event.key == pygame.K_s:
                self.centering_position[1] += self.MOVING_FACTOR

        if event.type == pygame.MOUSEWHEEL:
            if event.y == 1:
                self.scaling *= self.SCALING_FACTOR/3
            if event.y == -1:
                self.scaling *= 3 / self.SCALING_FACTOR


class LaneGraphics(object):
    """
    A visualization of a lane.
    """
    stripe_spacing = 3  # Offset between stripes [m]
    stripe_length = 1  # Length of a stripe [m]
    stripe_width = 0.1  # Width of a stripe [m]

    @classmethod
    def display(cls, lane: Lane, world_surface: WorldSurface):
        """
        Display a lane on a surface
        :param lane: the lane to be displayed
        :param surface: the pygame surface
        :return: no return object
        """
        left_way_list = lane.left_way_list
        right_way_list = lane.right_way_list

        left_way_type = lane.left_way_type
        right_way_type = lane.right_Way_type

        cls.draw_line(world_surface=world_surface, start_pos=left_way_list[0], end_pos=left_way_list[1],
                      line_type=left_way_type)
        cls.draw_line(world_surface=world_surface, start_pos=right_way_list[0], end_pos=right_way_list[1],
                      line_type=right_way_type)

        cls.draw_lane(world_surface=world_surface, lane=lane)

    @classmethod
    def draw_line(cls, world_surface: WorldSurface, start_pos: tuple, end_pos: tuple, line_type: str):
        pygame.draw.aaline(surface=world_surface,
                           color=Color.WHITE if line_type == "solid" else Color.YELLOW,
                           start_pos=world_surface.pos2pix(x=start_pos[0], y=start_pos[1]),
                           end_pos=world_surface.pos2pix(x=end_pos[0], y=end_pos[1]),
                           blend=2)

    @classmethod
    def draw_lane(cls, world_surface: WorldSurface, lane: Lane):
        point_list = []
        for node in lane.left_way_list:
            point_list.append(world_surface.pos2pix(node[0], node[1]))

        right_way_list = copy.deepcopy(lane.right_way_list)
        right_way_list.reverse()
        for node in right_way_list:
            point_list.append(world_surface.pos2pix(node[0], node[1]))

        pygame.draw.polygon(surface=world_surface, color=Color.GREY, points=point_list, width=0)


class RoadGraphics(object):
    """
    A visualization of a road lanes and vehicles.
    """
    @classmethod
    def display(cls, road: Road, surface: WorldSurface):
        """
        Display the road lanes on a surface.

        :param road: the road to be displayed
        :param surface: the pygame surface
        :return:
        """
        surface.fill(Color.BLACK)

        for _from in road.network.graph.keys():
            for _to in road.network.graph[_from].keys():
                for l in road.network.graph[_from][_to]:
                    LaneGraphics.display(lane=l, world_surface=surface)

    @classmethod
    def display_traffic(cls, road, surface, simulation_frequency=15, offscreen=False):
        """
        Display the road vehicles on a surface.
        :param road: the road to be display
        :param surface: the pygame surface
        :param simulation_frequency: simulation frequency
        :param offscreen:
        :return:
        """
        for v in road.vehicles:
            VehicleGraphics.display(v, surface, offscreen=offscreen)

    @classmethod
    def display_trajectory(cls, road, surface):
        for v in road.vehicles:
            if isinstance(v, InteractionVehicle):
                VehicleGraphics.display_interaction_trajectory(surface, v)
            else:
                VehicleGraphics.display_planned_trajectory(surface, road.vehicles[0])


if __name__ == '__main__':
    file_name = "DR_CHN_Merging_ZS0"
    osm_file_path = "/home/joe/Prediction/Dataset/interaction-dataset/maps/" + file_name + ".osm"

    file_path = "/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/train/" + file_name + "_train.csv"

    interaction_dataset = InteractionDataset(dataset_file_path=file_path)
    hd_map = HDMap(osm_file_path=osm_file_path)

    pygame.init()
    pygame.display.set_caption("Env-Based-HDMap")
    screen = pygame.display.set_mode((1920, 1080))

    time_step = 1
    clock = pygame.time.Clock()

    world_surface = WorldSurface(width=1920,
                                 height=1080,
                                 min_x=950,
                                 min_y=20,
                                 max_x=1180,
                                 max_y=70)

    road_network = RoadNetwork()
    for draw_lane_index, draw_lane_obj in hd_map.draw_lane_dict.items():
        for key, coord in draw_lane_obj.index_coord_dict.items():
            left_way_list = coord[0]
            right_way_list = coord[1]

            way_type = draw_lane_obj.index_type_dict[key]

            lane = Lane(left_way_list=left_way_list,
                        right_way_list=right_way_list,
                        left_way_type=way_type[0],
                        right_way_type=way_type[1],
                        draw_lane_id=0,
                        index=0)
            road_network.add_lane(draw_lane_obj.start_str, draw_lane_obj.end_str, lane)

    road = Road(network=road_network)

    vehicles = []

    snap_shot_obj = interaction_dataset.id_snapshot_dict[(1, 1)]

    for v in snap_shot_obj.vehicle_object_list:
        interaction_vehicle = InteractionVehicle(road=road,
                                                 position=(v.x, v.y),
                                                 heading=v.psi_rad,
                                                 velocity=v.speed)
        vehicles.append(interaction_vehicle)
        print("v_id = {}, position = {}, heading = {}".format(v.id, (v.x, v.y), v.psi_rad))

    road.vehicles = vehicles

    while True:

        # for draw_lane_index, draw_lane_obj in hd_map.draw_lane_dict.items():
        #     for key, coord in draw_lane_obj.index_coord_dict.items():
        #         left_way_list = coord[0]
        #         right_way_list = coord[1]
        #
        #         way_type = draw_lane_obj.index_type_dict[key]
        #
        #         lane = Lane(left_way_list=left_way_list,
        #                     right_way_list=right_way_list,
        #                     left_way_type=way_type[0],
        #                     right_way_type=way_type[1])
        #
        #         LaneGraphics.display(lane=lane, world_surface=world_surface)
        world_surface.move_display_window_to(vehicles[0].position)
        RoadGraphics.display(road, world_surface)
        RoadGraphics.display_traffic(road, world_surface, offscreen=True)

        screen.blit(world_surface, (0, 0))

        pygame.image.save(world_surface, "/home/joe/Desktop/test.png")
        pygame.display.flip()
        clock.tick(20)

        for event in pygame.event.get():
            world_surface.handle_event(event)

        # break
