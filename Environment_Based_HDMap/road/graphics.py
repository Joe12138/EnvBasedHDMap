import pygame
from road.lane import Lane
from road.road_network import RoadNetwork
import numpy as np
from osm_parser.hd_map import HDMap


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
        super().__init__((width, height))
        self.scaling = min(self.INITIAL_SCALING, min(width/(max_x-min_x), height/(max_y-min_y)))
        self.origin = np.array([min_x, min_y])

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

    @classmethod
    def draw_line(cls, world_surface: WorldSurface, start_pos: tuple, end_pos: tuple, line_type: str):
        pygame.draw.aaline(surface=world_surface,
                           color=Color.WHITE if line_type == "solid" else Color.YELLOW,
                           start_pos=world_surface.pos2pix(x=start_pos[0], y=start_pos[1]),
                           end_pos=world_surface.pos2pix(x=end_pos[0], y=end_pos[1]),
                           blend=2)

    @classmethod
    def draw_lane(cls, world_surface: WorldSurface, left_upper_pos, width, height, heading):
        if heading == 0:
            lane_surface = pygame.Surface


if __name__ == '__main__':
    osm_file = "/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/maps/DR_CHN_Merging_ZS0.osm"
    hd_map = HDMap(osm_file_path=osm_file)

    pygame.init()
    pygame.display.set_caption("Env-Based-HDMap")
    screen = pygame.display.set_mode((1080, 300))

    world_surface = WorldSurface(width=1080,
                                 height=300,
                                 min_x=900,
                                 min_y=900,
                                 max_x=1200,
                                 max_y=1000)


    while True:
        screen.fill(color=Color.BLACK)

        for draw_lane_index, draw_lane_obj in hd_map.draw_lane_dict.items():
            for key, coord in draw_lane_obj.index_coord_dict.items():
                left_way_list = coord[0]
                right_way_list = coord[1]

                way_type = draw_lane_obj.index_type_dict[key]

                lane = Lane(left_way_list=left_way_list,
                            right_way_list=right_way_list,
                            left_way_type=way_type[0],
                            right_way_type=way_type[1])

                LaneGraphics.display(lane=lane, world_surface=world_surface)

        screen.blit(world_surface, (0, 0))
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 200)
        pygame.display.flip()
