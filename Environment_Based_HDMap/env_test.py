import os
import json
from road.road_network import RoadNetwork, Road
from osm_parser.hd_map import HDMap
from Interaction_data.data_process import InteractionDataset
import pygame
from road.graphics import WorldSurface, Lane, RoadGraphics
from vehicle.human_driving import InteractionVehicle, HumanLikeVehicle


def get_json():
    path_list = os.listdir(path="/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/maps/")

    directory_path = "/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/maps/"

    save_path = "/home/joe/Desktop/EnvBasedHDMap/Test"
    for path in path_list:
        if "_xy" not in path:
            osm_file = directory_path+path
            print(path)
            hd_map = HDMap(osm_file_path=osm_file, name=path[:-4])

            save_dir = os.path.join(save_path, path[:-4])

            if not os.path.exists(save_dir):
                os.mkdir(save_dir)

            node_dict = {}
            for key, value in hd_map.id_node_dict.items():
                node_dict[key] = value.to_json()

            with open(os.path.join(save_dir, "id_node_dict.json"), "w", encoding="UTF-8") as node:
                json.dump(node_dict, node)

            way_dict = {}
            for key, value in hd_map.id_way_dict.items():
                way_dict[key] = value.to_json()

            with open(os.path.join(save_dir, "id_way_dict.json"), "w", encoding="UTF-8") as way:
                json.dump(way_dict, way)

            lane_dict = {}
            for key, value in hd_map.id_lane_dict.items():
                lane_dict[key] = value.to_json()
            with open(os.path.join(save_dir, "id_lane_dict.json"), "w", encoding="UTF-8") as lane:
                json.dump(lane_dict, lane)

            draw_lane_dict = {}
            for key, value in hd_map.draw_lane_dict.items():
                draw_lane_dict[key] = value.to_json()
            with open(os.path.join(save_dir, "draw_lane_dict.json"), "w", encoding="UTF-8") as draw_lane:
                json.dump(draw_lane_dict, draw_lane)


if __name__ == '__main__':
    scene_name = "DR_CHN_Merging_ZS0"
    osm_file_path = "/home/joe/Prediction/Dataset/interaction-dataset/maps/" + scene_name + ".osm"
    draw_lane_dict_path = "/home/joe/Desktop/EnvBasedHDMap/Test/" + scene_name + "/draw_lane_dict.json"

    file_path = "/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/train/" + scene_name + "_train.csv"

    interaction_dataset = InteractionDataset(dataset_file_path=file_path)

    draw_lane_obj = open(draw_lane_dict_path)
    draw_lane_dict = json.load(draw_lane_obj)
    draw_lane_obj.close()

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
    for draw_lane_index, draw_lane_obj in draw_lane_dict.items():
        for key, coord in draw_lane_obj["index_coord_dict"].items():
            left_way_list = coord[0]
            right_way_list = coord[1]

            way_type = draw_lane_obj["index_type_dict"][key]

            lane = Lane(left_way_list=left_way_list,
                        right_way_list=right_way_list,
                        left_way_type=way_type[0],
                        right_way_type=way_type[1],
                        draw_lane_id=0,
                        index=0)
            road_network.add_lane(draw_lane_obj["start_str"], draw_lane_obj["end_str"], lane)

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
