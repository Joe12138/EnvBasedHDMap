import csv
from Interaction_data.vehicle_state import VehicleState
from Interaction_data.snap_shot import SnapShot
from Interaction_data.trajectory import Trajectory
import numpy as np


class InteractionDataset(object):
    def __init__(self, dataset_file_path: str):
        self.data_file_path = dataset_file_path

        # Dict: key = (case_id, vehicle_id), value = Trajectory object
        self.id_traj_dict = {}

        # Dict: key = (case_id, frame_id), value = SnapShot object
        self.id_snapshot_dict = {}
        self.load_dataset()

    def load_dataset(self):
        print("---------------Loading Dataset...-----------------")

        with open(self.data_file_path, "r", encoding="UTF-8") as f:
            f_csv = csv.reader(f)
            row_index = 0

            for row in f_csv:
                # print("{}/{}".format(row_index, f_csv.__sizeof__()))
                if row_index == 0:
                    row_index += 1
                else:
                    case_id = int(float(row[0]))
                    track_id = int(row[1])
                    frame_id = int(row[2])

                    agent_type = str(row[4])

                    if agent_type != "car":
                        print("Wrong agent type! The type is ({})".format(agent_type))
                        continue

                    # For testing
                    if case_id != 1:
                        break

                    x = float(row[5])
                    y = 1000-float(row[6])
                    vx = float(row[7])
                    vy = float(row[8])
                    psi_rad = -float(row[9])
                    length = float(row[10])
                    width = float(row[11])

                    traj_key = (case_id, track_id)
                    snap_key = (case_id, frame_id)

                    if traj_key not in self.id_traj_dict.keys():
                        self.id_traj_dict[traj_key] = Trajectory(case_id=case_id, track_id=track_id, v_length=length,
                                                                 v_width=width)
                    vehicle = VehicleState(v_id=track_id, x=x, y=y, vx=vx, vy=vy, psi_rad=psi_rad)
                    self.id_traj_dict[traj_key].time_state_dict[frame_id] = vehicle

                    if snap_key not in self.id_snapshot_dict.keys():
                        self.id_snapshot_dict[snap_key] = SnapShot(case_id=case_id, frame_id=frame_id)

                    self.id_snapshot_dict[snap_key].vehicle_object_list.append(vehicle)

                    row_index += 1

        for k, v in self.id_traj_dict.items():
            v.get_trajectory()

        print("----------------Loading Dataset Complete!------------------------")

    def get_neighbour_vehicle(self, ego_vehicle_obj: VehicleState, dist_threshold: float):
        for key, value in self.id_snapshot_dict.items():
            value.neighbor_vehicle_list = value.get_neighbor_vehicles(ego_vehicle_obj=ego_vehicle_obj,
                                                                      dist_threshold=dist_threshold)


if __name__ == '__main__':
    file_path = "/home/joe/Dataset/Interaction/INTERACTION-Dataset-DR-single-v1_2/train/DR_CHN_Merging_ZS0_train.csv"
    interaction_dataset = InteractionDataset(dataset_file_path=file_path)
    print(len(interaction_dataset.id_traj_dict))