from Interaction_data.vehicle_state import VehicleState
import numpy as np


class SnapShot(object):
    def __init__(self, case_id: int, frame_id: int):
        self.case_id = case_id
        self.frame_id = frame_id

        self.vehicle_object_list = []
        self.neighbor_vehicle_list = None

    def get_neighbor_vehicles(self, ego_vehicle_obj: VehicleState, dist_threshold: float):
        """
        Get neighbor vehicles around the ego vehicle object within a distance threshold.
        """
        neighbor_v_list = []
        for v in self.vehicle_object_list:
            distance = np.linalg.norm([ego_vehicle_obj.x-v.x, ego_vehicle_obj.y-v.y])

            if distance <= dist_threshold:
                neighbor_v_list.append(v)

        return neighbor_v_list