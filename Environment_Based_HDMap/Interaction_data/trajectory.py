class Trajectory(object):
    def __init__(self, case_id: int, track_id: int, v_length: float, v_width: float):
        self.case_id = case_id
        self.track_id = track_id
        self.v_length = v_length
        self.v_width = v_width

        # Dict: key = frame_id, value = vehicle state object
        self.time_state_dict = {}

        # element = vehicle state object
        self.trajectory = []

    def get_trajectory(self):
        time_state_list = sorted(self.time_state_dict.items(), key=lambda x: x[0])

        for element in time_state_list:
            self.trajectory.append(element[1])
