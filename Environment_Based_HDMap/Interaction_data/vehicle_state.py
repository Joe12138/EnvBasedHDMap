import numpy as np


class VehicleState(object):
    def __init__(self, v_id: int, x: float, y: float, vx: float, vy: float, psi_rad: float):
        self.id = v_id
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.psi_rad = psi_rad

        self.speed = np.sqrt(np.power(self.vx, 2)+np.power(self.vy, 2))

    def __str__(self):
        return "x = {}, y = {}, vx = {}, vy = {}, psi_rad = {}".format(self.x, self.y, self.vx, self.vy, self.psi_rad)

    def __repr__(self):
        return self.__str__()