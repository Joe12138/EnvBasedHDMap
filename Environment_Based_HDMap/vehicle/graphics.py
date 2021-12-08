import itertools
import pygame
import numpy as np

from vehicle.human_driving import InteractionVehicle, HumanLikeVehicle
from vehicle.behavior import IDMVehicle
from vehicle.dynamics import Obstacle


class VehicleGraphics(object):
    RED = (255, 100, 100)
    GREEN = (50, 200, 0)
    BLUE = (100, 200, 255)
    YELLOW = (200, 200, 0)
    BLACK = (60, 60, 60)
    PURPLE = (200, 0, 150)
    DEFAULT_COLOR = YELLOW
    EGO_COLOR = GREEN

    @classmethod
    def display(cls, vehicle, surface, his_length=4, his_width=2, transparent=False, offscreen=False):
        """
        Display a vehicle on a pygame surface.

        The vehicle is represented as a colored rotated rectangle.
        :param vehicle: the vehicle to be drawn
        :param surface: the surface to draw the vehicle on
        :param his_length: the default length of the vehicle
        :param his_width: the default width of the vehicle
        :param transparent: whether the vehicle should be drawn slight transparent
        :param offscreen: whether the rendering should be done offscreen or not
        :return:
        """
        v = vehicle
        if v.LENGTH:
            s = pygame.Surface((surface.dist2pix(v.LENGTH), surface.dist2pix(v.LENGTH)), pygame.SRCALPHA)
            rect = pygame.Rect(0, surface.dist2pix(v.LENGTH)/2-surface.dist2pix(v.WIDTH)/2,
                               surface.dist2pix(v.LENGTH), surface.dist2pix(v.WIDTH))
        else:
            v_length, v_width = his_length, his_width
            s = pygame.Surface((surface.dist2pix(v_length), surface.dist2pix(v_length)), pygame.SRCALPHA)
            rect = pygame.Rect(0, surface.dist2pix(v_length) / 2 - surface.dist2pix(v_width) / 2,
                               surface.dist2pix(v_length), surface.dist2pix(v_width))

        pygame.draw.rect(s, cls.get_color(v, transparent), rect, width=0, border_radius=5)

        h = v.heading if abs(v.heading) > 2*np.pi/180 else 0

        sr = pygame.transform.rotate(s, -h*180/np.pi)
        # print("vehicle position = {}, heading = {}".format(v.position, -h*180/np.pi))
        # print(-h*180/np.pi)

        if v.LENGTH:
            surface.blit(sr, (surface.pos2pix(v.position[0] - v.LENGTH / 2, v.position[1] - v.LENGTH / 2)))
        else:
            surface.blit(sr, (surface.pos2pix(v.position[0] - v_length / 2, v.position[1] - v_length / 2)))

    @classmethod
    def display_trajectory(cls, states, surface, offscreen=False):
        """
        Display the whole trajectory of a vehicle on a pygame surface.

        :param states: the list of vehicle state within the trajectory to be display
        :param surface: the surface to draw the vehicle future states on
        :param offscreen: whether the rendering should be done offscreen or not
        :return:
        """
        for vehicle in states:
            cls.display(vehicle, surface, transparent=True, offscreen=offscreen)

    @classmethod
    def display_interaction_trajectory(cls, surface, vehicle):
        if isinstance(vehicle, InteractionVehicle) and not vehicle.overtaken and vehicle.appear:
            trajectories = vehicle.interaction_traj[vehicle.sim_steps:, 0:2]
            points = []

            for i in range(trajectories.shape[0]):
                if trajectories[i][0] >= 0.1:
                    point = surface.pos2pix(trajectories[i][0], trajectories[i][1])
                    pygame.draw.circle(surface, cls.GREEN, point, 2)
                    points.append(point)
                else:
                    break
            if len(points) >= 2:
                pygame.draw.lines(surface, cls.GREEN, False, points)

    @classmethod
    def display_planned_trajectory(cls, surface, vehicle):
        if isinstance(vehicle, HumanLikeVehicle) and not vehicle.human:
            trajectory = vehicle.planned_trajectory
            points = []
            if trajectory is not None:
                for i in range(trajectory.shape[0]):
                    point = surface.pos2pix(trajectory[i][0], trajectory[i][1])
                    pygame.draw.circle(surface, cls.RED, point, 2)
                    points.append(point)

                pygame.draw.lines(surface, cls.RED, False, points)

    @classmethod
    def get_color(cls, vehicle, transparent=False):
        color = cls.DEFAULT_COLOR
        if getattr(vehicle, "color", None):
            color = vehicle.color
        elif vehicle.crashed:
            color = cls.RED
        elif isinstance(vehicle, InteractionVehicle) and not vehicle.overtaken:
            color = cls.BLUE
        elif isinstance(vehicle, InteractionVehicle) and vehicle.overtaken:
            color = cls.YELLOW
        elif isinstance(vehicle, HumanLikeVehicle):
            color = cls.EGO_COLOR
        elif isinstance(vehicle, IDMVehicle):
            color = cls.YELLOW
        elif isinstance(vehicle, Obstacle):
            color = cls.GREEN

        if transparent:
            color = (color[0], color[1], color[2], 30)

        return color