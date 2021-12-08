from road.graphics import *
import os


class EnvViewer(object):
    """
    A viewer to render a driving environment.
    """
    def __init__(self, env, offscreen=False):
        self.env = env
        self.offscreen = offscreen

        pygame.init()
        pygame.display.set_caption("Env_Based-HDMap")
        panel_size = (self.env.config["screen_width"], self.env.config["screen_height"])

        if not self.offscreen:
            self.screen = pygame.display.set_mode(panel_size)

        self.sim_surface = WorldSurface(width=panel_size[0],
                                        height=panel_size[1],
                                        min_x=env.min_x if env.min_x is not None else 950,
                                        min_y=env.min_y if env.min_y is not None else 20,
                                        max_x=env.max_x if env.max_x is not None else 1180,
                                        max_y=env.max_y if env.max_y is not None else 70)

        self.sim_surface.scaling = env.config.get("scaling", self.sim_surface.INITIAL_SCALING)
        self.sim_surface.centering_position = env.config.get("centering_position", self.sim_surface.INITIAL_CENTERING)
        self.clock = pygame.time.Clock()

        self.enabled = True
        if "SDL_VIDEODRIVER" in os.environ and os.environ["SDL_VIDEODRIVER"] == "dummy":
            self.enabled = False

        self.agent_display = None
        self.agent_surface = None
        self.vehicle_trajectory = None

        self.frame = 0

    def handle_event(self):
        """
        Handle pygame event by forwarding them to the display and environment vehicle.
        :return:
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.env.close()
            self.sim_surface.handle_event(event)

    def display(self):
        """
        Display the road and vehicles on a pygame window
        :return:
        """
        if not self.enabled:
            return

        self.sim_surface.move_display_window_to(self.window_position())
        RoadGraphics.display(self.env.road, self.sim_surface)

        RoadGraphics.display_traffic(road=self.env.road,
                                     surface=self.sim_surface,
                                     simulation_frequency=self.env.SIMULATION_FREQUENCY,
                                     offscreen=self.offscreen)

        RoadGraphics.display_trajectory(self.env.road, self.sim_surface)

        if self.agent_display:
            self.agent_display(self.agent_surface, self.sim_surface)
            if self.env.config["screen_width"] > self.env.config["screen_height"]:
                self.screen.blit(self.agent_surface, (0, self.env.config["screen_height"]))
            else:
                self.screen.blit(self.agent_surface, (self.env.config["screen_width"], 0))

        if not self.offscreen:
            self.screen.blit(self.sim_surface, (0, 0))
            self.clock.tick(self.env.SIMULATION_FREQUENCY)
            pygame.display.flip()

    def window_position(self):
        """
        :return: the world position of the center of the displayed window.
        """
        if self.env.vehicle:
            return self.env.vehicle.position
        else:
            return np.array([0, 0])

    def set_agent_display(self, agent_display):
        """
        Set a display callback provided by an agent, so that they can render their behaviour on a dedicated agent
        surface, or even on the simulation surface.

        :param agent_display: a callback provided by the agent to display on surfaces.
        :return:
        """
        if self.agent_display is None:
            width = self.env.config["screen_width"]
            height = self.env.config["screen_height"]
            if width > height:
                self.screen = pygame.display.set_mode((width, 2*height))
            else:
                self.screen = pygame.display.set_mode((2*width, height))
        self.agent_display = agent_display

    def set_agent_action_sequence(self, action):
        """
        Set the sequence of actions chosen by the agent, so that it can be displayed.

        :param action: list of action, following the action specification of the env
        :return:
        """
        pass