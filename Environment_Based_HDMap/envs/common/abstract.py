import gym
from gym.utils import seeding

from vehicle.control import MDPVehicle
from envs.common.observation import observation_factory
from envs.common.graphics import EnvViewer


class AbstractEnv(gym.Env):
    """
    A generic environment for various tasks involving a vehicle driving on a road.

    The environment contains a road populated with vehicles, and a controlled ego-vehicle that can change lane and
    velocity. The action space is fixed, but the observation space and reward function must be defined in the
    environment implementations.
    """
    metadata = {"render.modes": ["human", "rgb_array"]}

    # A mapping of action indexes to action labels
    ACTIONS = {
        0: "LANE_LEFT",
        1: "IDLE",
        2: "LANE_RIGHT",
        3: "FASTER",
        4: "SLOWER"
    }

    # A mapping of action label to action indexes
    ACTIONS_INDEXES = {v: k for k, v in ACTIONS.items()}

    # The frequency at which the system dynamics are simulated [Hz]
    SIMULATION_FREQUENCY = 10

    # The maximum distance of any vehicle present in the observation [m]
    PERCEPTION_DISTANCE = 6.0 * MDPVehicle.SPEED_MAX

    def __init__(self, config=None):
        # Configuration
        self.config = self.default_config()
        if config:
            self.config.update(config)

        # seeding
        self.np_random = None
        self.seed()

        # Scene
        self.road = None
        self.vehicle = None

        # Spaces
        self.observation = None
        self.action_space = None
        self.observation_space = None
        self.define_spaces()

        # Running
        self.time = 0  # Simulation time
        self.steps = 0  # Actions performed
        self.done = False

        # Rendering
        self.viewer = None
        self.automatic_rendering_callback = None
        self.should_update_rendering = True
        self.rendering_mode = "human"
        self.offscreen = self.config.get("offscreen_rendering", False)
        self.enable_auto_render = False

        self.reset()

    @classmethod
    def default_config(cls):
        """
        Default environment configuration.

        Can be overloaded in environment inplemnetations, or by calling configure()
        :return: a configuration dict
        """
        return {
            "observation": {"type": "TimeToCollision"},
            "policy_frequency": 1,
            "other_vehicles_type": "vehicle.behavior.IDMVehicle",
            "screen_width": 640,
            "screen_height": 320,
            "centering_position": [0.5, 0.5],
            "show_trajectories": False

        }

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)

        return [seed]

    def configure(self, config):
        if config:
            self.config.update(config)

    def define_spaces(self):
        self.action_space = gym.spaces.Discrete(len(self.ACTIONS))

        if "observation" not in self.config:
            raise ValueError("The observation configuration must be defined!")
        self.observation = observation_factory(self, self.config["observation"])
        self.observation_space = self.observation.space()

    def reset(self):
        """
        Reset the environment to it's initial configuration
        :return: the observation of the reset state
        """
        self.time = 0
        self.done = False
        self.define_spaces()

        return self.observation.observe()

    def _is_terminal(self):
        """
        Check whether the current state is a terminal state
        :return:is the state terminal
        """
        raise NotImplementedError

    def step(self, action):
        """
        Perform an action and step the environment dynamics.

        The action is executed by the ego-vehicle, and all other vehicles on the road performs their default behaviour
        for several simulation time steps until the next decision-making step.
        :param action: the action performed by the ego-vehicle
        :return: a tuple (observation, reward, terminal, info)
        """
        if self.road is None or self.vehicle is None:
            raise NotImplementedError("The road and vehicle must be initialized in the environment implementation!")

    def _simulate(self, action=None):
        """
        Perform several steps of simulation with constant action
        """
        for _ in range(int(self.SIMULATION_FREQUENCY // self.config["policy_frequency"])):
            if action is not None and self.time % int(self.SIMULATION_FREQUENCY // self.config["policy_frequency"]) == 0:
                # Forward action to the vehicle
                self.vehicle.act(self.ACTIONS[action])

            self.road.act()
            self.road.step(1/self.SIMULATION_FREQUENCY)
            self.time += 1

            # Automatically render intermediate simulation steps if a viewer has been launched
            # Ignored if the rendering is done offscreen
            self._automatic_rendering()

            # Stop at terminal states
            if self.done or self._is_terminal():
                break

        self.enable_auto_render = False

    def render(self, mode='human'):
        """
        Render the environment.

        Create a viewer if none exists, and use it to render an image.
        :param mode: the rendering mode
        """
        self.rendering_mode = mode

        if self.viewer is None:
            self.viewer = EnvViewer(self, offscreen=self.offscreen)

        self.enable_auto_render = not self.offscreen

        # If the frame has already been rendered, do nothing
        if self.should_update_rendering:
            self.viewer.display()

        if mode == 'rgb_array':
            image = self.viewer.get_image()
            # if not self.viewer.offscreen:
            #    self.viewer.handle_events()
            # self.viewer.handle_events()
            return image
        # elif mode == 'human':
        #    if not self.viewer.offscreen:
        #   self.viewer.handle_events()

        self.should_update_rendering = False

    def _automatic_rendering(self):
        """
        Automatically render the intermediate frames while an action is still ongoing.
        This allows to render the whole video and not only single steps corresponding to agent decision-making.

        If a callback has been set, use it to perform the rendering. This is useful for the environment wrappers
        such as video-recording monitor that need to access these intermediate renderings.
        """
        if self.viewer is not None and self.enable_auto_render:
            self.should_update_rendering = True

            if self.automatic_rendering_callback:
                self.automatic_rendering_callback()
            else:
                self.render(self.rendering_mode)

