import numpy as np



import copy

# ===== HIGH-LEVEL SETTINGS ============ #
# --- Main MPC Parameters --- #

class Config:

    def __init__(self):

        self.N = 10

class AlbertConfig:

    def __str__(self):
        return self.name


    def __init__(self):
        '''
        Vehicle bounds (need to be checked)
        '''

        self.name = 'Albert'

        self.length = 0.65
        self.width = 0.65
        self.com_to_back = 0.325

        self.lower_bound = dict()
        self.upper_bound = dict()

        self.lower_bound['x'] = -50.0
        self.upper_bound['x'] = 50.0

        self.lower_bound['y'] = -50.0
        self.upper_bound['y'] = 50.0


        self.lower_bound['psi'] = -10.5 * np.pi
        self.upper_bound['psi'] = +10.5 * np.pi

        self.lower_bound['v'] = 0
        self.upper_bound['v'] = 1.2

        self.lower_bound['w'] = -2.0
        self.upper_bound['w'] = 2.0

        self.lower_bound['alpha'] = -2.0  # Not correct!
        self.upper_bound['alpha'] = 2.0

        self.lower_bound['a'] = -1.0
        self.upper_bound['a'] = 1.0

        self.lower_bound['slack'] = 0
        self.upper_bound['slack'] = 10000





class SensorConfig:

    def __init__(self, max_num_other_agents, ego_map_size, map_size, ig_map_resolution, submap_resolution):

        self.SENSING_HORIZON = np.inf
        self.STATE_INFO_DICT = {
            "heading_ego_frame": {
                "dtype": np.float64,
                "size": 1,
                "bounds": [-np.pi, np.pi],
                "agent_attr": "heading_ego_frame",
            },
            "heading_global_frame": {
                "dtype": np.float64,
                "size": 1,
                "bounds": [-np.pi, np.pi],
                "agent_attr": "heading_global_frame",
            },
            "pos_global_frame": {
                "dtype": np.float64,
                "size": 2,
                "bounds": [-np.inf, np.inf],
                "agent_attr": "pos_global_frame",
            },
            "vel_global_frame": {
                "dtype": np.float64,
                "size": 2,
                "bounds": [-np.inf, np.inf],
                "agent_attr": "vel_global_frame",
            },
            "angvel_global_frame": {
                "dtype": np.float64,
                "size": 1,
                "bounds": [-np.inf, np.inf],
                "agent_attr": "angular_speed_global_frame",
            },
            "goal_global_frame": {
                "dtype": np.float64,
                "size": 2,
                "bounds": [-np.inf, np.inf],
                "agent_attr": "goal_global_frame",
            },
            "rel_goal": {
                "dtype": np.float64,
                "size": 2,
                "bounds": [-np.inf, np.inf],
                "agent_attr": "rel_goal",
            },
            "local_grid": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "GlobalMapSensor",
                "sensor_kwargs": dict(obs_type="ego_submap"),
            },
            "global_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "GlobalMapSensor",
                "module_name": "global_map_sensor",
                "sensor_kwargs": dict(obs_type="as_is"),
            },
            "explored_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "ExploreMapSensor",
                "sensor_kwargs": dict(obs_type="as_is"),
            },
            "ego_explored_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "ExploreMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map"),
            },
            "ego_global_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "GlobalMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map"),
            },
            "goal_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "GoalMapSensor",
                "sensor_kwargs": dict(obs_type="as_is"),
            },
            "ego_goal_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "GoalMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map"),
            },
            "pos_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "PosMapSensor",
                "sensor_kwargs": dict(obs_type="as_is"),
            },
            "ego_pos_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "PosMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map"),
            },
            "semantic_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "SemanticMapSensor",
                "sensor_kwargs": dict(obs_type="as_is"),
            },
            "semantic_map_rgb": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "sensor_name": "SemanticMapSensor",
                "sensor_kwargs": dict(obs_type="as_is", rgb=True),
            },
            "ego_semantic_map": {
                "dtype": np.uint8,
                "size": (ego_map_size[0], ego_map_size[1], "semantics"),
                "bounds": [0, 255],
                "sensor_name": "SemanticMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map"),
            },
            "ego_semantic_map_rgb": {
                "dtype": np.uint8,
                "size": (ego_map_size[0], ego_map_size[1], 3),
                "bounds": [0, 255],
                "sensor_name": "SemanticMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map", rgb=True),
            },
            "local_semantic_map_rgb": {
                "dtype": np.uint8,
                "size": (ego_map_size[0], ego_map_size[1], 3),
                "bounds": [0, 255],
                "sensor_name": "SemanticMapSensor",
                "sensor_kwargs": dict(obs_type="ego_submap", rgb=True),
            },
            "explored_semantic_map": {
                "dtype": np.uint8,
                "size": (
                    int(map_size[1] / submap_resolution),
                    int(map_size[0] / submap_resolution),
                    "semantics"
                ),
                "bounds": [0, 255],
                "sensor_name": "ExploreSemanticMapSensor",
                "sensor_kwargs": dict(obs_type="as_is"),
            },
            "explored_semantic_map_rgb": {
                "dtype": np.uint8,
                "size": (
                    int(map_size[1] / submap_resolution),
                    int(map_size[0] / submap_resolution),
                    3
                ),
                "bounds": [0, 255],
                "sensor_name": "ExploreSemanticMapSensor",
                "sensor_kwargs": dict(obs_type="as_is", rgb=True),
            },
            "ego_explored_semantic_map": {
                "dtype": np.uint8,
                "size": (ego_map_size[0], ego_map_size[1], "semantics"),
                "bounds": [0, 255],
                "sensor_name": "ExploreSemanticMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map"),
            },
            "ego_explored_semantic_map_rgb": {
                "dtype": np.uint8,
                "size": (ego_map_size[0], ego_map_size[1], 3),
                "bounds": [0, 255],
                "sensor_name": "ExploreSemanticMapSensor",
                "sensor_kwargs": dict(obs_type="ego_fixed_global_map", rgb=True),
            },
            "local_explored_semantic_map_rgb": {
                "dtype": np.uint8,
                "size": (ego_map_size[0], ego_map_size[1], 3),
                "bounds": [0, 255],
                "sensor_name": "ExploreSemanticMapSensor",
                "sensor_kwargs": dict(obs_type="ego_submap", rgb=True),
            },
            "target_map": {
                "dtype": np.uint8,
                "size": (map_size[0] * ig_map_resolution, map_size[1] * ig_map_resolution),
                "bounds": [-np.inf, np.inf],
                "agent_attr": "ig_model.targetMap.probMap",
            },
            "entropy_map": {
                "dtype": np.uint8,
                "size": (map_size[0] * ig_map_resolution, map_size[1] * ig_map_resolution),
                "bounds": [-np.inf, np.inf],
                "agent_attr": "ig_model.targetMap.entropyMap",
            },
            "agent_pos_map": {
                "dtype": np.uint8,
                "size": (map_size[0] * ig_map_resolution, map_size[1] * ig_map_resolution),
                "bounds": [-np.inf, np.inf],
                "agent_attr": "ig_model.agent_pos_map",
            },
            "binary_map": {
                "dtype": np.uint8,
                "size": (map_size[0] * ig_map_resolution, map_size[1] * ig_map_resolution),
                "bounds": [-np.inf, np.inf],
                "agent_attr": "ig_model.targetMap.binaryMap",
            },
            # "goal_map": {
            #     "dtype": np.uint8,
            #     "size": (MAP_SIZE[0] * IG_MAP_RESOLUTION, MAP_SIZE[1] * IG_MAP_RESOLUTION),
            #     "bounds": [-np.inf, np.inf],
            #     "agent_attr": "ig_model.targetMap.goal_map",
            # },
            "ego_entropy_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [-np.inf, np.inf],
                "agent_attr": "ig_model.targetMap.ego_map",
            },
            "ego_binary_map": {
                "dtype": np.uint8,
                "size": ego_map_size,
                "bounds": [0, 255],
                "agent_attr": "ig_model.targetMap.bin_ego_map",
            },
            # "ego_goal_map": {
            #     "dtype": np.uint8,
            #     "size": EGO_MAP_SIZE,
            #     "bounds": [0, 255],
            #     "agent_attr": "ig_model.targetMap.goal_ego_map",
            # },
            "mc_ego_binary_goal": {
                "dtype": np.uint8,
                "size": (2, ego_map_size[0], ego_map_size[1]),
                "bounds": [0, 255],
                "agent_attr": "ig_model.targetMap.mc_ego_binary_goal",
            },
            "ego_agent_states": {
                "dtype": np.float64,
                "size": 12,
                "bounds": [-np.inf, np.inf],
                "sensor_name": "EgoStatesSensor",
                "module_name": 'ego_states_sensor',
                "sensor_kwargs": None,
            },
            "other_agents_states": {
                "dtype": np.float64,
                "size": (max_num_other_agents, 5),
                "bounds": [-np.inf, np.inf],
                "sensor_name": "OtherAgentsRelativeHistoryStatesSensor",
                "module_name": "OtherAgentsRelativeHistoryStatesSensor",
                "sensor_kwargs": None,
            },
        }

class RewardConfig:

    def __init__(self, max_num_other_agents):
        self.TRAIN_SINGLE_AGENT = True
        self.FEATURE_DIM = 4 + (max_num_other_agents-1)*5

        self.REWARD_AT_GOAL = 3.0  # 2.5 reward given when agent reaches goal position
        self.REWARD_COLLISION_WITH_AGENT = (
            -10  # reward given when agent collides with another agent
        )
        self.REWARD_TIMEOUT = 0.0  # reward given for not reaching the goal
        self.REWARD_INFEASIBLE = 0.0
        self.REWARD_COLLISION_WITH_WALL = -10  # reward given when agent collides with wall
        self.REWARD_GETTING_CLOSE = (
            0.0  # reward when agent gets close to another agent (unused?)
        )
        self.REWARD_TIME_STEP = (
            -0.1
        )  # -0.1  # default reward given if none of the others apply (encourage speed)
        self.REWARD_DISTANCE_TO_GOAL = (
            1  # default reward given if none of the others apply (encourage speed)
        )
        self.REWARD_WIGGLY_BEHAVIOR = 0.0

        self.REWARD_DEADLOCKED = -0.0
        self.REWARD_SUBGOAL_INFEASIBLE = -0.0
        self.REWARD_FACTOR_DISTANCE = 0.0  # -0.1
        self.REWARD_COVERAGE = 0.0

        self.WIGGLY_BEHAVIOR_THRESHOLD = 0.0

        self.IG_REWARD_MODE = "binary"  # entropy, binary
        self.IG_REWARD_BINARY_CELL = 0.0
        self.IG_REWARD_COVERAGE_FINISHED = 0.0
        self.IG_REWARD_GOAL_CELL_FACTOR = 0.0
        self.IG_REWARD_GOAL_PENALTY = -0.0
        self.IG_REWARD_GOAL_COMPLETION = 0.0
        self.REWARD_MAX_IG = (
            0  # 6.7 4.0 # 0.2 ## binary 1.2 entropy 4.0 (w/o accumulating)
            + 13 * self.IG_REWARD_GOAL_CELL_FACTOR * self.IG_REWARD_BINARY_CELL
            + self.IG_REWARD_GOAL_COMPLETION
            + self.IG_REWARD_COVERAGE_FINISHED
            if self.IG_REWARD_MODE == "binary"
            else 0
        )
        self.REWARD_MIN_IG = self.IG_REWARD_GOAL_PENALTY
        self.REWARDS_NORMALIZE = True

class TerminationConfig:
    TERMINATE_AT_GOAL = True  # if True, episode ends when agent reaches goal
    IG_COVERAGE_TERMINATE = False
    IG_TARGETS_TERMINATE = False


class AgentConfig:
    def __init__(self):
        self.EGO_AGENT_POLICY = 'InteractiveMPCPolicy'
        self.EGO_AGENT_DYNAMICS = 'unicycle_2o_dynamics'
        #self.EGO_AGENT_POLICY = 'SocialForcesPolicy'
        #self.EGO_AGENT_DYNAMICS = 'PtMassFirstOrderDynamics'
        self.OTHER_AGENTS_POLICIES = ['SocialForcesPolicy']
        self.OTHER_AGENTS_DYNAMICS = 'pt_mass_2o_dynamics'

        self.MAX_TIME_RATIO = 100  # 6.4  # agent has this number times the straight-line-time to reach its goal before "timing out"
        self.MAX_TIME_STEPS = 64  # 64  # max number of time steps in an episode



class BasePolicyConfig:

    input_action_space_is_discrete = False
    input_action_space_discrete_subgoal_n_angles = 12
    input_action_space_discrete_subgoal_radii = [1.0]
    input_action_space_continuous_subgoal_max = 1.0
    constraints_lookahead = 3.0
    n_constraints = 10
    subgoal_egocentric = True


class MPCConfig:
    M = 4
    FORCES_N = 15
    FORCES_N_bar = FORCES_N + 2
    FORCES_NU = 3
    FORCES_NX = 40
    FORCES_TOTAL_V = FORCES_NX + FORCES_NU
    FORCES_NPAR = 62

    egoforceFactorDesired = 1.0
    egoforceFactorSocial = 1.0
    egoforceFactorObst = 0.0
    egoforceSigmaObstacle = 0
    egopref_speed = 1.2

    include_fixed_sfm = True

    BaseConfig=BasePolicyConfig()
    NEAR_GOAL_THRESHOLD = 0.2

class SFConfig:
    random_parameters = False
    forceFactorDesired = 1.0
    forceFactorSocial = 2.0
    forceFactorObstacle = 0.2
    forceSigmaObstacle = 0
    pref_speed = 1.29

    BaseConfig = BasePolicyConfig()

class ExplorationConfig:
    IG_ACCUMULATE_REWARDS = False
    SEMANTIC_MAP_ACTIVE = False


class EnvConfig:


    MAX_NUM_AGENTS_IN_ENVIRONMENT = 4

    RewardConfig = RewardConfig(MAX_NUM_AGENTS_IN_ENVIRONMENT)
    ExplorationConfig = ExplorationConfig()
    TerminationConfig = TerminationConfig()
    MPCConfig = MPCConfig()
    SFConfig = SFConfig()
    RobotConfig = AlbertConfig()

    terminate = 'all_agents_done'

    DT = 0.15  # seconds between simulation time steps
    AgentConfig = AgentConfig()
    COLLISION_DIST = 0.5  # meters between agents' boundaries for collision
    GETTING_CLOSE_RANGE = 0.2  # meters between agents' boundaries for collision

    SCENARIOS_FOR_TRAINING = [
        {"env": "empty_map", "agents": "train_agents_swap_circle"}
    ]
    collision_check_radius_bound = 0.01

    MAP_HEIGHT = 10
    REPEAT_STEPS = 1
    continuous, discrete = range(2)  # Initialize game types as enum
    ACTION_SPACE_TYPE = discrete
    radius = 0.25
    # Map
    PNG_PATH = "/home/luzia/gompc_ws/src/learning-guided-mpc/vendor/gym-navigation2d/gym_navigation2d/src/maps/png_files/square.png"
    MAP_SIZE = (10, 10)  # meters
    PLT_LIMITS = (
        (-MAP_SIZE[0] / 2, MAP_SIZE[0] / 2),
        (-MAP_SIZE[1] / 2, MAP_SIZE[1] / 2),
    )
    PLT_FIG_SIZE = (12, 8)
    ANIMATION_PERIOD_STEPS = 5 # plot every n-th DT step (if animate mode on)
    PLOT_EVERY_N_EPISODES= 1
    GLOBAL_SEMANTICS = "SemanticsRooms"
    SUBMAP_LOOKAHEAD = 3.0  # meters

    NEAR_GOAL_THRESHOLD = 0.1
    COLLISION_AV_W_STATIC_AGENT = False
    USE_MPC_EXPERT_IN_TEST = False

    STATES_IN_OBS = [
        "ego_agent_states",
        "other_agents_states",
        "global_map",
        # "heading_global_frame",
        # "angvel_global_frame",
        # "pos_global_frame",
        # "vel_global_frame",
        # "goal_global_frame",
        # "rel_goal",
        # "ego_binary_map",
        # "ego_explored_map",
        # "ego_global_map",
        # "ego_goal_map",
        # "global_map",
        # "pos_map",
        # "goal_map",
        # "local_grid",
    ]
    EGO_MAP_SIZE = (160, 160)  # pixels
    IG_MAP_RESOLUTION = 0.5
    SUBMAP_RESOLUTION = 0.01 #0.025  # Pixel / meter

    SensorConfig = SensorConfig(MAX_NUM_AGENTS_IN_ENVIRONMENT-1,EGO_MAP_SIZE, MAP_SIZE, IG_MAP_RESOLUTION, SUBMAP_RESOLUTION)




    # visualization
    SHOW_EPISODE_PLOTS = False
    SAVE_EPISODE_PLOTS = True

    TEST_MODE = False

    IG_SENSE_FOV = 360.0
    IG_SENSE_RADIUS = 2.0
    IG_SENSE_rOcc = 3.0
    IG_SENSE_rEmp = 0.33
    IG_GOALS_SETTINGS = {"max_steps": 128}





