# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import ArticulationCfg, AssetBaseCfg, RigidObject, RigidObjectCfg
from omni.isaac.orbit.envs import RLTaskEnvCfg
from omni.isaac.orbit.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.orbit.managers import ObservationTermCfg as ObsTerm
#from omni.isaac.orbit.managers import RandomizationTermCfg as RandTerm
#from omni.isaac.orbit.managers import RewardTermCfg as RewTerm #| Old implementation
#import taskboard_rewards as RewTerm                           # | Own implementation
#from omni.isaac.orbit.managers import SceneEntityCfg
#from omni.isaac.orbit.managers import TerminationTermCfg as DoneTerm
from omni.isaac.orbit.scene import InteractiveSceneCfg
from omni.isaac.orbit.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
#import omni.isaac.orbit_tasks.classic.cartpole.mdp as mdp # | Old implementation



import omni.isaac.orbit_tasks.manipulation.lift.mdp as mdp # | Own Implementation

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

##
# Pre-defined configs
##
#from omni.isaac.orbit_assets.cartpole import CARTPOLE_CFG  # isort:skip
from omni.isaac.orbit_assets import UR10_CFG  # isort:skip 


##
# Scene definition
@configclass
class TaskBoardSceneCfg(InteractiveSceneCfg):
    """Configuration for a task-board scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # Cube for robot
    cube = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Cuboid",
        spawn=sim_utils.CuboidCfg(size=(0.4, 0.2, 1.0)),
    )

    #robot
    UR10_CFG.init_state.pos = (0.0, 0.0, .5)
    robot: ArticulationCfg = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
    
    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )

    taskBoard = AssetBaseCfg(
        prim_path = "{ENV_REGEX_NS}/Cuboid/TaskBoards",
        spawn = sim_utils.UsdFileCfg(usd_path ="/home/chris/Desktop/ws_sims/Models/taskBoards.usdc",
        scale=[0.001,0.001,0.001]
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-0.15, 0.25, 0.7))
        
        #init_state= InitialStateCfg()
    )
    # Object to move 
    #table = RigidObjectCfg(
    #            prim_path="{ENV_REGEX_NS}/Table",
    #            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.014, 0.265, 0.0], rot=[1, 0, 0, 0]),
    #            spawn=sim_utils.CuboidCfg(size=(0.15, 0.30, 0.25),
    #                rigid_props=RigidBodyPropertiesCfg(
    #                    solver_position_iteration_count=16,
    #                    solver_velocity_iteration_count=1,
    #                    max_angular_velocity=1000.0,
    #                    max_linear_velocity=1000.0,
    #                    max_depenetration_velocity=5.0,
    #                    disable_gravity=True,
    #                ),
    #            ),
    #        )

    # TableCube for robot
    #table = RigidObjectCfg(
    #    prim_path="{ENV_REGEX_NS}/Cuboid/Table",
    #    spawn=sim_utils.CuboidCfg(size=(0.15, 0.30, 1),
    #    rigid_props=RigidBodyPropertiesCfg(
    #        solver_position_iteration_count=16,
    #        solver_velocity_iteration_count=1,
    #        max_angular_velocity=1000.0,
    #        max_linear_velocity=1000.0,
    #        max_depenetration_velocity=5.0,
    #        disable_gravity=False)),
    #    init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.2, -0.07))
    #
    #)    


    # Object to move 
    object = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/Object",
                init_state=RigidObjectCfg.InitialStateCfg(pos=[0, 0.25, 0.8], rot=[1, 0, 0, 0]),
                spawn=UsdFileCfg(
                    usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                    scale=(0.8, 0.8, 0.8),
                    rigid_props=RigidBodyPropertiesCfg(
                        solver_position_iteration_count=16,
                        solver_velocity_iteration_count=1,
                        max_angular_velocity=1000.0,
                        max_linear_velocity=1000.0,
                        max_depenetration_velocity=5.0, 
                        disable_gravity=False,
                    ),
                ),
            )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-0.23, 0.1200, 0,], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(usd_path="/home/chris/Desktop/ws_sims/Models/TableMahog.usdc",scale=[0.0008,0.0008,0.0008]),
    )

    #table = RigidObjectCfg(
    #            prim_path="{ENV_REGEX_NS}/Table",
    #            init_state=RigidObjectCfg.InitialStateCfg(pos=[-0.14, 0.25, 0.0], rot=[1, 0, 0, 0]),
    #            spawn=UsdFileCfg(
    #            usd_path="/home/chris/Desktop/ws_sims/Models/TableMahog.usdc"
    #            ,scale=(0.001,0.001,0.001),
    #                #scale=(0.8, 0.8, 0.8),
    #                rigid_props=RigidBodyPropertiesCfg(
    #                    solver_position_iteration_count=16,
    #                    solver_velocity_iteration_count=1,
    #                    max_angular_velocity=1000.0,
    #                    max_linear_velocity=1000.0,
    #                    max_depenetration_velocity=5.0, 
    #                    disable_gravity=False,
    #                ),
    #            ),
    #        )









@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    # no commands for this MDP
    null = mdp.NullCommandCfg()


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    joint_efforts = mdp.JointEffortActionCfg(asset_name="robot", joint_names=[".*"], scale=100.0)


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


##
# MDP settings
##


#@configclass
#class CommandsCfg:
#    """Command terms for the MDP."""
#
#    # no commands for this MDP
#    null = mdp.NullCommandCfg()


#@configclass
#class ActionsCfg:
#    """Action specifications for the MDP."""
#
#
#
#
#    joint_effort = mdp.JointEffortActionCfg(asset_name="robot", 
#                                            #UR3e joint names
#                                            joint_names=[".*"],
#                                            #joint_names=["slider_to_cart"],
#                                             scale=100.0)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


#@configclass
#class RandomizationCfg:
#    """Configuration for randomization."""
#
#    # reset
#    reset_cart_position = RandTerm(
#        func=mdp.reset_joints_by_offset,
#        mode="reset",
#        params={
#            "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
#            "position_range": (-1.0, 1.0),
#            "velocity_range": (-0.5, 0.5),
#        },
#    )
#
#    reset_pole_position = RandTerm(
#        func=mdp.reset_joints_by_offset,
#        mode="reset",
#        params={
#            "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
#            "position_range": (-0.25 * math.pi, 0.25 * math.pi),
#            "velocity_range": (-0.25 * math.pi, 0.25 * math.pi),
#        },
#    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    ## (1) Constant running reward
    #alive = RewTerm(func=mdp.is_alive, weight=1.0)
    ## (2) Failure penalty
    #terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    ## (3) Primary task: keep pole upright
    #pole_pos = RewTerm(
    #    func=mdp.joint_pos_target_l2,
    #    weight=-1.0,
    #    params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]), "target": 0.0},
    #)
    ## (4) Shaping tasks: lower cart velocity
    #cart_vel = RewTerm(
    #    func=mdp.joint_vel_l1,
    #    weight=-0.01,
    #    params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    #)
    ## (5) Shaping tasks: lower pole angular velocity
    #pole_vel = RewTerm(
    #    func=mdp.joint_vel_l1,
    #    weight=-0.005,
    #    params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    #)

    #lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": 0.06}, weight=15.0)

#@configclass
#class TerminationsCfg:
#    """Termination terms for the MDP."""
#
#    # (1) Time out
#    time_out = DoneTerm(func=mdp.time_out, time_out=True)
#    # (2) Cart out of bounds
#    cart_out_of_bounds = DoneTerm(
#        func=mdp.joint_pos_manual_limit,
#        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"], "bounds": (-3.0, 3.0)},
#    )


@configclass
class CurriculumCfg:
    """Configuration for the curriculum."""

    pass


##
# Environment configuration
##


@configclass
class TaskBoardEnvCfg(RLTaskEnvCfg):
    """Configuration for the task-board environment."""

    # Scene settings
    scene: TaskBoardSceneCfg = TaskBoardSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    #randomization: RandomizationCfg = RandomizationCfg()
    # MDP settings
    curriculum: CurriculumCfg = CurriculumCfg()
    rewards: RewardsCfg = RewardsCfg()
    #terminations: TerminationsCfg = TerminationsCfg()
    # No command generator
    commands: CommandsCfg = CommandsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
