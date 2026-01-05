"""Unitree phybot constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import (
  ElectricActuator,
  reflected_inertia_from_two_stage_planetary,
)
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

Phybot_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "phybot_mini" / "xmls" / "phybot_mini_mark2.xml"
)
assert Phybot_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, Phybot_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(Phybot_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec

##
# Actuator config.
##

# Motor specs (from Unitree).
ARMATURE_102 = 0.01
ARMATURE_78 = 0.01
ARMATURE_68 = 0.01
ARMATURE_47 = 0.01
PHYARC_102 = ElectricActuator(
  reflected_inertia=ARMATURE_102,
  velocity_limit=20.0,
  effort_limit=200.0,
)
PHYARC_78 = ElectricActuator(
  reflected_inertia=ARMATURE_78,
  velocity_limit=20.0,
  effort_limit=80.0,
)
PHYARC_68 = ElectricActuator(
  reflected_inertia=ARMATURE_68,
  velocity_limit=20.0,
  effort_limit=58.0,
)
PHYARC_47 = ElectricActuator(
  reflected_inertia=ARMATURE_47,
  velocity_limit=20.0,
  effort_limit=9.0,
)

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

# STIFFNESS_102 = ARMATURE_102 * NATURAL_FREQ**2
# STIFFNESS_78 = ARMATURE_78 * NATURAL_FREQ**2
# STIFFNESS_68 = ARMATURE_68 * NATURAL_FREQ**2
# STIFFNESS_47 = ARMATURE_47 * NATURAL_FREQ**2
#
# DAMPING_102 = 2.0 * DAMPING_RATIO * ARMATURE_102 * NATURAL_FREQ
# DAMPING_78 = 2.0 * DAMPING_RATIO * ARMATURE_78 * NATURAL_FREQ
# DAMPING_68 = 2.0 * DAMPING_RATIO * ARMATURE_68 * NATURAL_FREQ
# DAMPING_47 = 2.0 * DAMPING_RATIO * ARMATURE_47 * NATURAL_FREQ

STIFFNESS_102 = 100.0
STIFFNESS_78 = 100.0
STIFFNESS_68_I = 100.0
STIFFNESS_68_II = 50.0
STIFFNESS_47 = 5.0

DAMPING_102 = 10.0
DAMPING_78 = 10.0
DAMPING_68_I = 10.0
DAMPING_68_II = 5.0
DAMPING_47 = 5.0

PHYBOT_ARC_102 = BuiltinPositionActuatorCfg(
  target_names_expr=(
      ".*_hip_pitch_joint",
  ),
  effort_limit=PHYARC_102.effort_limit,
  armature=PHYARC_102.reflected_inertia,
  stiffness=STIFFNESS_102,
  damping=DAMPING_102,
)
PHYBOT_ARC_78 = BuiltinPositionActuatorCfg(
  target_names_expr=(
      ".*_hip_roll_joint",
      ".*_knee_joint",),
  effort_limit=PHYARC_78.effort_limit,
  armature=PHYARC_78.reflected_inertia,
  stiffness=STIFFNESS_78,
  damping=DAMPING_78,
)
PHYBOT_ARC_68_I = BuiltinPositionActuatorCfg(
  target_names_expr=(
      ".*_hip_yaw_joint",
      "waist_yaw_joint",
  ),
  effort_limit=PHYARC_68.effort_limit,
  armature=PHYARC_68.reflected_inertia,
  stiffness=STIFFNESS_68_I,
  damping=DAMPING_68_I,
)
PHYBOT_ARC_68_II = BuiltinPositionActuatorCfg(
  target_names_expr=(
      ".*_ankle_pitch_joint",
      ".*_ankle_roll_joint",
      ".*_shoulder_pitch_joint",
      ".*_shoulder_roll_joint",
      ".*_elbow_pitch_joint"
  ),
  effort_limit=PHYARC_68.effort_limit,
  armature=PHYARC_68.reflected_inertia,
  stiffness=STIFFNESS_68_II,
  damping=DAMPING_68_II,
)
PHYBOT_ARC_47 = BuiltinPositionActuatorCfg(
  target_names_expr=(
    ".*_shoulder_yaw_joint",
  ),
  effort_limit=PHYARC_47.effort_limit,
  stiffness=STIFFNESS_47,
  armature=PHYARC_47.reflected_inertia,
  damping=DAMPING_47,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.65),
  joint_pos={},
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.69),
  joint_pos={
    ".*_hip_pitch_joint": -0.15,
    ".*_knee_joint": 0.3,
    ".*_ankle_pitch_joint": -0.15,
    ".*_elbow_pitch_joint": -0.1,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# This enables all collisions, including self collisions.
# Self-collisions are given condim=1 while foot collisions
# are given condim=3 and custom friction and solimp.

FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot[1-7]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-7]_collision$": 1},
  friction={r"^(left|right)_foot[1-7]_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=0,
  conaffinity=1,
  condim={r"^(left|right)_foot[1-7]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-7]_collision$": 1},
  friction={r"^(left|right)_foot[1-7]_collision$": (0.6,)},
)

FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(left|right)_foot[1-7]_collision$",),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Final config.
##

PHYBOT_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    PHYBOT_ARC_102,
    PHYBOT_ARC_78,
    PHYBOT_ARC_68_I,#KP100
    PHYBOT_ARC_68_II,#KP50
    PHYBOT_ARC_47
  ),
  soft_joint_pos_limit_factor=0.95,
)

def get_phybot_robot_cfg() -> EntityCfg:
  """Get a fresh G1 robot configuration instance.
  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=PHYBOT_ARTICULATION,
  )

PHYBOT_ACTION_SCALE: dict[str, float] = {}
#这里只控制下肢，如果全身控制需要注释_LOWER_LIMB_JOINT_EXPR
_LOWER_LIMB_JOINT_EXPR = {
  r".*_hip_pitch_joint",
  r".*_hip_roll_joint",
  r".*_hip_yaw_joint",
  r".*_knee_joint",
  r".*_ankle_pitch_joint",
  r".*_ankle_roll_joint",
}
for a in PHYBOT_ARTICULATION.actuators:
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    if n in _LOWER_LIMB_JOINT_EXPR:
      PHYBOT_ACTION_SCALE[n] = 0.25

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_phybot_robot_cfg())

  viewer.launch(robot.spec.compile())
