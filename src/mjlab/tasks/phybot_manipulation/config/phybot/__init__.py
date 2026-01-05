from mjlab.tasks.registry import register_mjlab_task

from .env_cfgs import phybot_lift_cube_env_cfg
from .rl_cfg import phybot_lift_cube_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Lift-Cube-Phybot",
  env_cfg=phybot_lift_cube_env_cfg(),
  play_env_cfg=phybot_lift_cube_env_cfg(play=True),
  rl_cfg=phybot_lift_cube_ppo_runner_cfg(),
)
