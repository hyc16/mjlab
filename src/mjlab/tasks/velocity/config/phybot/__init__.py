from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import phybot_mini_flat_env_cfg, phybot_mini_rough_env_cfg
from .rl_cfg import phybot_mini_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-phybot-mini",
  env_cfg=phybot_mini_rough_env_cfg(),
  play_env_cfg=phybot_mini_rough_env_cfg(play=True),
  rl_cfg=phybot_mini_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-phybot-mini",
  env_cfg=phybot_mini_flat_env_cfg(),
  play_env_cfg=phybot_mini_flat_env_cfg(play=True),
  rl_cfg=phybot_mini_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
