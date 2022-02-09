from gym.envs.registration import register
from rl_env.three_envs.gym_foodhunting import HSR_FH, HSRSimple_FH
from rl_env.three_envs.maze import HSR_M, HSRSimple_M
from rl_env.three_envs.grasping import HSR_G, HSRSimple_G
# FoodHunting HSR
register(
    id='FoodHuntingHSR-v0',
    entry_point='rl_env.three_envs:FoodHuntingEnv',
    max_episode_steps=50,
    kwargs={'render': False,
            'robot_model': HSRSimple_FH,
            'max_steps': 50,
            'num_foods': 1,
            'num_fakes': 1,
            'object_size': 0.5,
            'object_radius_scale': 1.0,
            'object_radius_offset': 1.0,
            'object_angle_scale': 1.0}
)

register(
    id='FoodHuntingHSRGUI-v0',
    entry_point='rl_env.three_envs:FoodHuntingEnv',
    max_episode_steps=50,
    kwargs={'render': True, 'robot_model': HSRSimple_FH, 'max_steps': 50, 'num_foods': 1, 'num_fakes': 1, 'object_size': 0.5, 'object_radius_scale': 1.0, 'object_radius_offset': 1.0, 'object_angle_scale': 1.0}
)

register(
    id='MazeNavigatingHSR-v0',
    entry_point='rl_env.three_envs:MazeEnv',
    max_episode_steps=50,
    kwargs={'render': False,
            'robot_model': HSRSimple_M,
            'max_steps': 50,
            'num_foods': 1,
            'num_fakes': 0,
            'object_size': 0.5,
            'object_radius_scale': 1.0,
            'object_radius_offset': 1.0,
            'object_angle_scale': 1.0}
)

register(
    id='MazeNavigatingHSRGUI-v0',
    entry_point='rl_env.three_envs:MazeEnv',
    max_episode_steps=50,
    kwargs={'render': True, 'robot_model': HSRSimple_M, 'max_steps': 50, 'num_foods': 1, 'num_fakes': 0, 'object_size': 0.5, 'object_radius_scale': 1.0, 'object_radius_offset': 1.0, 'object_angle_scale': 1.0}
)

register(
    id='GraspingHSR-v0',
    entry_point='rl_env.three_envs:GraspingEnv',
    max_episode_steps=50,
    kwargs={'render': False,
            'robot_model': HSRSimple_G,
            'max_steps': 50,
            'num_foods': 1,
            'num_fakes': 0,
            'object_size': 0.1,
            'object_radius_scale': 0.1,
            'object_radius_offset': 0.5,
            'object_angle_scale': 0.1}
)

register(
    id='GraspingHSRGUI-v0',
    entry_point='rl_env.three_envs:GraspingEnv',
    max_episode_steps=50,
    kwargs={'render': True, 'robot_model': HSRSimple_G, 'max_steps': 50, 'num_foods': 1, 'num_fakes': 0, 'object_size': 0.1, 'object_radius_scale': 0.1, 'object_radius_offset': 0.5, 'object_angle_scale': 0.1}
)


register(
    id='FoodHuntingHSRTestGUI-v0',
    entry_point='rl_env.three_envs:FoodHuntingEnv',
    max_episode_steps=10000,
    kwargs={'render': True, 'robot_model': HSR_FH, 'max_steps': 10000, 'num_foods': 3, 'num_fakes': 3, 'object_size': 0.5, 'object_radius_scale': 1.0, 'object_radius_offset': 1.0, 'object_angle_scale': 1.0}
)

register(
    id='FoodHuntingHSRTestGUI-v1',
    entry_point='rl_env.three_envs:FoodHuntingEnv',
    max_episode_steps=10000,
    kwargs={'render': True, 'robot_model': HSRSimple_FH, 'max_steps': 10000, 'num_foods': 1, 'num_fakes': 0, 'object_size': 0.5, 'object_radius_scale': 0.0, 'object_radius_offset': 1.5, 'object_angle_scale': 0.25}
)
