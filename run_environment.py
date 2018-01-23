import numpy as np
from policy import *
from functools import partial
import matplotlib.pyplot as plt

class RunEnvironment(object):
    def __init__(self, policy):
        self.policy = policy

    def run(self, rrt_env, max_iter=100000, show=False):
        obs = rrt_env.reset()

        obs_list = []
        action_list = []
        reward_list = []

        done = False
        i = 0
        last_i = 0
        while not done: 
            action = self.policy.get_action(obs)
            obs_list.append(obs)
            action_list.append(action)

            obs, reward, done, _ = rrt_env.step(action)
            reward_list.append(reward)


            if show and i - last_i > 500:
                last_i = i
                rrt_env.show()
                plt.show(block=False)
                plt.pause(0.01)

            i += 1
            if i >= max_iter:
                break

        return [obs_list, action_list, reward_list]


if __name__ == '__main__':
    from rrt_connect_env import RRTConnectEnv
    from rrt_bi_env import RRTBiEnv
    from generate_data import generate_data
    from utils import *

    # num_feats = 1
    # # policy = DefaultPolicy()
    # # policy = BallTreePolicy()
    # policy = DynamicDomainPolicy()
    # rrtprob = RunEnvironment(policy)

    # np.random.seed(2)
    # l2_data_dict = generate_data('fly_trap_fixed_b')
    # l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
    # l2_goal = l2_goal_region
    # l2_config = {'collision_check': map_collision_check,
    #           'random_sample': l2_random_sampler,
    #           'steer': holonomic_steer,
    #           'dist': l2_dist,
    #           'goal_region': l2_goal,
    #           # 'feat': get_feat_flytrap,
    #           'feat': get_feat_flytrap,
    #           'num_feat': num_feats,
    #           'precomputed': map_obst_precompute(l2_data_dict['map'])}

    # env = RRTConnectEnv(l2_config, l2_data_dict)
    # # env = RRTBiEnv(l2_config, l2_data_dict)

    # rrtprob.run(env, show=True)
    # env.show()
    # plt.show()




    import arm
    import scipy.io
    import tensorflow as tf
    from tqdm import tqdm


    qstart = np.array([90, 10, 0, -150, 0, 0, 0]) * math.pi / 180
    # qgoal = np.array([20, -15, 0, 0, 0, 10, 0]) * math.pi / 180
    # qgoal = np.array([20, 15, 0, 0, 0, -70, 0]) * math.pi / 180
    qgoal = np.array([35, -15, 0, 0, 90, 45, 0]) * math.pi / 180
    pointcloud = scipy.io.loadmat('pointclouddata/processed_3.mat')['save_struct'][0, 0]
    arm_data_dict = arm.arm_map_create(pointcloud, qstart, qgoal)
    arm_random_sampler = partial(arm.arm_random_sample, eps=0.1)
    arm_config = {'collision_check': arm.arm_collision_check,
                  'random_sample': arm_random_sampler,
                  'steer': arm.arm_steer,
                  'dist': arm.arm_dist_func,
                  'goal_region': arm.arm_goal_region,
                  'feat': arm.arm_feat_bi,
                  'num_feat': 4}

    env = RRTBiEnv(arm_config, arm_data_dict)



    # policy = DefaultPolicy()
    # policy = BallTreePolicy()
    # policy = DynamicDomainPolicy()
    
    sess = tf.InteractiveSession()
    policy = Policy(arm_config['num_feat'], sess)
    tf.global_variables_initializer().run()

    rrtprob = RunEnvironment(policy)

    for i in tqdm(range(10)):
        rrtprob.run(env)

