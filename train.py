import numpy as np
import tensorflow as tf
import scipy
import scipy.misc
import scipy.io
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from tqdm import tqdm
import time
import os
from functools import partial

from generate_data import generate_data
from utils import *
from run_environment import RunEnvironment
from policy import *
from baseline import Baseline
from rrt_connect_env import RRTConnectEnv
from rrt_bi_env import RRTBiEnv
from est_env import ESTEnv
import arm

gamma = 1

def reinforce_train(env_list, baseline_list, policy, savefile, niter=5000):
    num_repeat = 4
    max_batch = 10000
    basename = os.path.splitext(os.path.basename(savefile))[0]

    # save an initial models
    policy.save_model(savefile + ".initial.ckpt")

    stats_list = []
    avg_reward_list = []
    for env in env_list:
        stats_list.append(RunningStats(200000))
        avg_reward_list.append(RunningStats(niter))

    run_env = RunEnvironment(policy)
    print("Calculating starting baseline")

    # train baseline estimator a bit
    for i, (env, baseline, stats) in enumerate(zip(env_list, baseline_list, stats_list)):
        print("Baseline " + str(i))
        reward_list = []
        state_list = []
        for _ in tqdm(range(4)):
            states, _, reward = run_env.run(env)
            states = np.array(states)
            reward = np.array(reward)
            reward = get_disc_rewards(reward, gamma)
            stats.push_list(reward.tolist())
            reward_list.append(reward)
            state_list.append(states)

        for _ in tqdm(range(4)):
            reward = (reward_list[i] - stats.get_mean()) / stats.get_std()
            states = state_list[i]
            # reward = (reward - stats.get_mean()) / stats.get_std()

            num_data = len(states)
            num_batches = int(math.ceil(float(num_data) / max_batch))
            for i in range(num_batches-1):
                idx1 = i*max_batch
                idx2 = (i+1)*max_batch
                baseline.train(np.array(states[idx1:idx2]), np.array(reward[idx1:idx2]))
            idx1 = (num_batches-1)*max_batch
            baseline.train(np.array(states[idx1:]), np.array(reward[idx1:]))

    for j in range(niter):
        for env, baseline, stats, avg_reward in zip(env_list, baseline_list, stats_list, avg_reward_list):
            state_list = []
            action_list = []
            advantage_list = []
            reward_list = []

            env_rewards = []
            for k in range(num_repeat):
                state, action, reward = run_env.run(env)
                state_list.extend(state)
                action_list.extend(action)

                env_rewards.append(np.sum(reward))

                # TODO: break this up into chunks incase we are asking for too many baselines at once
                baseline_value = baseline.get_baseline(np.array(state))
                reward = np.array(reward)
                reward = get_disc_rewards(reward, gamma)

                stats.push_list(reward.tolist())
                reward = (reward - stats.get_mean()) / stats.get_std()
                reward_list.extend(reward.tolist())

                advantage = reward - baseline_value
                advantage_list.extend(advantage.tolist())
                # baseline.train(np.array(state), reward)

            avg_reward.push(np.mean(env_rewards))

            num_data = len(state_list)
            num_batches = int(math.ceil(float(num_data) / max_batch))

            for i in range(num_batches-1):
                idx1 = i*max_batch
                idx2 = (i+1)*max_batch
                baseline.train(np.array(state_list[idx1:idx2]), np.array(reward_list[idx1:idx2]))
            idx1 = (num_batches-1)*max_batch
            baseline.train(np.array(state_list[idx1:]), np.array(reward_list[idx1:]))

            for i in range(num_batches-1):
                idx1 = i*max_batch
                idx2 = (i+1)*max_batch
                loss = policy.update(state_list[idx1:idx2], action_list[idx1:idx2], advantage_list[idx1:idx2])
            idx1 = (num_batches-1)*max_batch
            loss = policy.update(state_list[idx1:], action_list[idx1:], advantage_list[idx1:])

            # batch_size = min(max_batch, len(state_list))
            # batch_idx = np.random.randint(len(state_list), size=batch_size)
            # state_list = [state_list[idx] for idx in batch_idx]
            # action_list = [action_list[idx] for idx in batch_idx]
            # advantage_list = [advantage_list[idx] for idx in batch_idx]
            # reward_list = [reward_list[idx] for idx in batch_idx]

            loss = policy.update(state_list, action_list, advantage_list)


        iteration_str = "Iteration " + str(j) + ': '
        for avg_reward in avg_reward_list:
            iteration_str += str(avg_reward.get_mean_n(1)) + '\t'
        print(iteration_str)

        if j % 3 == 0:
            policy.save_model(savefile)
            pickle.dump(avg_reward_list, open(savefile + '.p', 'wb'))
            # np.save(savefile + ".npy", np.array(avg_rewards))
        if j % 20 == 0:
            policy.save_model(savefile + '.' + str(j) + '.ckpt')


def plot_feat(policy, savefile):
    obs = np.array([np.linspace(-10, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array([False])}
    probs = policy.sess.run(policy.prob, feed_dict=fd)

    plt.plot(obs, probs[:, 0])
    plt.xlabel("feature")
    plt.ylabel("p(accept)")
    plt.show()

def plot_reward(rrtprob, savefile):
    # def moving_average(a, n=3) :
    #     ret = np.cumsum(a, dtype=float)
    #     ret[n:] = ret[n:] - ret[:-n]
    #     return ret[n - 1:] / n

    avg_reward_list = pickle.load(open(savefile + '.p', 'rb'))

    for i, avg_reward in enumerate(avg_reward_list):
        plt.figure(i)
        # plt.plot(moving_average(avg_reward.vals, 10))
        plt.plot(avg_reward.vals)

    plt.show()

def plot_value(baseline):
    obs = np.array([np.linspace(-10, 10, 100)]).T
    fd = {baseline.state_input: obs, baseline.is_train: np.array(False)}
    value = baseline.sess.run(baseline.value, feed_dict=fd)

    plt.plot(obs, value)
    plt.xlabel("feature")
    plt.ylabel("p(accept)")
    plt.show()


if __name__ == '__main__':
    import argparse
    import sys
    import pickle


    parser = argparse.ArgumentParser(description="Reinforcement Training of Implicit Sampling")
    parser.add_argument('--load', dest='load_file', action='store', type=str, default=None)
    parser.add_argument('--store', dest='store_file', action='store', type=str, default=None)
    parser.add_argument('--env', dest='env', action='store', type=str, default=None,
        required=True,
        choices=['fly_trap_fixed_a', 'fly_trap_fixed_b', 'empty', 'arm'])
    parser.add_argument('--planner', dest='planner', action='store', type=str, 
        default='rrt_connect',
        choices=['rrt_connect', 'rrt_bi', 'est'])
    parser.add_argument('--type', dest='type', action='store',
        required=True,
        choices=['train', 'plot_feat', 'plot_reward', 'plot_value'],
        help="what you want the script to do")
    args = parser.parse_args(sys.argv[1:])

    if args.env == 'fly_trap_fixed_a':
        if args.planner == 'rrt_connect':
            num_feats = 1
            feat_func = get_feat_flytrap
        elif args.planner == 'rrt_bi':
            num_feats = 1
            feat_func = get_feat_flytrap_bi
        elif args.planner == 'est':
            num_feats = 2
            feat_func = get_feat_flytrap_est

        np.random.seed(0)
        data_dict = generate_data('fly_trap_fixed_a')
        l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
        config = {'collision_check': map_collision_check,
                  'random_sample': l2_random_sampler,
                  'steer': holonomic_steer,
                  'dist': l2_dist,
                  'goal_region': l2_goal_region,
                  'feat': feat_func,
                  'num_feat': num_feats}
        data_dict_list = [data_dict]
        config_list = [config]
    elif args.env == 'fly_trap_fixed_b':
        if args.planner == 'rrt_connect':
            num_feats = 1
            feat_funplannerc = get_feat_flytrap
        elif args.planner == 'rrt_bi':
            num_feats = 1
            feat_func = get_feat_flytrap_bi
        elif args.planner == 'est':
            num_feats = 2
            feat_func = get_feat_flytrap_est

        np.random.seed(0)
        data_dict = generate_data('fly_trap_fixed_b')
        l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
        config = {'collision_check': map_collision_check,
                  'random_sample': l2_random_sampler,
                  'steer': holonomic_steer,
                  'dist': l2_dist,
                  'goal_region': l2_goal_region,
                  'feat': feat_func,
                  'num_feat': num_feats}
        data_dict_list = [data_dict]
        config_list = [config]
    elif args.env == 'empty':
        num_feats = 1
        feat_func = get_feat_empty

        np.random.seed(0)
        data_dict = generate_data('empty')
        l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
        config = {'collision_check': map_collision_check,
                  'random_sample': l2_random_sampler,
                  'steer': holonomic_steer,
                  'dist': l2_dist,
                  'goal_region': l2_goal_region,
                  'feat': feat_func,
                  'num_feat': num_feats}
        data_dict_list = [data_dict]
        config_list = [config]
    elif args.env == 'arm':
        num_feats = 3
        feat_func = arm.arm_feat

        points5 = scipy.io.loadmat('pointclouddata/processed_5.mat')['save_struct'][0, 0]
        start5 = np.array([0, 0.0, 0., 0., 0., 0., 0.])
        goal5 = np.array([1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481])
        # points1 = scipy.io.loadmat('pointclouddata/processed_5.mat')['points_arm'][:, 0:3]
        pointcloud_list = [points5]
        start_list = [start5]
        goal_list = [goal5]

        data_dict_list = []
        config_list = []
        for i in range(len(pointcloud_list)):
            data_dict = arm.arm_map_create(pointcloud_list[i], start_list[i], goal_list[i])
            arm_random_sampler = partial(arm.arm_random_sample, eps=0.1)
            config = {'collision_check': arm.arm_collision_check,
                      'random_sample': arm_random_sampler,
                      'steer': arm.arm_steer,
                      'dist': arm.arm_dist_func,
                      'goal_region': arm.arm_goal_region,
                      'feat': feat_func,
                      'num_feat': num_feats}
            data_dict_list.append(data_dict)
            config_list.append(config)
    else:
        raise Exception('Not a valid Environment')


    sess = tf.InteractiveSession()
    baseline_list = []
    env_list = []
    for i, (data_dict, config) in enumerate(zip(data_dict_list, config_list)):
        if args.planner == 'rrt_connect':
            env = RRTConnectEnv(config, data_dict)
        elif args.planner == 'rrt_bi':
            env = RRTBiEnv(config, data_dict)
        elif args.planner == 'est':
            env = ESTEnv(config, data_dict)
        else:
            raise Exception('not valid environment type')
        env_list.append(env)
        if args.type == 'train' or args.type == 'plot_value':

            baseline_list.append(Baseline(config['num_feat'], name='baseline' + str(i), sess=sess))

    np.random.seed(int(time.time()))

    policy = Policy(num_feats, sess)
    tf.global_variables_initializer().run()

    if args.store_file == None:
        store_file = '/tmp/model.ckpt'
    else:
        store_file = args.store_file

    if args.load_file != None:
        policy.load_model(args.load_file)
    else:
        args.load_file = '/tmp/model.ckpt'

    if args.type == 'train':
        reinforce_train(env_list, baseline_list, policy, store_file)
    elif args.type == 'plot_feat':
        plot_feat(policy, store_file)
    elif args.type == 'plot_reward':
        plot_reward(policy, args.load_file)
    elif args.type == 'plot_value':
        plot_value(baseline_list[0])
        



