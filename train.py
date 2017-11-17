import numpy as np
import tensorflow as tf
import scipy
import scipy.misc
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
        for _ in tqdm(range(4)):
            state_list, _, reward = run_env.run(env)
            state_list = np.array(state_list)
            reward = np.array(reward)
            reward = get_disc_rewards(reward, gamma)
            stats.push_list(reward.tolist())
            reward = (reward - stats.get_mean()) / stats.get_std()
            baseline.train(state_list, reward)

    for j in range(niter):
        for env, baseline, stats, avg_reward in zip(env_list, baseline_list, stats_list, avg_reward_list):
            state_list = []
            action_list = []
            advantage_list = []

            env_rewards = []
            for k in range(num_repeat):
                state, action, reward = run_env.run(env)
                state_list.extend(state)
                action_list.extend(action)

                env_rewards.append(np.sum(reward))

                baseline_value = baseline.get_baseline(np.array(state))
                reward = np.array(reward)
                reward = get_disc_rewards(reward, gamma)

                stats.push_list(reward.tolist())
                reward = (reward - stats.get_mean()) / stats.get_std()

                advantage = reward - baseline_value
                advantage_list.extend(advantage.tolist())
                baseline.train(np.array(state), reward)

            avg_reward.push(np.mean(env_rewards))

            batch_size = min(max_batch, len(state_list))
            batch_idx = np.random.randint(len(state_list), size=batch_size)
            state_list = [state_list[idx] for idx in batch_idx]
            action_list = [action_list[idx] for idx in batch_idx]
            advantage_list = [advantage_list[idx] for idx in batch_idx]


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


if __name__ == '__main__':
    import argparse
    import sys
    import pickle

    parser = argparse.ArgumentParser(description="Reinforcement Training of Implicit Sampling")
    parser.add_argument('--load', dest='load_file', action='store', type=str, default=None)
    parser.add_argument('--store', dest='store_file', action='store', type=str, default=None)
    parser.add_argument('--env', dest='env', action='store', type=str, default=None,
        required=True,
        choices=['fly_trap_fixed_a', 'fly_trap_fixed_b', 'empty'])
    parser.add_argument('--type', dest='type', action='store',
        required=True,
        choices=['train', 'plot_feat', 'plot_reward'],
        help="what you want the script to do")
    args = parser.parse_args(sys.argv[1:])

    if args.env == 'fly_trap_fixed_a':
        num_feats = 1
        feat_func = get_feat_flytrap

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
        num_feats = 1
        feat_func = get_feat_flytrap

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
    else:
        raise Exception('Not a valid Environment')


    sess = tf.InteractiveSession()
    baseline_list = []
    env_list = []
    for i, (data_dict, config) in enumerate(zip(data_dict_list, config_list)):
        env = RRTConnectEnv(config, data_dict)
        env_list.append(env)
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
        



