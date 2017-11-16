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


num_feats = 1
feat_func = get_feat_flytrap
gamma = 1

def reinforce_train(policy, baseline, savefile, niter=5000):
    num_repeat = 4
    max_batch = 10000
    env_list = []
    baseline_list = []

    basename = os.path.splitext(os.path.basename(savefile))[0]

    # save an initial models
    policy.save_model(savefile + ".initial.ckpt")
    policy.save_model(savefile + ".best.ckpt")
    best_reward = -np.Inf

    # l2_data_dict = generate_data('empty')
    l2_data_dict = generate_data('fly_trap_fixed')
    l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
    l2_goal = l2_goal_region
    l2_config = {'collision_check': map_collision_check,
              'random_sample': l2_random_sampler,
              'steer': holonomic_steer,
              'dist': l2_dist,
              'goal_region': l2_goal,
              'feat': feat_func,
              'num_feat': num_feats,
              'precomputed': map_obst_precompute(l2_data_dict['map'])}

    run_env = RunEnvironment(policy)

    print("Calculating starting baseline")
    env = RRTConnectEnv(l2_config, l2_data_dict)
    
    stats = RunningStats(200000)

    # train baseline estimator a bit
    for _ in tqdm(range(4)):
        state_list, _, reward = run_env.run(env)
        state_list = np.array(state_list)
        reward = np.array(reward)
        stats.push_list(reward.tolist())
        reward = (reward - stats.get_mean()) / stats.get_std()

        baseline.train(state_list, reward)

    # to evaluate how well the policy is currently doing
    running_avg_rewards = RunningAverage(5)

    avg_rewards = []
    for j in range(niter):
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
            stats.push_list(reward.tolist())

            reward = (reward - stats.get_mean()) / stats.get_std()


            advantage = reward - baseline_value
            advantage_list.extend(advantage.tolist())
            baseline.train(np.array(state), reward)

        avg_rewards.append(np.mean(env_rewards))
        running_avg_rewards.push(avg_rewards[-1])
        

        batch_size = min(max_batch, len(state_list))
        batch_idx = np.random.randint(len(state_list), size=batch_size)
        state_list = [state_list[idx] for idx in batch_idx]
        action_list = [action_list[idx] for idx in batch_idx]
        advantage_list = [advantage_list[idx] for idx in batch_idx]


        state_list = np.array(state_list)
        action_list = np.array(action_list)
        advantage_list = np.array(advantage_list)

        total_loss, rloss, entropy_loss, prob_loss = policy.update(state_list, action_list, advantage_list)


        baseline_vals = [baseline.get() for baseline in baseline_list]
        print("Iteration " + str(j) + ': ' + str(avg_rewards[-1]))
        print("total loss: " + str(total_loss))

        if running_avg_rewards.get() > best_reward:
            best_reward = running_avg_rewards.get()
            policy.save_model(savefile + ".best" + str(j) + ".ckpt")

        if j % 3 == 0:
            policy.save_model(savefile)
            np.save(savefile + ".npy", np.array(avg_rewards))

    policy.save_model(savefile)


def test2(policy, savefile):
    obs = np.array([np.linspace(-10, 10, 100)]).T
    fd = {policy.state_input: obs, policy.is_train: np.array([False])}
    probs = policy.sess.run(policy.prob, feed_dict=fd)

    plt.plot(obs, probs[:, 0])
    plt.xlabel("feature")
    plt.ylabel("p(accept)")
    plt.show()

def test3(rrtprob, savefile):
    rewards = np.load(savefile + ".npy")

    def moving_average(a, n=3) :
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    plt.plot(rewards)
    # plt.plot(moving_average(rewards, 10))
    plt.show()

if __name__ == '__main__':
    import argparse
    import sys
    import pickle

    parser = argparse.ArgumentParser(description="Supervised Training")
    parser.add_argument('--load', dest='load_file', action='store', type=str, default=None)
    parser.add_argument('--store', dest='store_file', action='store', type=str, default=None)
    parser.add_argument('--type', dest='type', action='store',
        required=True,
        choices=['train', 'test', 'plot', 'test2', 'test3'],
        help="what you want the script to do")
    parser.add_argument('--policy', dest='policy', action='store', type=str, default='nn',
        choices=['nn', 'logistic'],
        help='type of policy to use')
    args = parser.parse_args(sys.argv[1:])


    sess = tf.InteractiveSession()
    policy = Policy(num_feats, sess)
    baseline = Baseline(num_feats, sess)
    tf.global_variables_initializer().run() 


    if args.store_file == None:
        store_file = '/tmp/model.ckpt'
    else:
        store_file = args.store_file

    if args.load_file != None:
        policy.load_model(args.load_file)

    if args.type == 'train':
        reinforce_train(policy, baseline, store_file)
    elif args.type == 'test':
        test(policy, args.load_file)
    elif args.type == 'plot':
        plot(policy)
    elif args.type == 'test2':
        test2(policy, store_file)
    elif args.type == 'test3':
        test3(policy, args.load_file)
        
