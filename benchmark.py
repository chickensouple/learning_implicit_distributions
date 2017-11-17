import numpy as np
from utils import *
from functools import partial
from generate_data import generate_data
from rrt_connect_env import RRTConnectEnv
from run_environment import RunEnvironment
from policy import *
import time
from tqdm import tqdm

def test(env, policy, policyname, niter, file):
    run_env = RunEnvironment(policy)

    rewards = []
    num_nodes = []
    num_collision_checks = []
    for i in tqdm(range(niter)):
        _, _, reward = run_env.run(env)

        rewards.append(np.sum(reward))
        num_nodes.append(len(env.tree.node_states))
        num_collision_checks.append(env.num_collision_checks)

    print(policyname)
    print("===================")
    print("Mean Reward: " + str(np.mean(rewards)))
    print("Std Reward: " + str(np.std(rewards)))
    print("Mean Num Nodes: " + str(np.mean(num_nodes)))
    print("Std Num Nodes: " + str(np.std(num_nodes)))
    print("Mean Num Collision Checks: " + str(np.mean(num_collision_checks)))
    print("Std Num Collision Checks: " + str(np.std(num_collision_checks)))

    file.write(policyname + "\n")
    file.write("===================" + "\n")
    file.write("Mean Reward: " + str(np.mean(rewards)) + "\n")
    file.write("Std Reward: " + str(np.std(rewards)) + "\n")
    file.write("Mean Num Nodes: " + str(np.mean(num_nodes)) + "\n")
    file.write("Std Num Nodes: " + str(np.std(num_nodes)) + "\n")
    file.write("Mean Num Collision Checks: " + str(np.mean(num_collision_checks)) + "\n")
    file.write("Std Num Collision Checks: " + str(np.std(num_collision_checks)) + "\n")
    file.write("\n\n\n")




if __name__ == '__main__':
    import argparse
    import sys
    import pickle

    parser = argparse.ArgumentParser(description="Benchmark")
    parser.add_argument('--output', dest='output', 
        action='store', 
        type=str, 
        default='data/output.txt',
        help='output file to write to')
    parser.add_argument('--env', dest='env', action='store',
        required=True,
        choices=['fly_trap_fixed_a', 'fly_trap_fixed_b'],
        help="environment type")
    args = parser.parse_args(sys.argv[1:])



    niter = 100

    policy1 = DefaultPolicy()
    policy2 = BallTreePolicy()
    policy3 = DynamicDomainPolicy()


    policies = [\
        [policy1, 'default'],
        # [policy2, 'balltree'],
        # [policy3, 'dynamicdomain']
    ]

    np.random.seed(0)
    l2_data_dict = generate_data(args.env)
    l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
    l2_goal = l2_goal_region
    l2_config = {'collision_check': map_collision_check,
              'random_sample': l2_random_sampler,
              'steer': holonomic_steer,
              'dist': l2_dist,
              'goal_region': l2_goal,
              'feat': get_feat_flytrap,
              'num_feat': 1,
              'precomputed': map_obst_precompute(l2_data_dict['map'])}
    np.random.seed(int(time.time()))


    file = open(args.output, 'w')

    env = RRTConnectEnv(l2_config, l2_data_dict)
    for policy, policyname in policies:
        test(env, policy, policyname, niter, file)

    file.close()


