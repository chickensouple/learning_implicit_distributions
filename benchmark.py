import numpy as np
from utils import *
from functools import partial
from generate_data import generate_data
from rrt_connect_env import RRTConnectEnv
from rrt_bi_env import RRTBiEnv
from est_env import ESTEnv
from rrt_env import RRTEnv
from run_environment import RunEnvironment
from policy import *
import time
from tqdm import tqdm
import pendulum

import cProfile

def test(env, policy, policyname, niter, file):
    run_env = RunEnvironment(policy)

    rewards = []
    num_nodes = []
    num_collision_checks = []
    path_lengths = []
    num_samples = []

    num_fail_paths = 0

    start = time.time()
    for i in tqdm(range(niter)):
        _, _, reward = run_env.run(env, max_iter=100000)

        if not env.found_path:
            num_fail_paths += 1
            continue

        rewards.append(np.sum(reward))
        num_nodes.append(len(env.tree.node_states))
        # num_nodes.append(len(env.trees[0].node_states) + len(env.trees[1].node_states))
        num_collision_checks.append(env.num_collision_checks)
        _, path_len = env.get_path()
        path_lengths.append(path_len)
        num_samples.append(env.samples_drawn)
    end = time.time()


    print(policyname)
    print("===================")
    print("Num Paths Failed: " + str(num_fail_paths))
    print("Mean Reward: " + str(np.mean(rewards)))
    print("Std Reward: " + str(np.std(rewards)))
    print("Mean Num Nodes: " + str(np.mean(num_nodes)))
    print("Std Num Nodes: " + str(np.std(num_nodes)))
    print("Mean Num Collision Checks: " + str(np.mean(num_collision_checks)))
    print("Std Num Collision Checks: " + str(np.std(num_collision_checks)))
    print("Mean Path Length: " + str(np.mean(path_lengths)))
    print("Std Path Length: " + str(np.std(path_lengths)))
    print("Mean Num Samples: " + str(np.mean(num_samples)))
    print("Std Num Samples: " + str(np.std(num_samples)))
    print("Time: " + str(end - start))

    file.write(policyname + "\n")
    file.write("===================" + "\n")
    file.write("Mean Reward: " + str(np.mean(rewards)) + "\n")
    file.write("Std Reward: " + str(np.std(rewards)) + "\n")
    file.write("Mean Num Nodes: " + str(np.mean(num_nodes)) + "\n")
    file.write("Std Num Nodes: " + str(np.std(num_nodes)) + "\n")
    file.write("Mean Num Collision Checks: " + str(np.mean(num_collision_checks)) + "\n")
    file.write("Std Num Collision Checks: " + str(np.std(num_collision_checks)) + "\n")
    file.write("Mean Path Length: " + str(np.mean(path_lengths)) + "\n")
    file.write("Std Path Length: " + str(np.std(path_lengths)) + "\n")
    file.write("Mean Num Samples: " + str(np.mean(num_samples)) + "\n")
    file.write("Std Num Samples: " + str(np.std(num_samples)) + "\n")
    file.write("\n\n\n")




if __name__ == '__main__':
    import argparse
    import sys
    import pickle
    import scipy.io
    import arm

    parser = argparse.ArgumentParser(description="Benchmark")
    parser.add_argument('--output', dest='output', 
        action='store', 
        type=str, 
        default='results/output.txt',
        help='output file to write to')
    parser.add_argument('--env', dest='env', action='store',
        required=True,
        choices=['fly_trap_fixed_a', 'fly_trap_fixed_b', 'fly_trap_fixed_a_test', 'fly_trap_fixed_b_test', 'empty', 'arm', 'pendulum'],
        help="environment type")
    args = parser.parse_args(sys.argv[1:])


    niter = 100

    policy1 = DefaultPolicy()
    policy2 = BallTreePolicy()
    policy3 = DynamicDomainPolicy()
    # policy4 = Policy(1)
    # policy4.load_model('good_models/models/model_envA1/model_envA1.ckpt')
    # policy4.load_model('data/model_envB3.ckpt')

    # policy5 = Policy(2)
    # policy5.load_model('good_models/models/model_envA2_est/model_envA2_est.ckpt.20.ckpt')
    # policy5.load_model('good_models/models/model_envA2_est/model_envA2_est.ckpt.0.ckpt')

    # policy6 = Policy(1) 
    # policy6.load_model('good_models/models/model_envA2_bi/model_envA2_bi.ckpt.480.ckpt')

    # policy7 = Policy(4)
    # policy7.load_model('good_models/models/model_envArm3.ckpt/model_envArm3.ckpt.140.ckpt')

    policies = [\
        [policy1, 'default'],
        # [policy7, 'model_Arm3']
        # [policy5, 'est_a2']
        # [policy6, 'rrtbi_a2']
        # [policy4, 'model a1'],
        # [policy4, 'model_b2']
        # [policy2, 'balltree'],
        # [policy3, 'dynamicdomain']
    ]



    # # for rrt_connect
    # feat = get_feat_flytrap
    # num_feat = 1

    # # for est
    # feat = get_feat_flytrap_est
    # num_feat = 2

    # # for rrt bi
    # feat = get_feat_flytrap_bi
    # num_feat = 1



    if args.env == 'arm':


        start = np.array([90, 10, 0, -150, 0, 0, 0]) * math.pi / 180

        goal1 = np.array([20, -15, 0, 0, 0, 10, 0]) * math.pi / 180
        goal2 = np.array([20, 15, 0, 0, 0, -70, 0]) * math.pi / 180
        # 3 is harder problem
        goal3 = np.array([35, -15, 0, 0, 90, 45, 0]) * math.pi / 180
        goal4 = np.array([80, 0, 0, -90, 90, 0, 0]) * math.pi / 180
        goal5 = np.array([100, -50, 0, -90, 90, 0, 0]) * math.pi / 180
        goal6 = np.array([80, 0, 0, -90, 90, 0, 0]) * math.pi / 180
        goal7 = np.array([80, -40, 0, -90, 90, 0, 0]) * math.pi / 180
        goal8 = np.array([80, -40, 0, -90, 90, 0, 0]) * math.pi / 180
        goal9 = np.array([80, -40, 0, -90, 90, 0, 0]) * math.pi / 180
        goal10 = np.array([80, -40, 0, -90, 90, 0, 0]) * math.pi / 180
        goal11 = np.array([85, -40, 0, -90, 90, 0, 0]) * math.pi / 180


        qstart = start
        qgoal = goal11


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        pointcloud = scipy.io.loadmat('pointclouddata/processed_11.mat')['save_struct'][0, 0]
        arm_data_dict = arm.arm_map_create(pointcloud, qstart, qgoal)
        arm_random_sampler = partial(arm.arm_random_sample, eps=0.1)
        arm_config = {'collision_check': arm.arm_collision_check,
                      'random_sample': arm_random_sampler,
                      'steer': arm.arm_steer,
                      'dist': arm.arm_dist_func,
                      'goal_region': arm.arm_goal_region,
                      'feat': arm.arm_feat_bi}
        config = arm_config
        data_dict = arm_data_dict
    elif args.env == 'pendulum':

        data_dict = pendulum.pendulum_generate_map(map_type='test')


        sampler = partial(pendulum.pendulum_sample, eps=0.1)
        config = {'collision_check': pendulum.pendulum_collision_check,
                  'random_sample': sampler,
                  'steer': pendulum.pendulum_steer,
                  'dist': pendulum.pendulum_dist,
                  'goal_region': pendulum.pendulum_goal,
                  'feat': pendulum.pendulum_feat,
                  'num_feat': 2}

    else:
        np.random.seed(0)
        l2_data_dict = generate_data(args.env)
        l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
        l2_goal = l2_goal_region
        l2_config = {'collision_check': map_collision_check,
                  'random_sample': l2_random_sampler,
                  'steer': holonomic_steer,
                  'dist': l2_dist,
                  'goal_region': l2_goal,
                  'feat': feat,
                  'num_feat': num_feat,
                  'precomputed': map_obst_precompute(l2_data_dict['map'])}
        np.random.seed(int(time.time()))
        config = l2_config
        data_dict = l2_data_dict



    file = open(args.output, 'w')

    # env = RRTConnectEnv(config, data_dict)
    # env = ESTEnv(config, data_dict)
    # env = RRTBiEnv(config, data_dict)
    env = RRTEnv(config, data_dict)

    for policy, policyname in policies:
        test(env, policy, policyname, niter, file)

    file.close()


