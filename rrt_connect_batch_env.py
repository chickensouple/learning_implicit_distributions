from tree import Tree
import matplotlib.pyplot as plt
import tensorflow as tf
import numpy as np
from utils import *
from itertools import compress


class ExtendState(object):
    TRAPPED = 0
    ADVANCED = 1
    REACHED = 2


class RRTConnectBatchEnv(object):
    def __init__(self, config, map_info):
        self.config = config
        self.map_info = map_info
        self.reset()

    def reset(self):
        self.tree = Tree()
        self.found_path = False
        self.num_collision_checks = 0
        self.samples_drawn = 0

        self.tree.insert_node(self.map_info['start'], [self.map_info['start']])


    def __run(self, rand_node):
        closest_idx = self.tree.closest_idx(rand_node, self.config['dist'])
        closest_node = self.tree.node_states[closest_idx]

        state, path = self.extend(closest_node, rand_node, self.map_info)
        while state == ExtendState.ADVANCED:
            new_node = path[-1]
            self.tree.insert_node(new_node, path, closest_idx)

            if self.config['goal_region'](new_node, self.map_info['goal']):
                self.found_path = True
                break

            closest_node = new_node
            closest_idx = len(self.tree.node_states) - 1
            state, path = self.extend(closest_node, rand_node, self.map_info)

        if state == ExtendState.REACHED:
            new_node = path[-1]
            self.tree.insert_node(new_node, path, closest_idx)
            if self.config['goal_region'](new_node, self.map_info['goal']):
                self.found_path = True


    def extend(self, node_from, node_to, map_info):
        path, path_cost = self.config['steer'](node_from, node_to)
        new_node = path[-1]

        collision, num_checks = self.config['collision_check'](map_info, path, True)
        self.num_collision_checks += num_checks
        if collision:
            return ExtendState.TRAPPED, path

        dist = self.config['dist'](np.array([new_node]), node_to)
        if dist < 1e-2:
            return ExtendState.REACHED, path
        else: 
            return ExtendState.ADVANCED, path

    def run(self, policy):

        self.reset()

        random_sample_list = []
        num_batch_samples = 100


        while not self.found_path:
            while len(random_sample_list) == 0:
                feat_list = []
                sample_list = []
                for i in range(num_batch_samples):
                    sample = self.config['random_sample'](self.map_info)
                    feat = self.config['feat'](sample, self.tree, self.map_info)
                    feat_list.append(feat)
                    sample_list.append(sample)

                feat_list = np.array(feat_list)

                actions = policy.get_action_multiple(feat_list)
                
                samples = list(compress(sample_list, actions))
                random_sample_list.extend(samples)

            sample = random_sample_list[0]
            random_sample_list.pop(0) # TODO: make more efficient

            self.__run(sample)      


    def show(self):
        plt.cla()
        if self.found_path:
            self.tree.show(im=self.map_info['map'], goal=self.map_info['goal'], path_idx=len(self.tree.node_states)-1)
        else:
            self.tree.show(im=self.map_info['map'], goal=self.map_info['goal'])



if __name__ == '__main__':
    from functools import partial
    from generate_data import generate_data
    from run_environment import RunEnvironment
    from policy import *
    import time
    from tqdm import tqdm

    policy = Policy(1)

    policy.load_model('data/model_envA1.ckpt')

    feat = get_feat_flytrap
    num_feat = 1

    np.random.seed(0)
    l2_data_dict = generate_data('fly_trap_fixed_a')
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

    rrt = RRTConnectBatchEnv(config, data_dict)
    rrt.run(policy)

    for i in tqdm(range(10)):
        rrt.run(policy)

    rrt.show()
    plt.show()
