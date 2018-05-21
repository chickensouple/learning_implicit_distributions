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

class RRTBiBatchEnv(object):
    def __init__(self, config, map_info):
        self.config = config
        self.map_info = map_info
        self.reset()

    def reset(self):
        self.forward_tree = Tree()
        self.backward_tree = Tree()

        self.found_path = False
        self.num_collision_checks = 0
        self.samples_drawn = 0


        self.forward_tree.insert_node(self.map_info['start'])
        self.backward_tree.insert_node(self.map_info['goal'])
        self.trees = [self.forward_tree, self.backward_tree]
        self.tree_idx = 0
        self.goal_idx = [None, None]

    def __run(self, rand_node):
        curr_tree = self.trees[self.tree_idx]
        other_tree = self.trees[1 - self.tree_idx]

        closest_idx = curr_tree.closest_idx(rand_node, self.config['dist'])
        closest_node = curr_tree.node_states[closest_idx]

        state, path = self.extend(closest_node, rand_node, self.map_info)
        if state == ExtendState.TRAPPED:
            return
        else:
            new_node = path[-1]
            curr_tree.insert_node(new_node, path, closest_idx)

            if self.tree_idx == 0 and self.config['goal_region'](new_node, self.map_info['goal']):
                self.found_path = True
                self.goal_idx[0] = len(curr_tree.node_states)-1

                return

            # connect
            while True:
                closest_idx = other_tree.closest_idx(new_node, self.config['dist'])
                closest_node = other_tree.node_states[closest_idx]
                state, path = self.extend(closest_node, new_node, self.map_info)
                if state != ExtendState.ADVANCED:
                    break
                other_tree.insert_node(path[-1], path, closest_idx)

            if state == ExtendState.REACHED:
                other_tree.insert_node(path[-1], path, closest_idx)

                self.goal_idx[1 - self.tree_idx] = len(other_tree.node_states) - 1
                self.goal_idx[self.tree_idx] = len(curr_tree.node_states) - 1

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

        num_batch_samples = 100
        random_sample_list = [[], []]

        while not self.found_path:
            while len(random_sample_list[self.tree_idx]) == 0:
                feat_list = []
                sample_list = []
                for i in range(num_batch_samples):
                    sample = self.config['random_sample'](self.map_info)
                    feat = self.config['feat'](sample, 
                        self.trees, 
                        self.map_info,
                        self.tree_idx)
                    feat_list.append(feat)
                    sample_list.append(sample)

                feat_list = np.array(feat_list)

                actions = policy.get_action_multiple(feat_list)

                samples = list(compress(sample_list, actions))
                random_sample_list[self.tree_idx].extend(samples)

            sample = random_sample_list[self.tree_idx][0]
            random_sample_list[self.tree_idx].pop(0) # TODO: make more efficient

            self.__run(sample)


    def get_path(self):
        if not self.found_path:
            raise Exception('Path not found yet')

        path1_idx = self.forward_tree.path_to_root(self.goal_idx[0])
        path1_idx =  list(reversed(path1_idx))
        path1 = [self.forward_tree.node_states[i] for i in path1_idx]
        if self.goal_idx[1] == None:
            # only a forward path
            path = path1
        else:
            path2_idx = self.trees[1].path_to_root(self.goal_idx[1])
            path2 = [self.trees[1].node_states[i] for i in path2_idx]
            path = path1 + path2

        path_len = 0
        for i in range(1, len(path)):
            node1 = path[i]
            node2 = path[i-1]

            path_len += self.config['dist'](np.array([node1]), node2)

        return path, path_len

    def show(self):
        plt.cla()
        if self.found_path:
            self.trees[0].show(im=self.map_info['map'], path_idx=len(self.trees[0].node_states)-1)
            self.trees[1].show(goal=self.map_info['goal'], path_idx=len(self.trees[1].node_states)-1)
        else:
            self.trees[0].show(im=self.map_info['map'])
            self.trees[1].show(goal=self.map_info['goal'])




if __name__ == '__main__':
    from functools import partial
    from generate_data import generate_data
    from run_environment import RunEnvironment
    from policy import *
    import time
    from tqdm import tqdm

    policy = Policy(1)

    # policy.load_model('data/model_envA1.ckpt')
    policy.load_model('good_models/models/model_envA2_bi/model_envA2_bi.ckpt.480.ckpt')

    feat = get_feat_flytrap_bi
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

    rrt = RRTBiBatchEnv(config, data_dict)
    rrt.run(policy)

    for i in tqdm(range(10)):
        rrt.run(policy)

    rrt.show()
    plt.show()


