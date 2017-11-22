from tree import Tree
import matplotlib.pyplot as plt
import tensorflow as tf
import numpy as np
from utils import *

def sample_ball(r, dim=2):
    sample = np.random.random_sample((2)) * (2 * r) - r
    while np.linalg.norm(sample) > r:
        sample = np.random.random_sample((2)) * (2 * r) - r
    return sample

class ESTEnv(object):
    def __init__(self, config, map_info):
        self.config = config
        self.map_info = map_info
        self.state_size = len(self.map_info['start'])

        self.r = 3 # radius of sampling
        self.n = 1 # number of samples to take each iteration
        self.reset()

    def _radius_search(self, node_list, node, radius):

        dists = self.config['dist'](np.array(node_list), node)
        dist_mask = dists < radius

        node_idx = np.where(dist_mask)[0]
        return node_idx

    def _compute_and_update_w(self, node, neighbor_idx=None):
        if np.any(neighbor_idx == None):
            neighbor_idx = self.radius_search(self.tree.node_states, node, self.r)
        self.tree.node_info[-1] = 1.0 / (len(neighbor_idx) + 1)
        self.total_weight += self.tree.node_info[-1]

        for idx in neighbor_idx:
            w = self.tree.node_info[idx]
            self.tree.node_info[idx] = w / (w + 1.)
            self.total_weight += (self.tree.node_info[idx] - w)

    def _sample_node(self):
        sample = np.random.random_sample()

        weight_sum = 0
        idx = None
        for i in range(len(self.tree.node_info)):
            weight_sum += self.tree.node_info[i] / self.total_weight
            if weight_sum > sample:
                idx = i
                break

        return idx

    def sample_free_space(self, node, r):
        # with eps chance, sample goal
        rand = np.random.random_sample()
        if rand < 0.1:
            goal = self.map_info['goal']
            direction = goal - node
            dir_len = np.linalg.norm(direction)
            if dir_len > 1e-2:
                sample = node + (r * direction / dir_len)
                if not self.config['collision_check'](self.map_info['map'], sample):
                    return sample
                    
        sample = node + sample_ball(r)
        while self.config['collision_check'](self.map_info['map'], sample):
            sample = node + sample_ball(r)
        return sample


    def reset(self):
        self.tree = Tree()
        self.found_path = False
        self.num_collision_checks = 0

        # node info is 1/w(x)
        self.tree.insert_node(self.map_info['start'], 1)
        # keeps track of sum of 1/w(x)
        self.total_weight = 1

        # self.rand_node = self.config['random_sample'](self.map_info)
        self.rand_node_idx = self._sample_node()
        self.rand_node = self.tree.node_states[self.rand_node_idx]
        self.node_feat = self.config['feat'](self.rand_node, 
            self.tree, 
            self.map_info)

        return self.node_feat

    def __run(self):
        samples = []
        for i in range(self.n):
            samples.append(self.sample_free_space(self.rand_node, self.r))

        for sample in samples:
            neighbor_idx = self._radius_search(self.tree.node_states, sample, self.r)
            w = len(neighbor_idx) + 1
            if np.random.sample() > 1.0 / w:
                continue

            path, path_cost = self.config['steer'](self.rand_node, sample)
            collision, num_checks = self.config['collision_check'](self.map_info['map'], path, True)
            self.num_collision_checks += num_checks

            if collision:
                continue

            new_node = path[-1]
            self.tree.insert_node(new_node, node_info=1, parent_idx=self.rand_node_idx)
            self._compute_and_update_w(new_node, neighbor_idx)

            if self.config['goal_region'](new_node, self.map_info['goal']):
                self.found_path = True
                break


    def step(self, action):
        if self.found_path:
            return self.node_feat, 0, self.found_path, None


        prev_num_coll_checks = self.num_collision_checks
        prev_node_states = len(self.tree.node_states)
        if action == 1:
            self.__run()


        self.rand_node_idx = self._sample_node()
        self.rand_node = self.tree.node_states[self.rand_node_idx]
        self.node_feat = self.config['feat'](self.rand_node, 
            self.tree, self.map_info)

        reward = 0
        reward += -0.01
        reward += -(len(self.tree.node_states) - prev_node_states)
        reward += -(self.num_collision_checks - prev_num_coll_checks)

        return self.node_feat, reward, self.found_path, None

    def show(self):
        plt.cla()
        if self.found_path:
            self.tree.show(im=self.map_info['map'], goal=self.map_info['goal'], path_idx=len(self.tree.node_states)-1)
        else:
            self.tree.show(im=self.map_info['map'], goal=self.map_info['goal'])


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from generate_data import generate_data
    from functools import partial
    from utils import *
    from policy import *
    from tqdm import tqdm
    import time

    l2_data_dict = generate_data('empty', dubins=False)
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

    np.random.seed(0)
    rrt = ESTEnv(l2_config, l2_data_dict)
    # policy = BallTreePolicy()
    # policy = DynamicDomainPolicy()
    policy = DefaultPolicy()
    # policy = Policy(l2_config['num_feat'])

    obs = rrt.reset()
    done = False

    idx = 0
    while not done:
        action = policy.get_action(obs)
        obs, reward, done, _ = rrt.step(action)
        
        idx += 1

        if idx % 10 == 0:
            rrt.show()
            plt.show(block=False)
            plt.pause(0.1)

    rrt.show()
    plt.show()





