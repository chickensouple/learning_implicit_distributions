from tree import Tree
import matplotlib.pyplot as plt
import tensorflow as tf
import numpy as np
from utils import *

class ExtendState(object):
    TRAPPED = 0
    ADVANCED = 1
    REACHED = 2


class RRTConnectEnv(object):
    def __init__(self, config, map_info):
        self.config = config
        self.maxiter = 15000
        self.map_info = map_info
        self.reset()

    def reset(self):
        self.tree = Tree()
        self.found_path = False
        self.num_collision_checks = 0

        self.tree.insert_node(self.map_info['start'], [self.map_info['start']])

        self.map_info['precomputed'] = map_obst_precompute(self.map_info['map'])

        self.rand_node = self.config['random_sample'](self.map_info)
        self.node_feat = self.config['feat'](self.rand_node, 
            self.tree, 
            self.map_info)

        return self.node_feat

    def __run_rrt(self, rand_node):
        closest_idx = self.tree.closest_idx(rand_node, self.config['dist'])
        closest_node = self.tree.node_states[closest_idx]

        state, path = self.extend(closest_node, rand_node, self.map_info['map'])
        while state == ExtendState.ADVANCED:
            new_node = path[-1]
            self.tree.insert_node(new_node, path, closest_idx)

            if self.config['goal_region'](new_node, self.map_info['goal']):
                self.found_path = True
                break

            closest_node = new_node
            closest_idx = len(self.tree.node_states) - 1
            state, path = self.extend(closest_node, rand_node, self.map_info['map'])

        if state == ExtendState.REACHED:
            new_node = path[-1]
            self.tree.insert_node(new_node, path, closest_idx)
            if self.config['goal_region'](new_node, self.map_info['goal']):
                self.found_path = True


    def extend(self, node_from, node_to, env):
        path, path_cost = self.config['steer'](node_from, node_to)
        new_node = path[-1]

        collision, num_checks = self.config['collision_check'](env, path, True)
        self.num_collision_checks += num_checks
        if collision:
            return ExtendState.TRAPPED, path

        dist = self.config['dist'](np.array([new_node]), node_to)
        if dist < 1e-2:
            return ExtendState.REACHED, path
        else: 
            return ExtendState.ADVANCED, path

    def step(self, action):
        if self.found_path:
            return self.node_feat, 0, self.found_path, None


        prev_num_coll_checks = self.num_collision_checks
        prev_node_states = len(self.tree.node_states)
        if action == 1:
            self.__run_rrt(self.rand_node)

        self.rand_node = self.config['random_sample'](self.map_info)
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

    l2_data_dict = generate_data('fly_trap_fixed_b', dubins=False)
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

    rrt = RRTConnectEnv(l2_config, l2_data_dict)
    # policy = BallTreePolicy()
    policy = DynamicDomainPolicy()
    # policy = DefaultPolicy()
    # policy = Policy(l2_config['num_feat'])

    obs = rrt.reset()
    done = False

    idx = 0
    while not done:
        action = policy.get_action(obs)
        obs, reward, done, _ = rrt.step(action)
        
        idx += 1


    rrt.show()
    plt.show()





