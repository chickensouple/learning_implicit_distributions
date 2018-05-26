from tree import Tree
import matplotlib.pyplot as plt
import tensorflow as tf
import numpy as np
from utils import *

class RRTEnv(object):
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

        self.rand_node = self.config['random_sample'](self.map_info)
        self.node_feat = self.config['feat'](self.rand_node, 
            self.tree, 
            self.map_info)

        return self.node_feat

    def __run(self, rand_node):
        closest_idx = self.tree.closest_idx(rand_node, self.config['dist'])
        closest_node = self.tree.node_states[closest_idx]

        collision, path = self.extend(closest_node, rand_node, self.map_info)
        new_node = path[-1]

        if not collision:
            self.tree.insert_node(new_node, path, closest_idx)

            if self.config['goal_region'](new_node, self.map_info['goal']):
                self.found_path = True

    def extend(self, node_from, node_to, map_info):
        path, path_cost = self.config['steer'](node_from, node_to)
        new_node = path[-1]

        collision, num_checks = self.config['collision_check'](map_info, path, True)
        self.num_collision_checks += num_checks
        return collision, path


    def step(self, action):
        if self.found_path:
            return self.node_feat, 0, self.found_path, None

        prev_num_coll_checks = self.num_collision_checks
        prev_node_states = len(self.tree.node_states)
        if action == 1:
            self.__run(self.rand_node)

        self.rand_node = self.config['random_sample'](self.map_info)
        self.node_feat = self.config['feat'](self.rand_node, 
            self.tree, self.map_info)

        reward = 0
        reward += -0.1
        reward += -(len(self.tree.node_states) - prev_node_states)
        # reward += -(self.num_collision_checks - prev_num_coll_checks)
        reward += (action == 1) * (-5)

        self.samples_drawn += 1

        closest_idx, dist = self.tree.closest_idx(self.map_info['goal'], self.config['dist'], return_dist=True)
        # print("len: " + str(len(self.tree.node_states)) + "\tdist: " + str(dist))

        return self.node_feat, reward, self.found_path, None

    def show(self):
        plt.cla()
        if self.found_path:
            # self.tree.show(im=self.map_info['map'], goal=self.map_info['goal'], path_idx=len(self.tree.node_states)-1)
            self.tree.show(goal=self.map_info['goal'], path_idx=len(self.tree.node_states)-1)

        else:
            # self.tree.show(im=self.map_info['map'], goal=self.map_info['goal'])
            self.tree.show(goal=self.map_info['goal'])

    def get_path(self):
        if not self.found_path:
            return [], None

        path = self.tree.path_to_root(len(self.tree.node_states)-1)
        path_len = 0

        for i in range(1, len(path)):
            node1 = self.tree.node_states[path[i-1]]
            node2 = self.tree.node_states[path[i]]

            path_len += self.config['dist'](np.array([node1]), node2)
        path.reverse()
        return path, path_len


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from generate_data import generate_data
    from functools import partial
    from utils import *
    from policy import *
    from tqdm import tqdm
    import time
    import pendulum

    np.random.seed()
    data_dict = pendulum.pendulum_generate_map()

    sampler = partial(pendulum.pendulum_sample, eps=0.1)
    l2_config = {'collision_check': pendulum.pendulum_collision_check,
              'random_sample': sampler,
              'steer': pendulum.pendulum_steer,
              'dist': pendulum.pendulum_dist,
              'goal_region': pendulum.pendulum_goal,
              'feat': pendulum.pendulum_feat,
              'num_feat': 2}

    rrt = RRTEnv(l2_config, data_dict)
    # policy = DefaultPolicy()
    policy = Policy(l2_config['num_feat'])

    obs = rrt.reset()
    done = False

    idx = 0
    while not done:
        action = policy.get_action(obs)
        obs, reward, done, _ = rrt.step(action)
        idx += 1

    # path = rrt.get_path()
    # rrt.show()
    # plt.show()


    # rrt.show()
    # plt.show()



    # import matplotlib.pyplot as plt
    # from generate_data import generate_data
    # from functools import partial
    # from utils import *
    # from policy import *
    # from tqdm import tqdm
    # import time
    # import cartpole

    # np.random.seed()
    # data_dict = cartpole.cartpole_generate_map()
    # l2_config = {'collision_check': cartpole.cartpole_collision_check,
    #           'random_sample': cartpole.cartpole_sample,
    #           'steer': cartpole.cartpole_steer,
    #           'dist': cartpole.cartpole_dist,
    #           'goal_region': cartpole.cartpole_goal,
    #           'feat': cartpole.cartpole_feat,
    #           'num_feat': 1}

    # rrt = RRTEnv(l2_config, data_dict)
    # policy = DefaultPolicy()

    # obs = rrt.reset()
    # done = False

    # idx = 0
    # while not done:
    #     action = policy.get_action(obs)
    #     obs, reward, done, _ = rrt.step(action)
    #     idx += 1

    # rrt.show()
    # plt.show()


