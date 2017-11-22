from tree import Tree
import matplotlib.pyplot as plt
import tensorflow as tf
import numpy as np
from utils import *

class ExtendState(object):
    TRAPPED = 0
    ADVANCED = 1
    REACHED = 2

class RRTBiEnv(object):
    def __init__(self, config, map_info):
        self.config = config
        self.map_info = map_info
        self.reset()

    def reset(self):
        self.forward_tree = Tree()
        self.backward_tree = Tree()

        self.found_path = False
        self.num_collisions = 0

        self.forward_tree.insert_node(self.map_info['start'])
        self.backward_tree.insert_node(self.map_info['goal'])
        self.trees = [self.forward_tree, self.backward_tree]
        self.tree_idx = 0
        self.goal_idx = [None, None]

        self.rand_node = self.config['random_sample'](self.map_info)
        self.node_feat = self.config['feat'](self.rand_node, 
            self.trees, 
            self.map_info,
            self.tree_idx)

        return self.node_feat

    def __run(self, rand_node):

        curr_tree = self.trees[self.tree_idx]
        other_tree = self.trees[1 - self.tree_idx]
        env = self.map_info['map']

        closest_idx = curr_tree.closest_idx(rand_node, self.config['dist'])
        closest_node = curr_tree.node_states[closest_idx]

        state, path = self.extend(closest_node, rand_node, env)
        if state == ExtendState.TRAPPED:
            self.num_collisions += 1
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
                state, path = self.extend(closest_node, new_node, env)
                if state != ExtendState.ADVANCED:
                    break
                other_tree.insert_node(path[-1], path, closest_idx)

            if state == ExtendState.REACHED:
                other_tree.insert_node(path[-1], path, closest_idx)

                self.goal_idx[1 - self.tree_idx] = len(other_tree.node_states) - 1
                self.goal_idx[self.tree_idx] = len(curr_tree.node_states) - 1

                self.found_path = True
                return
            elif state == ExtendState.TRAPPED:
                self.num_collisions += 1

    def step(self, action):
        if self.found_path:
            return self.node_feat, 0, self.found_path, None

        prev_num_collisions = self.num_collisions
        prev_node_states = len(self.forward_tree.node_states) + len(self.backward_tree.node_states)

        if action == 1:
            self.__run(self.rand_node)

        new_node_states = len(self.forward_tree.node_states) + len(self.backward_tree.node_states)


        self.rand_node = self.config['random_sample'](self.map_info)
        self.node_feat = self.config['feat'](self.rand_node, 
            self.trees, 
            self.map_info,
            self.tree_idx)

        reward = 0
        reward += -(1) * 0.01
        reward += -(new_node_states - prev_node_states)
        reward += -(self.num_collisions - prev_num_collisions)

        return self.node_feat, reward, self.found_path, None
        

    def extend(self, node_from, node_to, env):
        path, path_cost = self.config['steer'](node_from, node_to)
        new_node = path[-1]

        if self.config['collision_check'](env, path):
            return ExtendState.TRAPPED, path

        dist = self.config['dist'](np.array([new_node]), node_to)
        if dist < 1e-2:
            return ExtendState.REACHED, path
        else: 
            return ExtendState.ADVANCED, path


    def get_path(self):
        if not self.found_path:
            raise Exception('Path not found yet')

        path1_idx = self.forward_tree.path_to_root(self.goal_idx[0])
        path1_idx =  list(reversed(path1_idx))
        path1 = [self.forward_tree.node_states[i] for i in path1_idx]
        if self.goal_idx[1] == None:
            # only a forward path
            return path1
        else:
            path2_idx = self.backward_tree.path_to_root(self.goal_idx[1])
            path2 = [self.backward_tree.node_states[i] for i in path2_idx]
            return path1 + path2


    def show(self):
        plt.cla()
        if self.found_path:
            self.trees[0].show(im=self.map_info['map'], path_idx=len(self.trees[0].node_states)-1)
            self.trees[1].show(goal=self.map_info['goal'], path_idx=len(self.trees[1].node_states)-1)
        else:
            self.trees[0].show(im=self.map_info['map'])
            self.trees[1].show(goal=self.map_info['goal'])




if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from generate_data import generate_data
    from functools import partial
    from rrt_utils import *
    from map_utils import *
    from gazebo_map import *
    from CylinderRRT import * 
    from arm_interface_ros import *
    from tqdm import tqdm
    import time

    arm_map = CylinderMap('table')
    arm_interface = ArmInterface()
    arm_data_dict = {'map': arm_map,
        'start': np.array([math.pi/2,
            (260./180) * math.pi,
            (160./180.) * math.pi, 
            0., 0, 0.0]),
        'goal': np.array([math.pi/2,
            (210./180) * math.pi,
            (120./180.) * math.pi, 
            0., 0, 0.0])}
    arm_helper = CylinderRRT(arm_map, arm_interface)
    arm_config = {'collision_check': arm_helper.get_collision_check(),
        'random_sample': arm_helper.get_random_sample(eps=0.1),
        'steer': arm_helper.get_steer(),
        'dist': arm_helper.get_dist_func(),
        'goal_region': arm_goal_region,
        'feat': arm_helper.get_feat(),
        'num_feat': 5}


    rrt = RRTBiEnv(arm_config, arm_data_dict)
    rrtpolicy = RRTPolicy(arm_config['num_feat'])

    obs = rrt.reset()
    done = False

    while not done:
        action = rrtpolicy.get_action(obs)

        obs, reward, done, _ = rrt.step(action)


    print "Done"
    if rrt.found_path:
        print('Found Path')
        num_nodes = len(rrt.forward_tree.node_states) + len(rrt.backward_tree.node_states)
        print('Expanded ' + str(num_nodes) + ' nodes')
        path = rrt.get_path()
        print('Path has ' + str(len(path)) + ' nodes')


        for joints in path:
            print "joints:", joints
            # print arm_helper.collision_check(arm_map, joints, debug=True)
            _ = raw_input("Press Enter:")
            arm_interface.send_joint_list(joints)

