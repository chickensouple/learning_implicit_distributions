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
        self.num_collision_checks = 0
        self.samples_drawn = 0


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


    def step(self, action):
        if self.found_path:
            return self.node_feat, 0, self.found_path, None

        prev_num_coll_checks = self.num_collision_checks
        prev_node_states = len(self.forward_tree.node_states) + len(self.backward_tree.node_states)

        if action == 1:
            self.__run(self.rand_node)


        self.tree_idx = 1 - self.tree_idx
        new_node_states = len(self.forward_tree.node_states) + len(self.backward_tree.node_states)

        self.rand_node = self.config['random_sample'](self.map_info)
        self.node_feat = self.config['feat'](self.rand_node, 
            self.trees, 
            self.map_info,
            self.tree_idx)

        reward = 0
        reward += -(1) * 0.01
        reward += -(new_node_states - prev_node_states)
        reward += -(self.num_collision_checks - prev_num_coll_checks)

        self.samples_drawn += 1
        return self.node_feat, reward, self.found_path, None
        

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
    import matplotlib.pyplot as plt
    from generate_data import generate_data
    from functools import partial
    from utils import *
    from policy import *
    from tqdm import tqdm
    import time

    # # np.random.seed(0)
    # # l2_data_dict = generate_data('fly_trap_fixed_a', dubins=False)
    # l2_data_dict = generate_data('fly_trap_fixed_b', dubins=False)
    # l2_random_sampler = partial(map_sampler_goal_bias, eps=0.1)
    # l2_goal = l2_goal_region
    # l2_config = {'collision_check': map_collision_check,
    #           'random_sample': l2_random_sampler,
    #           'steer': holonomic_steer,
    #           'dist': l2_dist,
    #           'goal_region': l2_goal,
    #           'feat': get_feat_flytrap_bi,
    #           'num_feat': 1,
    #           'precomputed': map_obst_precompute(l2_data_dict['map'])}

    # rrt = RRTBiEnv(l2_config, l2_data_dict)
    # policy = DefaultPolicy()

    # obs = rrt.reset()
    # done = False


    # idx = 0
    # while not done:
    #     action = policy.get_action(obs)
    #     obs, reward, done, _ = rrt.step(action)
        
    #     idx += 1

    #     if idx % 10 == 0:
    #         rrt.show()
    #         plt.show(block=False)
    #         plt.pause(0.1)

    # rrt.show()
    # plt.show()



    import arm
    import scipy.io



    qstart = np.array([90, 10, 0, -150, 0, 0, 0]) * math.pi / 180
    qgoal = np.array([20, -15, 0, 0, 0, 10, 0]) * math.pi / 180
    qgoal = np.array([20, 15, 0, 0, 0, -70, 0]) * math.pi / 180
    qgoal = np.array([35, -15, 0, 0, 90, 45, 0]) * math.pi / 180


    pointcloud = scipy.io.loadmat('pointclouddata/processed_3.mat')['save_struct'][0, 0]
    # pointcloud = {'points': np.array([[0, 0, 0]]), 'means': 0, 'sigmas': 0}
    arm_data_dict = arm.arm_map_create(pointcloud, qstart, qgoal)
    arm_random_sampler = partial(arm.arm_random_sample, eps=0.1)
    arm_config = {'collision_check': arm.arm_collision_check,
                  'random_sample': arm_random_sampler,
                  'steer': arm.arm_steer,
                  'dist': arm.arm_dist_func,
                  'goal_region': arm.arm_goal_region,
                  'feat': arm.arm_feat_bi,
                  'num_feat': 1}

    rrt = RRTBiEnv(arm_config, arm_data_dict)
    policy = DefaultPolicy()


    obs = rrt.reset()
    done = False
    idx = 0
    while not done:
        action = policy.get_action(obs)
        obs, reward, done, _ = rrt.step(action)
        
        idx += 1
    
    if not rrt.found_path:
        print("Path Not Found")


    # plot points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    points = pointcloud['points']
    rand_points = np.random.randint(points.shape[0], size=1000)
    ax.scatter(points[rand_points, 0], points[rand_points, 1], points[rand_points, 2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')


    path, cost = rrt.get_path()
    print("Cost: " + str(cost))
    print("Path Len: " + str(len(path)))
    print("Path: " + str(path))
    armv = arm.ArmVisualize(ax)
    for node in path:
        q = node
        armv.plot(q)
        plt.show(block=False)
        plt.pause(0.1)
        raw_input("Press Enter to Continue:")

    print("Done")

    plt.show()
