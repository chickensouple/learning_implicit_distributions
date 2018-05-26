import numpy as np
import math
from pendulum import Pendulum
from ode_numerical import *

pendulum = Pendulum()

def pendulum_generate_map(map_type='train'):
    if map_type == 'train':
        map_dict = {}
        map_dict['start'] = np.array([[0, 0]]).T
        map_dict['goal'] = np.array([[np.pi, 0]]).T

    elif map_type == 'test':
        map_dict = {}
        map_dict['start'] = np.array([[0, 0]]).T
        map_dict['goal'] = np.array([[np.pi, 0]]).T


    return map_dict

def pendulum_sample(map_info, eps):
    rand = np.random.random()
    if rand > eps:
        x = np.random.random_sample((2, 1))
        x[0] = x[0] * 2*np.pi
        x[1] = x[1] * 4 - 2
    else:
        x = map_info['goal']
    return x

def pendulum_collision_check(map_info, node, return_num_coll):
    return False, 0

def pendulum_steer(node_from, node_to, extend_length=1.5, discrete=0.2):
    global pendulum
    num_tries = 20

    min_path = None
    min_dist = float('inf')

    for i in range(num_tries):
        # sample random control inputs
        u = np.random.random_sample()*2 - 1
        u = np.array([[u]])
        dt = 0.1
        num_dt = np.random.randint(1, 10)

        controller = constant_controller(u)

        path, _ = ode_solver(pendulum.get_diff_eq(), 
            controller,
            x0=node_from, 
            t0=0,
            dt=np.ones(num_dt) * dt)
        xf = path[:, -1]

        dist = pendulum_dist(np.array([xf]).T, node_to)

        if dist < min_dist:
            min_dist = dist
            min_path = copy.deepcopy(path)

    path_list = []
    for i in range(min_path.shape[1]):
        path_list.append(np.array([min_path[:, i]]).T)
    return path_list, path.shape[1]

def angle_dist(diff):
    while np.any(diff >= np.pi):
        diff[diff >= np.pi] -= 2*np.pi
    while np.any(diff < -np.pi):
        diff[diff < np.pi] += 2*np.pi
    return diff

def pendulum_dist(node_from, node_to):
    node_from_cp = np.squeeze(node_from).T
    node_from_cp = np.reshape(node_from_cp, (2, -1))

    diffs = node_from_cp - node_to
    angle_diffs = angle_dist(diffs[0, :])
    vel_diffs = diffs[1, :]


    dists = np.sqrt(np.square(angle_diffs) + np.square(vel_diffs))

    return dists


def pendulum_goal(a, b):
    diff = a - b

    angle_diff = np.abs(angle_dist(diff[0]))
    angular_vel_diff = np.abs(diff[1])

    if angle_diff < 0.1 and angular_vel_diff < 0.1:
        return True
    else:
        return False



def pendulum_feat(node, tree, map_dict):
    diff = node - map_dict['goal']
    diff[0] = angle_dist(diff[0])

    diff = diff.squeeze()

    return diff






