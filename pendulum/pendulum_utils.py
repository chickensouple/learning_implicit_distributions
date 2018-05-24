import numpy as np
import math
from pendulum import Pendulum
from ode_numerical import *

pendulum = Pendulum()

def pendulum_generate_map():
    balls = []

    map_dict = {}
    map_dict['start'] = np.array([[0, 0]]).T
    map_dict['goal'] = np.array([[np.pi, 0]]).T

    return map_dict

def pendulum_sample(map_info):
    x = np.random.random_sample((2, 1))
    x[0] = x[0] * 2*np.pi
    x[1] = x[1] * 4 - 2
    return x

def pendulum_collision_check(map_info, node):
    return False

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

        dist = np.linalg.norm(xf - node_to.squeeze())

        if dist < min_dist:
            min_dist = dist
            min_path = copy.deepcopy(path)

    path_list = []
    for i in range(min_path.shape[1]):
        path_list.append(np.array([min_path[:, i]]).T)
    return path_list, path.shape[1]



def pendulum_dist(node_from, node_to):
    node_from_cp = np.squeeze(node_from).T
    node_from_cp = np.reshape(node_from_cp, (2, -1))
    pass

def pendulum_goal(a, b):
    pass

def pendulum_feat(a, b, c):
    pass



