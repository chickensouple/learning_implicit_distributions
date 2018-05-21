import numpy as np
import math
from cart_pole import CartPole
from ode_numerical import *

cartpole = CartPole()


def cartpole_visualize(state, map_info):
    pass


def cartpole_generate_map():
    # (x, y, r)
    balls = [(np.array([5, -2]), 1.5),
             (np.array([6, 2]), 1.5)]

    map_dict = {}
    map_dict['start'] = np.array([[0, 0, 0, 0]]).T
    map_dict['goal'] = np.array([[10, np.pi, 0, 0]]).T
    map_dict['obst'] = balls
    return map_dict

def cartpole_sample():
    x = np.random_sample((4, 1))
    x[0,0] = x[0,0] * 10
    x[1,0] = x[1,0] * 6 - 3
    x[2,0] = x[2,0] * 2*np.pi
    x[3,0] = x[3,0] * 4 - 2
    return x


def _cartpole_collision_check(map_info, node):
    collision_pts = [np.array([node[0], 0]),
        np.array([node[0]+0.5, 0]),
        np.array([node[0]-0.5, 0]),
        np.array([node[0]+np.sin(node[1]), np.cos(node[1])])
    ]
    
    for ball in map_info['obst']:
        for pt in collision_pts:
            if np.linalg.norm(ball[0] - pt) < ball[1]:
                return True
    return False

def cartpole_collision_check(map_info, path, return_num_coll=False):
    # returns True, if collision
    num_collision_checks = 0
    collision = False
    if type(path) is list:
        for node in path:
            if _cartpole_collision_check(map_info, node):
                num_collision_checks += 1
                collision = True
                break
    else:
        collision = _cartpole_collision_check(map_info, path)
        num_collision_checks += 1

    if return_num_coll:
        return collision, num_collision_checks
    else:
        return collision


def cartpole_steer(node_from, node_to, extend_length=1.5, discrete=0.2):
    global cartpole
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

        path, _ = ode_solver(cartpole.get_diff_eq(), 
            controller,
            x0=node_from, 
            t0=0,
            dt=np.ones(num_dt) * dt)
        xf = path[:, -1]

        dist = np.linalg.norm(xf - node_to)
        if dist < min_dist:
            min_dist = dist
            min_path = path

    return min_path

if __name__ == '__main__':
    node_from = np.array([[0, 0, 0, 0]]).T
    node_to = np.array([[-0.2, 0.1, 0, 0]]).T

    path = cartpole_steer(node_from, node_to)







