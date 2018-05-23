import numpy as np
import math
from cart_pole import CartPole
from ode_numerical import *
import matplotlib.pyplot as plt
import copy

cartpole = CartPole()


def cartpole_visualize(state, map_info):
    plt.cla()

    def plot_cart(state, **kwargs):
        center = np.array([state[0], 0])
        left = np.array([state[0]+0.5, 0])
        right = np.array([state[0]-0.5, 0])
        top = np.array([state[0]+np.sin(state[1]), -np.cos(state[1])])

        plt.plot([left[0], right[0]], [left[1], right[1]], **kwargs)
        plt.plot([center[0], top[0]], [center[1], top[1]], **kwargs)
        plt.xlim([-1, 11])
        plt.ylim([-6, 6 ])

    plot_cart(state, c='black')

    plot_cart(map_info['start'], c='g')
    plot_cart(map_info['goal'], c='b')

    for ball in map_info['obst']:
        circle = plt.Circle(ball[0], ball[1], color='r')
        plt.gca().add_artist(circle)

def cartpole_generate_map():
    # (x, y, r)
    # balls = [(np.array([5, -2]), 1.5),
    #          (np.array([6, 2]), 1.5)]
    balls = []

    map_dict = {}
    map_dict['start'] = np.array([[0, 0, 0, 0]]).T
    map_dict['goal'] = np.array([[5, np.pi, 0, 0]]).T
    map_dict['obst'] = balls
    return map_dict

def cartpole_sample(map_info):
    x = np.random.random_sample((4, 1))
    x[0,0] = x[0,0] * 10
    x[1,0] = x[1,0] * 6 - 3
    x[2,0] = x[2,0] * 2*np.pi
    x[3,0] = x[3,0] * 4 - 2
    return x


def _cartpole_collision_check(map_info, node):
    collision_pts = [np.array([node[0], 0]),
        np.array([node[0]+0.5, 0]),
        np.array([node[0]-0.5, 0]),
        np.array([node[0]+np.sin(node[1]), -np.cos(node[1])])
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
        u = np.random.random_sample()*20 - 10
        u = np.array([[u]])
        dt = 0.1
        num_dt = np.random.randint(1, 20)

        controller = constant_controller(u)

        path, _ = ode_solver(cartpole.get_diff_eq(), 
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


def cartpole_dist(node_from, node_to):
    node_from_cp = np.squeeze(node_from).T
    node_from_cp = np.reshape(node_from_cp, (4, -1))

    dists = np.linalg.norm(node_from_cp - node_to, axis=0)

    return dists


def cartpole_goal(a, b):
    dist = abs(a[0] - b[0])
    angular_dist = abs(a[1] - b[1])


    if dist < 0.1 and angular_dist < 0.15:
        return True
    else:
        return False


def cartpole_feat(a, b, c):
    return np.array([0])
    pass

if __name__ == '__main__':
    node_from = np.array([[0, 0, 0, 0]]).T
    node_to = np.array([[-0.2, 0.1, 0, 0]]).T

    path, dist = cartpole_steer(node_from, node_to)
    print path[-1]


    # map_dict = cartpole_generate_map()

    # u = np.array([[10]])
    # controller = constant_controller(u)
    # node_from = np.array([[0, 0, 0, 0]]).T
    # num_dt = 50
    # dt = 0.1
    # path, _ = ode_solver(cartpole.get_diff_eq(), 
    #     controller,
    #     x0=node_from, 
    #     t0=0,
    #     dt=np.ones(num_dt) * dt)

    # for i in range(path.shape[1]):
    #     state = path[:, i]
    #     cartpole_visualize(state, map_dict)
    #     plt.show(block=False)
    #     plt.pause(0.1)

    # state = np.array([[0, 0, 0, 0]]).T
    # cartpole_visualize(state, None)

    # plt.show()



