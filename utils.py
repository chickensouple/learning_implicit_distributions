import numpy as np
import math
import dubins


def map_sampler(map_info):
    rand_idx = np.random.random(2) * map_info['map'].shape
    while (map_collision_check(map_info['map'], rand_idx)):
        rand_idx = np.random.random(2) * map_info['map'].shape

    return rand_idx

def map_sampler_goal_bias(map_info, eps):
    rand = np.random.random()
    if rand > eps:
        rand_idx = map_sampler(map_info)
    else:
        rand_idx = map_info['goal']

    return rand_idx

def map_collision_check(arr, path, return_num_coll=False):
    # returns True, if collision
    num_collision_checks = 0
    collision = False
    if type(path) is list:
        for node in path:
            if _map_collision_check(arr, node):
                num_collision_checks += 1
                collision = True
                break
    else:
        collision = _map_collision_check(arr, path)
        num_collision_checks += 1

    if return_num_coll:
        return collision, num_collision_checks
    else:
        return collision

def map_closest_obstacle(precomputed, point):
    if np.sum(precomputed.shape) == 0:
        return float('inf')

    cx = np.maximum(np.minimum(point[0], precomputed[:, 0] + 1), precomputed[:, 0])
    cy = np.maximum(np.minimum(point[1], precomputed[:, 1] + 1), precomputed[:, 1])
    dists = np.sqrt(np.square(cx - point[0]) + np.square(cy - point[1]))

    return np.min(dists)


def map_obst_precompute(arr):
    idx = np.where(arr == 0)
    idx = [np.array([idx[0][i], idx[1][i]]) for i in range(len(idx[0]))]
    return np.array(idx)


def map_inbounds(arr, node):
    idx = np.array(np.floor(node), dtype=np.int)
    if (idx[0] < 0) or (idx[0] >= (arr.shape[0])):
        return False

    if (idx[1] < 0) or (idx[1] >= (arr.shape[1])):
        return False

    return True


# returns true of collision
def _map_collision_check(arr, node):
    idx = np.array(np.floor(node), dtype=np.int)
    if (idx[0] < 0) or (idx[0] >= (arr.shape[0])):
        return True

    if (idx[1] < 0) or (idx[1] >= (arr.shape[1])):
        return True

    return arr[idx[0], idx[1]] == 0








def get_feat_empty(point, tree, map_info):
    feat = np.array([np.linalg.norm(point - map_info['goal'])])
    return feat

def get_feat_flytrap(point, tree, map_info):
    # goal based features
    goal_delta = np.linalg.norm(map_info['goal'] - point)

    # tree based features
    closest_idx = tree.closest_idx(point, l2_dist)
    closest_node = tree.node_states[closest_idx]
    tree_delta = np.linalg.norm(closest_node - point)

    obst_delta = map_closest_obstacle(map_info['precomputed'], closest_node)

    feat = np.array([tree_delta - obst_delta])
    return feat

def get_feat_default(point, tree, map_info):
    return point










def l2_dist(node_from, node_to):
    return np.linalg.norm(node_from - node_to, axis=1)

def holonomic_steer(node_from, node_to, extend_length=1.5, discrete=0.2):
    diff = node_to - node_from
    diff_norm = np.linalg.norm(diff)
    if diff_norm < 1e-6:
        return [node_from], 0
    if diff_norm > extend_length:
        diff_vec = diff / diff_norm
        new_node = node_from + extend_length * diff_vec
    else:
        new_node = node_from + diff

    diff = new_node - node_from
    diff_norm = np.linalg.norm(diff)
    diff_vec = discrete * diff / diff_norm
    
    num_pts = int(np.floor(diff_norm / discrete))
    path = [node_from+diff_vec*(i+1) for i in range(num_pts)]

    if math.fabs((diff_norm/discrete)-num_pts) > 1e-6 or num_pts == 0:
        path.append(new_node)

    return path, diff_norm

def l2_goal_region(node, goal):
    return np.linalg.norm(goal - node) < 0.5

def dubins_dist(node_from, node_to, radius=1.0):
    res = np.zeros(len(node_from))

    for idx, node in enumerate(node_from):
        dist = dubins.path_length(node, node_to, radius)
        res[idx] = dist
    return res

def dubins_steer(node_from, node_to, extend_length=1.5, discrete=0.2, radius=1.0):
    path = dubins.path_sample(node_from, node_to, radius, discrete)[0]

    length = dubins.path_length(node_from, node_to, radius)
    if discrete > length:
        discrete = discrete - 1e-5

    if extend_length < length:
        num_pts = int(np.floor(extend_length / discrete))
        path = [np.array(node) for node in path[1:1+num_pts]]
        return path, extend_length
    else:
        path = [np.array(node) for node in path[1:]]
        path.append(node_to)
        return path, length

def dubins_goal_region(node, goal):
    dist = np.linalg.norm(node[0:2] - goal[0:2])
    if dist > 0.5:
        return False
    angdist = wrap_pi(goal[2] - node[2])
    if math.fabs(angdist) > 0.4:
        return False
    return True

def get_disc_rewards(rewards, gamma):
    disc_rewards = np.zeros(rewards.shape)
    n = len(disc_rewards) - 1
    for idx, r in enumerate(reversed(rewards)):
        if idx == 0:
            disc_rewards[n-idx] = r
        else:
            disc_rewards[n-idx] = disc_rewards[n-idx+1] * gamma + r
    return disc_rewards


# general utils
def wrap_pi(ang):
    while ang < -np.pi:
        ang += 2 * np.pi
    while ang >= np.pi:
        ang -= 2 * np.pi
    return ang


class RunningStats(object):
    def __init__(self, N):
        self.N = N
        self.vals = []
        self.num_filled = 0

    def push(self, val):
        if self.num_filled == self.N:
            self.vals.pop(0)
            self.vals.append(val)
        else:
            self.vals.append(val)
            self.num_filled += 1

    def push_list(self, vals):
        num_vals = len(vals)

        self.vals.extend(vals)
        self.num_filled += num_vals
        if self.num_filled >= self.N:
            diff = self.num_filled - self.N
            self.num_filled = self.N
            self.vals = self.vals[diff:]


    def get_mean(self):
        return np.mean(self.vals[:self.num_filled])
        
    def get_std(self):
        return np.std(self.vals[:self.num_filled])

class RunningAverage(object):
    def __init__(self, N):
        self.N = N
        self.vals = []
        self.num_filled = 0

    def push(self, val):
        if self.num_filled == self.N:
            self.vals.pop(0)
            self.vals.append(val)
        else:
            self.vals.append(val)
            self.num_filled += 1

    def get(self):
        return float(sum(self.vals)) / self.num_filled


if __name__ == '__main__':

    rewards = np.array([1, 2, 3, 4, 5, 6.])
    print get_disc_rewards(rewards, 0.1)


    # rand = np.array([4., 2., 0.6])
    # closest = np.array([3., 6.2, -3.8])

    # # path = dubins.path_sample(closest, rand, 1., 0.2)
    # # print path
    # # print dubins.path_length(closest, rand, 1.)
    # path, length = dubins_new_config(rand, closest)
    # # path, length = dubins_new_config(rand, closest, extend_length=np.Inf)
    # print length
    # print path