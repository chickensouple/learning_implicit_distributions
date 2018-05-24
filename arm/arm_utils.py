import numpy as np
import sklearn.neighbors
from forward_kinematics import *
import copy

def arm_map_create(pointcloud_data, start, goal):
    # pointcloud is N by 3
    kdtree = sklearn.neighbors.KDTree(pointcloud_data['points'])

    # angle_lower_limits = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi])
    # angle_upper_limits = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi])

    angle_lower_limits = np.array([-90, -175, -90, -160, -180, -87, -180.]) * np.pi / 180
    angle_upper_limits = np.array([180, 87, 135, 0, 180, 87, 180]) * np.pi / 180

    map_dict = {'kdtree': kdtree, 
    'means': pointcloud_data['means'],
    'sigmas': pointcloud_data['sigmas'],
    'start': start, 
    'goal': goal,
    'lower_limits': angle_lower_limits,
    'upper_limits': angle_upper_limits}
    return map_dict

def constrain_angles_two_pi(angles):
    return np.mod(angles, 2*np.pi)

def arm_goal_region(point, goal):
    dist = constrain_angles_two_pi(point - goal)
    mask = dist > np.pi
    dist[mask] = 2*np.pi - dist[mask] 
    dist = np.linalg.norm(dist)

    return dist < 0.2

def arm_steer(node_from, node_to, discr=0.08, delta_lim=0.01):
    delta = constrain_angles_two_pi(node_to - node_from)
    mask = delta > np.pi
    delta[mask] = -(2*np.pi - delta[mask])

    dist = np.linalg.norm(delta)

    # largest distance in angle
    max_delta = np.max(np.abs(delta))

    if max_delta < 1e-6:
        new_delta = delta * 1
    else:
        new_delta = delta * min(delta_lim / max_delta, 1)

    num_discr = int(np.ceil(min(max_delta, delta_lim) / discr))
    path = []
    if num_discr <= 1:
        path.append(node_from + new_delta)
    else:
        alphas = np.linspace(discr, 1., num_discr)
        for alpha in alphas:
            path.append(node_from + new_delta * alpha)

    return path, dist

def arm_random_sample(map_info, eps=0):
    rand = np.random.random()
    if rand > eps:
        scale = map_info['upper_limits'] - map_info['lower_limits']
        num_joints = len(map_info['upper_limits'])
        sample = np.random.random_sample(num_joints) * scale + \
            map_info['upper_limits']
    else:
        sample = map_info['goal']

    return sample

def arm_dist_func(node_from, node_to):
    dist = constrain_angles_two_pi(node_from - node_to)
    mask = dist > np.pi
    dist[mask] = 2*np.pi - dist[mask] 

    dist = np.linalg.norm(dist, axis=1)
    return dist

def arm_collision_check(map_info, path, return_num_coll=False):
    num_collision_checks = 0
    collision = False
    if type(path) is list:
        for node in path:
            if _arm_collision_check(map_info, node):
                num_collision_checks += 1
                collision = True
                break
    else:
        collision = _arm_collision_check(map_info, path)
        num_collision_checks += 1

    if return_num_coll:
        return collision, num_collision_checks
    else:
        return collision


def _linspace(pt1, pt2, num_pts):
    step = (pt2 - pt1) / num_pts

    curr_pt = copy.deepcopy(pt1)
    pts = []
    pts.append(pt1)
    for i in range(num_pts):
        curr_pt += step
        pts.append(copy.deepcopy(curr_pt))

    return pts

def _arm_collision_check(map_info, joint_angles):
    if np.any(joint_angles < map_info['lower_limits']) or \
        np.any(joint_angles > map_info['upper_limits']):
        return True

    # TODO: may need more points?
    T, pts = kinematics_forward_l_default(joint_angles)


    pts = _linspace(pts[0], pts[1], 5) + \
        _linspace(pts[2], pts[3], 5) + \
        _linspace(pts[3], pts[4], 4)[1:]

    for pt in pts:
        dists, indices = map_info['kdtree'].query(np.array([pt]), k=1)
        dist = np.asscalar(dists)
        if dist < 0.05:
            return True

    return False


def arm_feat_single(joint, trees, map_info):
    T, pts = kinematics_forward_l_default(joint)
    # extract features
    feats = []

    for i in [1, 3, 4]:
        pt = pts[i]
        dists, indices = map_info['kdtree'].query(np.array([pt]), k=1)
        dist = np.asscalar(dists)
        feats.append(dist)

    feats = np.array(feats)
    return feats

def arm_feat_bi(joint, trees, map_info, tree_idx):
    T, pts = kinematics_forward_l_default(joint)
    # extract features
    feats = []

    for i in [1, 3, 4]:
        pt = pts[i]
        dists, indices = map_info['kdtree'].query(np.array([pt]), k=1)
        dist = np.asscalar(dists)
        feats.append(dist)

    # tree distances
    _, dist = trees[tree_idx].closest_idx(joint, arm_dist_func, return_dist=True)
    feats.append(dist)

    feats = np.array(feats)
    return feats


def arm_feat_bi2(joint, trees, map_info, tree_idx):
    T, pts = kinematics_forward_l_default(joint)
    # extract features
    feats = []

    for i in [3, 4]:
        pt = pts[i]
        dists, indices = map_info['kdtree'].query(np.array([pt]), k=1)
        dist = np.asscalar(dists)
        feats.append(dist)

    # tree distances
    _, dist = trees[tree_idx].closest_idx(joint, arm_dist_func, return_dist=True)
    feats.append(dist)

    _, dist = trees[1 - tree_idx].closest_idx(joint, arm_dist_func, return_dist=True)
    feats.append(dist)

    # goal distance
    goal_dist = arm_dist_func(joint, np.array([map_info['goal']]))
    feats.append(goal_dist)


    # 5 features

    feats = np.array(feats)
    return feats

def arm_feat_bi3(joint, trees, map_info, tree_idx):
    T, pts = kinematics_forward_l_default(joint)
    # extract features
    feats = []

    for i in [4]:
        pt = pts[i]
        dists, indices = map_info['kdtree'].query(np.array([pt]), k=1)
        dist = np.asscalar(dists)
        feats.append(dist)

    # tree distances
    _, dist = trees[tree_idx].closest_idx(joint, arm_dist_func, return_dist=True)
    feats.append(dist)

    _, dist = trees[1 - tree_idx].closest_idx(joint, arm_dist_func, return_dist=True)
    feats.append(dist)

    # goal distance
    goal_dist = arm_dist_func(joint, np.array([map_info['goal']]))
    feats.append(goal_dist)

    # 4 features
    feats = np.array(feats)
    return feats




if __name__ == '__main__':
    import scipy.io

    pointcloud = scipy.io.loadmat('../pointclouddata/processed_2.mat')
    pointcloud = pointcloud['save_struct'][0, 0]

    q = np.array([1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481])
    target_q = np.array([0., 0., 0., 0., 0., 0., 0.])

    map_dict = arm_map_create(pointcloud, q, target_q)

    print "goal: ", arm_goal_region(q, target_q)
    print "steer: ", arm_steer(q, target_q)
    print "random: ", arm_random_sample(map_info=map_dict, eps=0.5)
    print "dist: ", arm_dist_func(q, np.array([target_q]))

    path, dist = arm_steer(q, target_q)
    print "collision: ", arm_collision_check(map_dict, path)


    # points = np.array([[0., 0., 2.],
    #     [0, 0, 0],
    #     [-2, 1, 1],
    #     [4, 2, 1]])

    # kdtree = sklearn.neighbors.KDTree(points)
    # query = kdtree.query(np.array([[0, 0, 0], 
    #     [-2, 1, 1]]), k=1)
    # # query = kdtree.query(np.array([0, 0, 0]), k=2)
    # print query


