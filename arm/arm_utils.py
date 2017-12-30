import numpy as np


def arm_map_create(pointcloud, start, goal):
    angle_lower_limits = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi])
    angle_upper_limits = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi])
    map_dict = {'map': pointcloud, 
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

def arm_steer(node_from, node_to, discr=0.15, delta_lim=0.5):
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

def arm_collision_check(map_info, path, return_num_coll):
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

def _arm_collision_check(map_info, joint_angles):
    if np.any(joint_angles < map_info['lower_limits']) or \
        np.any(joint_angles > map_info['upper_limits']):
        return True

    # TODO
    return False

def arm_feat(joint, trees, map_info):
    # TODO
    return 0


if __name__ == '__main__':
    q = np.array([1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481])
    target_q = np.array([0., 0., 0., 0., 0., 0., 0.])


    map_dict = arm_map_create(None, q, target_q)

    print "goal: ", arm_goal_region(q, target_q)
    print "steer: ", arm_steer(q, target_q)
    print "random: ", arm_random_sample(map_info=map_dict, eps=0.5)
    print "dist: ", arm_dist_func(q, np.array([target_q]))

    path, dist = arm_steer(q, target_q)
    print "collision: ", arm_collision_check(map_dict, path)
