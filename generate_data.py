import numpy as np
import matplotlib.pyplot as plt
from skimage import measure
import scipy
import math
from utils import *
from utils import _map_collision_check

def generate_map(nrows, ncols, nrooms, max_col_size, max_row_size):
    """
    Generates a random 
    map with rooms and connecting hallways
    :param nrows: number of rows in arr 
    :param ncols: number of columns in arr
    :param nrooms: number of rooms to generate
    :param max_col_size: maximum column size of a room
    :param max_row_size maximum row size of a room
    :return: numpy array representing the map
    1 is traversable region, 0 is not
    """
    arr = np.zeros((nrows, ncols), dtype=np.int8)

    for i in range(nrooms):
        rand_row_start = np.random.randint(nrows)
        rand_col_start = np.random.randint(ncols)

        rand_row_size = np.random.randint(max_row_size / 2, max_row_size)
        rand_col_size = np.random.randint(max_col_size / 2, max_col_size)

        arr[rand_row_start:rand_row_start + rand_row_size, rand_col_start:rand_col_start + rand_col_size] = 1

    labels = measure.label(arr)
    regions = measure.regionprops(labels)

    centroids = list()
    for region in regions:
        centroids.append(region.centroid)

    num_centroids = len(centroids)

    # get distances between every pair of centroids
    dists = scipy.spatial.distance.cdist(centroids, centroids)

    # get a distance that is greater than all current distances
    max_dist = np.max(dists) + 1

    # make sure upper triangle is at least max_dist so that when picking closest
    # pairs, we won't choose a diagonal element or a duplicate connection
    dists = dists + np.triu(np.ones((num_centroids, num_centroids))) * max_dist

    for i in range(num_centroids - 1):
        min_dist_idx = np.argmin(dists)
        min_dist_idx = np.unravel_index(min_dist_idx, dists.shape)

        # create a hallway between regionprops
        centroid1 = np.array(centroids[min_dist_idx[0]], dtype=np.int)
        centroid2 = np.array(centroids[min_dist_idx[1]], dtype=np.int)

        [row_centroid_1, row_centroid_2] = sorted([centroid1, centroid2], key=lambda x: x[0])
        [col_centroid_1, col_centroid_2] = sorted([centroid1, centroid2], key=lambda x: x[1])

        arr[row_centroid_1[0]:row_centroid_2[0] + 1, row_centroid_1[1]] = 1
        arr[row_centroid_2[0], col_centroid_1[1]:col_centroid_2[1] + 1] = 1

        dists[:, min_dist_idx[1]] += max_dist

    return arr


def generate_fly_trap(size, trap_size):
    arr = np.ones((size, size), dtype=np.int8)

    idx1 = int((size-trap_size) / 2)
    idx2 = idx1 + trap_size
    arr[idx1:idx2, idx1:idx2] = 0
    arr[idx1+1:idx2-1, idx1+1:idx2-1] = 1

    # creating hole in trap
    loc1 = np.random.randint(idx1+1, idx2-1)
    loc2 = np.random.choice([idx1, idx2-1])
    if np.random.random() > 0.5:
        arr[loc1, loc2] = 1
    else:
        arr[loc2, loc1] = 1

    # creating start and goal positions
    start = __gen_rand_point(arr)
    while np.any(start < idx1+1) or np.any(start >= idx2-1):
        start = __gen_rand_point(arr)

    goal = __gen_rand_point(arr)
    while np.any(goal >= idx1) and np.any(goal < idx2):
        goal = __gen_rand_point(arr)

    return arr, start, goal


def generate_fly_trap_fixed(size, trap_size):
    arr = np.ones((size, size), dtype=np.int8)

    # generate trap
    idx1 = int((size-trap_size) / 2)
    idx2 = idx1 + trap_size
    arr[idx1:idx2, idx1:idx2] = 0
    arr[idx1+1:idx2-1, idx1+1:idx2-1] = 1


    # generate tunnel
    middle = int((idx1 + idx2)/2)
    arr[middle-1, idx1+2:idx2] = 0
    arr[middle+1, idx1+2:idx2] = 0
    arr[middle, idx2-1] = 1

    # creating start and goal positions
    start = __gen_rand_point(arr)
    while np.any(start < idx1+1) or \
        np.any(start >= idx2-1) or \
        (start[0] >= middle-2 and start[0] <= middle+2):
        start = __gen_rand_point(arr)


    goal = __gen_rand_point(arr)
    while np.any(goal >= idx1) and np.any(goal < idx2):
        goal = __gen_rand_point(arr)

    return arr, start, goal


def __gen_rand_point(arr, dubins=False):
    if dubins:
        pose = np.zeros(3)
        pose[0:2] = np.random.random(2) * arr.shape
        pose[2] = np.random.random() * 2*math.pi
        return pose
    else:
        return np.random.random(2) * arr.shape

def __gen_path(arr, dubins=False):
    start = __gen_rand_point(arr, dubins)
    while map_collision_check(arr, start):
        start = __gen_rand_point(arr, dubins)

    goal = __gen_rand_point(arr, dubins)
    while map_collision_check(arr, goal):
        goal = __gen_rand_point(arr, dubins)

    return [start, goal]


def generate_data(map_type, dubins=False):
    if map_type == 'rooms':
        arr = generate_map(20, 20, nrooms=4, max_col_size=10, max_row_size=10)
        [start, goal] = __gen_path(arr, dubins)
    elif map_type == 'empty':
        arr = np.ones((20, 20))
        [start, goal] = __gen_path(arr, dubins)
    elif map_type == 'fly_trap':
        arr, start, goal = generate_fly_trap(21, 17)
    elif map_type == 'fly_trap_fixed_a':
        arr, start, goal = generate_fly_trap_fixed(51, 11) # Env A
    elif map_type == 'fly_trap_fixed_b':
        # arr, start, goal = generate_fly_trap_fixed(13, 11)
        arr, start, goal = generate_fly_trap_fixed(17, 15) # Env B
    else:
        raise Exception('Not a valid map type')
    data_dict = {'map': arr, 'start': start, 'goal': goal}
    return data_dict


if __name__ == '__main__':
    while 1:
        plt.cla()
        np.random.seed(0)
        data_dict = generate_data('fly_trap_fixed')
        im = data_dict['map']
        plt.imshow(im, interpolation='nearest', 
            extent=[0, im.shape[0], 0, im.shape[1]],
            origin='lower')
        goal = data_dict['goal']
        plt.scatter(goal[1], goal[0], color='red')
        start = data_dict['start']
        plt.scatter(start[1], start[0], color='green')

        print 'start:', data_dict['start']
        print 'goal:', data_dict['goal']
        plt.show(block=False)
        input_str = raw_input("(q) for quit or <filename>.npy to save: ")
        if input_str == 'q':
            exit()
        else:
            if '.npy' in input_str:
                np.save('map.npy', arr)
                print("saved to " + input_str)



