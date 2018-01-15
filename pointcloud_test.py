import scipy.io
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


if __name__ == '__main__':
    p1 = scipy.io.loadmat('pointcloud_data/points1.mat')['points_arm'][:, 0:3]

    print p1
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(p1[:, 0], p1[:, 1], p1[:, 2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

