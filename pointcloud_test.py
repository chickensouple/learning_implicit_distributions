import scipy.io
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import arm
import math

if __name__ == '__main__':
    p5 = scipy.io.loadmat('pointclouddata/processed_11.mat')
    p5 = p5['save_struct'][0, 0]
    points = p5['points']


    qinit = np.array([90, 10, 0, -160, 0, 0, 0]) * math.pi / 180

    # q1 = [0, 0.0, 0., 0., 0., 0., 0.]
    # q2 = [0.5, -1., 0.1047, 0., 1.4258, -0.4063, -1.4481]

    q2 = np.array([85, -40, 0, -90, 90, 0, 0]) * math.pi / 180

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    armv = arm.ArmVisualize(ax)
    armv.plot(q2)


    rand_points = np.random.randint(points.shape[0], size=1000)
    ax.scatter(points[rand_points, 0], points[rand_points, 1], points[rand_points, 2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

    # print p1
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(p1[:, 0], p1[:, 1], p1[:, 2])
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # plt.show()

