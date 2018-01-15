import scipy.io
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import arm

if __name__ == '__main__':
    p5 = scipy.io.loadmat('pointclouddata/processed_5.mat')
    p5 = p5['save_struct'][0, 0]
    points = p5['points']

    q1 = [0, 0.0, 0., 0., 0., 0., 0.]
    q2 = [1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481]

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

