from forward_kinematics import kinematics_forward_l_default
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class ArmVisualize(object):
    def __init__(self, ax=None):
        if ax == None:
            fig = plt.figure()
            self.ax = fig.add_subplot(111, projection='3d')
        else:
            self.ax = ax

    def plot(self, q):
        T, pt = kinematics_forward_l_default(q)
        pt = np.array(pt)
        self.ax.plot(pt[:, 0], pt[:, 1], pt[:, 2], c='b')
        self.ax.scatter(pt[:, 0], pt[:, 1], pt[:, 2], c='b')
        self.ax.scatter(pt[0, 0], pt[0, 1], pt[0, 2], c='r')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')


        min_axes = []
        max_axes = []
        bot, top = self.ax.get_xlim()
        min_axes.append(bot)
        max_axes.append(top)
        bot, top = self.ax.get_ylim()
        min_axes.append(bot)
        max_axes.append(top)
        bot, top = self.ax.get_zlim()
        min_axes.append(bot)
        max_axes.append(top)

        bot = min(min_axes)
        top = max(max_axes)
        self.ax.set_xlim((bot, top))
        self.ax.set_ylim((bot, top))
        self.ax.set_zlim((bot, top))



if __name__ == '__main__':
    # q = [1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481]
    
    # T, pt = kinematics_forward_l_default(q)

    # pt = np.array(pt)

    # armv = ArmVisualize()
    # armv.plot(q)
    # plt.show()

    def get_arm_length(pts):
        length = 0
        for i in range(1, len(pts)):
            length += np.linalg.norm(pts[i] - pts[i-1])
        return length

    q1 = [0, 0.0, 0., 0., 0., 0., 0.]
    q2 = [1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481]
    

    _, pts1 = kinematics_forward_l_default(q1)
    _, pts2 = kinematics_forward_l_default(q2)

    print "l1:", get_arm_length(pts1)
    print "l2:", get_arm_length(pts2)

    armv = ArmVisualize()
    armv.plot(q1)
    armv.plot(q2)
    plt.show()
