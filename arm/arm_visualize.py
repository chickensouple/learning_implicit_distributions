from forward_kinematics import kinematics_forward_l_default
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class ArmVisualize(object):
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def plot(self, q):
        T, pt = kinematics_forward_l_default(q)
        pt = np.array(pt)
        self.ax.plot(pt[:, 0], pt[:, 1], pt[:, 2])
        self.ax.scatter(pt[0, 0], pt[0, 1], pt[0, 2], c='r')



if __name__ == '__main__':
    q = [1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481]

    T, pt = kinematics_forward_l_default(q)

    pt = np.array(pt)

    armv = ArmVisualize()
    armv.plot(q)
    plt.show()
