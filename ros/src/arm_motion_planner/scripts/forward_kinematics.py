import numpy as np

def modified_denavit_hartenberg(alpha, a, theta, d):
    T = np.zeros((4, 4))
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    T[0, 0] = ct
    T[0, 1] = -st
    T[0, 3] = a

    T[1, 0] = st*ca
    T[1, 1] = ct*ca
    T[1, 2] = -sa
    T[1, 3] = -sa*d

    T[2, 0] = st*sa
    T[2, 1] = ct*sa
    T[2, 2] = ca
    T[2, 3] = ca*d

    T[3, 3] = 1

    return T

def translate_mat(t):
    T = np.eye(4)
    T[0:3, -1] = t
    return T

def kinematics_forward_l_arm_7(q, 
    handOffsetX, 
    handOffsetY, 
    handOffsetZ,
    shoulderOffsetY,
    shoulderOffsetZ,
    upperArmLengthL,
    lowerArmLengthL,
    elbowOffsetX):
    T = np.eye(4)
    T = np.dot(T, translate_mat(np.array([0, shoulderOffsetY, shoulderOffsetZ])))
    # arm pt 0
    pt = []
    pt.append(T[0:3, -1])

    T = np.dot(T, modified_denavit_hartenberg(-np.pi*0.5, 0, q[0], 0))
    T = np.dot(T, modified_denavit_hartenberg(np.pi*0.5, 0, np.pi*0.5+q[1], 0))
    T = np.dot(T, modified_denavit_hartenberg(np.pi*0.5, 0, np.pi*0.5+q[2], upperArmLengthL))
    # arm pt 1
    pt.append(T[0:3, -1])

    T = np.dot(T, modified_denavit_hartenberg(np.pi*0.5, elbowOffsetX, q[3], 0))
    # arm pt 2
    pt.append(T[0:3, -1])

    T = np.dot(T, modified_denavit_hartenberg(-np.pi*0.5, -elbowOffsetX, -np.pi*0.5+q[4], lowerArmLengthL))
    # arm pt 3
    pt.append(T[0:3, -1])

    T = np.dot(T, modified_denavit_hartenberg(-np.pi*0.5, 0, q[5], 0))
    T = np.dot(T, modified_denavit_hartenberg(np.pi*0.5, 0, q[6], 0))
    T = np.dot(T, modified_denavit_hartenberg(-np.pi*0.5, 0, -np.pi*0.5, 0))

    T = np.dot(T, translate_mat(np.array([handOffsetX, -handOffsetY, handOffsetZ])))
    # arm pt 4
    pt.append(T[0:3, -1])

    return T, pt


if __name__ == '__main__':
    handOffsetX = 0.150
    handOffsetY = 0
    handOffsetZ = 0

    shoulderOffsetY = 0.234
    shoulderOffsetZ = 0.165

    upperArmLengthL = 0.261
    lowerArmLengthL = 0.252

    elbowOffsetX =   0.030

    q = [1.0502, -0.0480, 0.1047, -1.4513, 1.4258, -0.4063, -1.4481]

    T, pt = kinematics_forward_l_arm_7(q,
        handOffsetX,
        handOffsetY,
        handOffsetZ,
        shoulderOffsetY,
        shoulderOffsetZ,
        upperArmLengthL,
        lowerArmLengthL,
        elbowOffsetX)

    print pt


