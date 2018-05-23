import numpy as np
import math
from . model_base import ModelBase

class Pendulum(ModelBase):
    """
    Models a simple pendulum driven by a motor
    
    state is x = [theta, theta_dot]
    where theta is 0 when pointing upwards
    control input is u = [torque]
    """
    def __init__(self, length=1., mass=0.2, m_type='point', mu=0.05, max_torque=1., **kwargs):
        """
        Initializes Pendulum
        
        Args:
            length (float, optional): length of pendulum in m
            mass (float, optional): mass of pendulum in kg
            m_type (str, optional): type of pendulum. 
            'point' for a point mass at end of pendulum
            'rod' for a solid uniform rod
            mu (float, optional): friction coefficient.
            friction generates torque according to -angular_vel * mu
            max_torque (float, optional): maximum torque that can be applied in N*m
            **kwargs: Description
        """
        control_limits = [np.array([-max_torque]), np.array([max_torque])]
        super(Pendulum, self).__init__(2, 1, control_limits, **kwargs)
        self.length = length
        self.mass = mass
        self.m_type = m_type
        self.mu = mu
        self.x = np.array([[math.pi, 0]]).T

        # compute center of mass and moment of inertia
        if m_type == 'point':
            self.com = length
            self.inertia = mass * length * length
        elif m_type == 'rod':
            self.com = 0.5 * length
            self.inertia = mass * length * length / 3
        else:
            raise Exception('Not a valid m_type')

    def diff_eq(self, x, u):
        u = self._check_and_clip(x, u)
        torque = u[0]

        grav = 9.81
        grav_torque = self.mass * grav * self.com * np.sin(x[0])
        fric_torque = -x[1] * self.mu

        x_dot = np.zeros(x.shape)
        x_dot[0] = x[1]
        x_dot[1] = (grav_torque + torque + fric_torque) / self.inertia
        return x_dot

    def after_step(self):
        # wrap angle to [-pi, pi)
        while self.x[0] < -np.pi:
            self.x[0] += 2 * np.pi
        while self.x[0] >= np.pi:
            self.x[0] -= 2 * np.pi


    def get_energy(self):
        kinetic = 0.5 * self.inertia * self.x[1] * self.x[1]
        potential = self.mass * 9.81 * self.com * (math.cos(self.x[0]) - 1)
        total = kinetic + potential
        return total


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    env = Pendulum()
    env.set_state(np.array([[0.1, 0]]).T)
    controls = np.array([[0., 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]).T
    states = np.zeros((env.state_dim, len(controls)))
    for idx, control in enumerate(controls):
        state, _ = env.step(control)
        states[:, idx] = state.squeeze()

    plt.scatter(states[0, :], states[1, :])
    plt.show()


