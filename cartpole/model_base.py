import numpy as np
# for when this file is imported outside of the top level package
from ode_numerical import ode_solver_once, ode_solver_once_adaptive

class ModelBase(object):
    """
    Base Class for Models that are described by
    Time Invariant Differential Equations, x_dot = f(x, u)
    
    Attributes:
        state_dim (int): number of state dimensions
        control_dim (int): number of control dimensions
        control_limits (list of 2 numpy arrays): lower and upper limits of control
        control_limis[0] is lower bound on control inputs
        control_limits[1] is upper bound on control inputs
        dt (TYPE): timestep used for step() function
    """
    def __init__(self, state_dim, control_dim, control_limits, dt=0.05):
        self.state_dim = state_dim
        self.control_dim = control_dim
        self.control_limits = control_limits
        self.dt = dt

        if len(self.control_limits) != 2:
            raise Exception('Control Limits should consist of lower and upper bounds')

        if len(self.control_limits[0]) != self.control_dim or \
            len(self.control_limits[1]) != self.control_dim:
            raise Exception('Control Limits not the right size')

        self.reset()

    def get_state(self):
        return self.x

    def set_state(self, x):
        if x.shape != self.x.shape:
            raise Exception("State is not proper shape")
        self.x = x

    def reset(self):
        self.x = np.zeros((self.state_dim, 1))
        self.T = 0

    def _check_and_clip(self, x, u):
        """
        Checks control and state shapes
        Checks a control input against control limits
        and clip the control input if needed
        
        Args:
            x (numpy arr): state
            u (numpy arr): control
        
        Returns:
            numpy arr: clipped control
        """
        if len(x) != self.state_dim:
            raise Exception("State is not proper shape")
        if len(u) != self.control_dim:
            raise Exception("Control input is not proper shape")

        u = np.clip(u, self.control_limits[0], self.control_limits[1])
        return u

    def diff_eq(self, x, u):
        """
        Differential equation to be implemented by subclass
        Needs to return x_dot

        Args:
            x (numpy array): array of length state_dim of state
            u (numpy array): array of length control_dim of control
        """
        raise Exception("Needs to be implemented")


    def get_linearization(self, x0, u0):
        """
        Linearize the model around (x0, u0)
        default behavior is to compute linearization with discrete intervals
        can override this function with specific analytic linearizations
        
        f(x) ~= f0 + A*(dx) + B*(du)

        Args:
            x0 (numpy arr): state to linearize around
            u0 (numpy arr): control to linearize around

        Returns:
            (list of numpy arrs): (f0, A, B)
        """
        f0 = self.diff_eq(x0, u0)

        dx = 0.01
        du = 0.01

        A = np.zeros((self.state_dim, self.state_dim))
        B = np.zeros((self.state_dim, self.control_dim))

        for i in range(self.state_dim):
            vec_dx = np.zeros((self.state_dim, 1))
            vec_dx[i] = dx
            new_f_x = self.diff_eq(x0 + vec_dx, u0)
            delta_f_x = (new_f_x - f0) / dx
            A[:, i] = delta_f_x.squeeze()

        for i in range(self.control_dim):
            vec_du = np.zeros((self.control_dim, 1))
            vec_du[i] = du
            new_f_u = self.diff_eq(x0, u0 + vec_du)
            delta_f_u = (new_f_u - f0) / du
            B[:, i] = delta_f_u.squeeze()

        return f0, A, B

    # def get_linearized_system(self, x0, u0):
    #     f0, A, B = self.get_linearization(x0, u0)

    #     A_new = np.zeros((A.shape[0]+1, A.shape[1]+1))
    #     A_new[:A.shape[0], :A.shape[1]] = A
    #     A_new[:-1, -1] = f0
    #     A_new[-1, -1] = 1

    #     B_new = np.zeros((B.shape[0]+1, B.shape[1]))
    #     B_new[:-1, :] = B

    #     return LTISystem(A_new, B_new, self.control_limits)

    def get_diff_eq(self):
        """
        returns a function to calculate diff_eq
        without needing to reference the object
        
        Returns:
            function: differential equation
        """
        func = lambda x, u: self.diff_eq(x, u)
        return func

    def step(self, u):
        f = self.get_diff_eq()
        # self.x = ode_solver_once(f, self.x, u, self.dt)
        self.x = ode_solver_once_adaptive(f, self.x, u, self.dt)
        self.T += self.dt
        self.after_step()
        return self.x, self.T

    def after_step(self):
        """
        Overload this function if you need to do some post processing
        after the ode solver is called
        """
        pass



