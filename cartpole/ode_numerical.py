import numpy as np
import copy

def ode_solver(f, controller, x0, t0, dt, stype='runge_kutta4'):
    """
    numerically computes ode solution to a time invariant system
    
    Args:
        f (function): ode to solve, x_dot = f(x, u)
        controller (function): controller, u = g(x, t)
        x0 (numpy): a state_size by 1 numpy array for initial state
        t0 (array): initial time
        dt (TYPE): a length N array of timesteps to integrate over
        stype (str, optional): type of solver, 'euler' or 'runge_kutta4'
    
    Returns:
        x, t: 
        where x is a state_size by N array of states
        t is a length N array of time
    """
    if stype == 'euler':
        func = euler
    elif stype == 'runge_kutta4':
        func = runge_kutta4

    f_new = lambda t, x: f(x, controller(x, t))

    n = len(dt)

    x = np.zeros((len(x0), n+1))
    t = np.zeros(n+1)
    x[:, 0] = x0.squeeze()
    t[0] = t0
    for i in range(1, n+1):
        x_prev = np.array([x[:, i-1]]).T
        x[:, i] = func(f_new, x_prev, t[i-1], dt[i-1]).squeeze()
        t[i] = t[i-1] + dt[i-1]
    return x, t

def ode_solver_once(f, x, u, dt, stype='runge_kutta4'):
    """
    Numerically computes solution to a time invariant ode with control input
    
    Args:
        f (function): ode to solve, x_dot = f(x, u)
        x (numpy array): a state_size by 1 numpy array for state
        u (numpy array): a control_size by 1 numpy array for control input
        dt (float): time step to integrate over
        stype (str, optional): type of solver, 'euler' or 'runge_kutta4'
    
    Returns:
        numpy array: state_size by 1 numpy array for new state
    """
    if stype == 'euler':
        func = euler
    elif stype == 'runge_kutta4':
        func = runge_kutta4

    f_2 = lambda t, x: f(x, u)
    return func(f_2, x, 0, dt)

def ode_solver_once_adaptive(f, x, u, dt, stype='runge_kutta4'):
    """
    Numerically computes solution to a time invariant ode with control input
    
    Args:
        f (function): ode to solve, x_dot = f(x, u)
        x (numpy array): a state_size by 1 numpy array for state
        u (numpy array): a control_size by 1 numpy array for control input
        dt (float): time step to integrate over
        stype (str, optional): type of solver, 'euler' or 'runge_kutta4'
    
    Returns:
        numpy array: state_size by 1 numpy array for new state
    """
    if stype == 'euler':
        func = euler
    elif stype == 'runge_kutta4':
        func = runge_kutta4

    f_2 = lambda t, x: f(x, u)

    large_step = func(f_2, x, 0, dt)
    small_step1 = func(f_2, x, 0, dt*0.5)
    small_step2 = func(f_2, small_step1, 0, dt*0.5)

    # calculate percentage different in step values
    diff = abs(np.linalg.norm(small_step2 - large_step) / np.linalg.norm(large_step))

    if diff < 1e-5:
        return small_step2

    step1 = ode_solver_once_adaptive(f, x, u, dt*0.5, stype=stype)
    step2 = ode_solver_once_adaptive(f, step1, u, dt*0.5, stype=stype)
    return step2


def euler(f, x, t, dt):
    # integrates an ode x_dot = f(t, x)
    # over one time step dt
    # returns x(t+dt)
    x_new = x + f(t, x) * dt
    return x_new

def runge_kutta4(f, x, t, dt):
    # integrates an ode x_dot = f(t, x)
    # over one time step dt
    # returns x(t+dt)
    k1 = f(t, x)
    k2 = f(t, x+k1*dt*0.5)
    k3 = f(t, x+k2*dt*0.5)
    k4 = f(t, x+k3*dt)

    slope = (1.0 / 6) * (k1 + 2*(k2 + k3) + k4)
    x_new = x + slope * dt
    return x_new


def step_controller(u, dt):
    """
    Generates function u(t) that returns the ith elemnt of u
    at the ith timestep for dt amount of time, creating a step function for u
    
    Args:
        u (array): array of control inputs
        dt (float): timestep size
    """
    def func(x, t, u=copy.deepcopy(u), dt=dt):
        n = len(u)
        if t < 0 or t >= n * dt:
            return 0.
        idx = int(np.floor(t / dt))

        return u[idx]
    return func

def constant_controller(u):
    """
    Generates a function u(t) that returns a constant value
    """
    return lambda x, t: u


if __name__ == '__main__':
    from dubins_car import DubinsCar
    import matplotlib.pyplot as plt
    car = DubinsCar(1., 1.)

    dt = 0.1
    controls = np.array([[-0.2, -0.2, -0., 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]]).T
    control_func = step_controller(controls, dt)

    dts = np.ones(len(controls)) * dt
    x1, t1 = ode_solver(car.get_diff_eq(), control_func, car.x, 0, dts)
    x2, t2 = ode_solver(car.get_diff_eq(), control_func, car.x, 0, dts, stype='euler')

    plt.scatter(x1[0, :], x1[1, :], label='runge_kutta')
    plt.scatter(x2[0, :], x2[1, :], label='euler')

    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()

    plt.show()


