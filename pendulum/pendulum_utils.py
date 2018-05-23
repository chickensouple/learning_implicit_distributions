import numpy as np
import math
from pendulum import Pendulum
from ode_numerical import *



def pendulum_generate_map():
    balls = []

    map_dict = {}
    map_dict['start'] = np.array([[]])

