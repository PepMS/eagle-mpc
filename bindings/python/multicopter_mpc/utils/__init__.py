import pinocchio
import crocoddyl
import multicopter_mpc
import example_robot_data

import numpy as np
from multicopter_mpc.utils.mpc_controllers import CarrotMpc


def rev_enumerate(lname):
    return reversed(list(enumerate(lname)))


