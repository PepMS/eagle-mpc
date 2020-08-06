import multicopter_mpc
import numpy as np
import pinocchio

def wayPointLitToStateArray(wp_list):
    for idx, wp in enumerate(wp_list):
        state = pinocchio.SE3ToXYZQUAT(wp.pose)
        state = np.append(state, wp.velocity)
        if 'states' in locals():
            states = np.append(states, np.array([state]).T, axis = 1)
            time = np.append(time, time[-1] + wp.time)
        else:
            states = np.array([state]).T
            time = np.array([wp.time])
    
    return states, time


        
