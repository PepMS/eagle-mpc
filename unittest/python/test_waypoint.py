import numpy as np
import crocoddyl
# import eigenpy
import pinocchio

import multicopter_mpc

knots = 24
pos = np.array([1, 1, 1])
quat = pinocchio.Quaternion(1, 0, 0, 0)
vel = np.array([1, 1, 1])
rate = np.array([1, 2, 1])

wp_1 = multicopter_mpc.WayPoint(knots, pos, quat)
wp_2 = multicopter_mpc.WayPoint(knots, pos, quat, vel, rate)

wp_lst = []
wp_lst.append(wp_1)
wp_lst.append(wp_2)

for idx, wp in enumerate(wp_lst):
    print("Printing Waypoint ", idx)
    print("Knots", wp.knots)
    print("Pose \n", wp.pose)
    if (wp.velocity is not None):
        print("Velocity \n", wp.velocity)
