import multicopter_mpc
import numpy as np
import pinocchio


def wayPointListToStateArray(wp_list):
    for idx, wp in enumerate(wp_list):
        state = pinocchio.SE3ToXYZQUAT(wp.pose)
        state = np.append(state, wp.velocity)
        if 'states' in locals():
            states = np.append(states, np.array([state]).T, axis=1)
            time = np.append(time, time[-1] + wp.time)
        else:
            states = np.array([state]).T
            time = np.array([wp.time])

    return states, time


def computeMissionStateError(xs, wp_list):
    if isinstance(xs, list):
        errors = []
        for xs_ in xs:
            traj_idx = 0
            for wp_idx, wp in enumerate(wp_list):
                if wp_idx == 0:
                    knots = wp.knots
                else:
                    knots = wp.knots - 1

                for i in range(knots):
                    M = pinocchio.XYZQUATToSE3(xs_[:7, traj_idx + i])
                    e_pos = wp.pose.translation - M.translation
                    e_rot = pinocchio.log3(np.matmul(M.rotation.T, wp.pose.rotation))
                    e_vlin = wp.velocity.linear - xs_[7:10, traj_idx + i]
                    e_vang = wp.velocity.angular - xs_[10:13, traj_idx + i]
                    e = np.vstack((np.linalg.norm(e_pos), np.linalg.norm(e_rot),
                                   np.linalg.norm(e_vlin), np.linalg.norm(e_vang)))
                    error = np.hstack((error, e)) if 'error' in locals() else e

                traj_idx += knots

            errors.append(error)
            del error
    else:
        traj_idx = 0
        for wp_idx, wp in enumerate(wp_list):
            if wp_idx == 0:
                knots = wp.knots
            else:
                knots = wp.knots - 1

            for i in range(knots):
                M = pinocchio.XYZQUATToSE3(xs[:7, traj_idx + i])
                e_pos = wp.pose.translation - M.translation
                e_rot = pinocchio.log3(np.matmul(M.rotation.T, wp.pose.rotation))
                e_vlin = wp.velocity.linear - xs[7:10, traj_idx + i]
                e_vang = wp.velocity.angular - xs[10:13, traj_idx + i]
                e = np.vstack((np.linalg.norm(e_pos), np.linalg.norm(e_rot),
                               np.linalg.norm(e_vlin), np.linalg.norm(e_vang)))
                errors = np.hstack((errors, e)) if 'errors' in locals() else e

            traj_idx += knots

    return errors
