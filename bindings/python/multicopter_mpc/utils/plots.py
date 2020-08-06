import numpy as np
import matplotlib.pyplot as plt

from multicopter_mpc.utils.tools import wayPointLitToStateArray


def PlotStates(xs, dt, wp_list=None):
    PlotPosition(xs, dt, wp_list)
    PlotAttitude(xs, dt, wp_list)
    PlotVelocityLin(xs, dt, wp_list)
    PlotVelocityAng(xs, dt, wp_list)


def PlotPosition(xs, dt, wp_list=None):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))
    plotTrajectory(xs, 4e-3, axs, 0, 3, names=['X pos', 'Y pos', 'Z pos'], wp_list=wp_list)
    # plt.vlines(wp_list[0].knots*dt)
    # plt.show()


def PlotAttitude(xs, dt, wp_list=None):
    fig, axs = plt.subplots(4, 1, figsize=(15, 10))
    plotTrajectory(xs,
                   4e-3,
                   axs,
                   3,
                   7,
                   names=['X quat', 'Y quat', 'Z quat', 'W quat'], wp_list=wp_list)
    # plt.show()


def PlotVelocityLin(xs, dt, wp_list=None):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))
    plotTrajectory(xs,
                   4e-3,
                   axs,
                   7,
                   10,
                   names=['X vel. lin.', 'Y vel. lin.', 'Z vel. lin.'], wp_list=wp_list)
    # plt.show()


def PlotVelocityAng(xs, dt, wp_list=None):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))
    plotTrajectory(xs,
                   4e-3,
                   axs,
                   10,
                   13,
                   names=['X vel. ang.', 'Y vel. ang.', 'Z vel. ang.'], wp_list=wp_list)
    # plt.show()


def PlotMotorSpeed(us, dt, wp_list=None):
    fig, axs = plt.subplots(4, 1, figsize=(15, 10))
    plotTrajectory(us, 4e-3, axs, 0, 4)


def plotTrajectory(data, dt, axs, row_init, row_end, names=None, wp_list=None):
    if isinstance(data, list):
        for d in data:
            knots = np.size(d, 1)
            t = np.arange(0, round(knots * dt, 4), dt)
            for i in range(row_end - row_init):
                axs[i].plot(t, d[row_init + i, :])
                if names is not None:
                    axs[i].set_title(names[i])
                if wp_list is not None:
                    wp_array, time_array = wayPointLitToStateArray(wp_list)
                    axs[i].plot(time_array, wp_array[row_init + i, :], 'r+')

    else:
        knots = np.size(data, 1)
        t = np.arange(0, round(knots * dt, 4), dt)
        for i in range(row_end - row_init):
            axs[i].plot(t, data[row_init + i, :])
            if names is not None:
                axs[i].set_title(names[i])


def plotRPY(xs, dt, axs, wp_list=None):
    if isinstance(xs, list):
        for xs_ in xs:
            knots = np.size(xs_, 1)
            t = np.arange(0, round(knots * dt, 4), dt)
            Q = xs_[3:7, :]
            R = []
            P = []
            Y = []
            for q in Q.T:
                r, p, y = q2e(q, True)
                R.append(r)
                P.append(p)
                Y.append(y)
            axs[0].plot(t, R)
            axs[1].plot(t, P)
            axs[2].plot(t, Y)
    else:
        knots = np.size(xs, 1)
        t = np.arange(0, round(knots * dt, 4), dt)
        Q = xs[3:7, :]
        R = []
        P = []
        Y = []
        for q in Q.T:
            r, p, y = q2e(q, True)
            R.append(r)
            P.append(p)
            Y.append(y)
        axs[0].plot(t, R)
        axs[1].plot(t, P)
        axs[2].plot(t, Y)
    axs[0].set_title("Roll")
    axs[1].set_title("Pitch")
    axs[2].set_title("Yaw")


def plotWaypoints(wp_list, dt, ax):
    if wp_list is not None:
        y_lim = ax.get_ylim()
        k = 0
        for wp in wp_list:
            k = k + wp.knots * dt
            ax.vlines(k,
                      y_lim[0],
                      y_lim[1],
                      color='silver',
                      linestyle='dashed')


def q2e(q, deg=False):
    a = q[3]
    b = q[0]
    c = q[1]
    d = q[2]

    y1 = 2 * c * d + 2 * a * b
    x1 = a**2 - b**2 - c**2 + d**2
    z2 = -2 * b * d + 2 * a * c
    y3 = 2 * b * c + 2 * a * d
    x3 = a**2 + b**2 - c**2 - d**2

    R = np.arctan(y1 / x1)
    P = np.arcsin(z2)
    Y = np.arctan(y3 / x3)

    if deg:
        r2d = 180 / np.pi
        R = R * r2d
        P = P * r2d
        Y = Y * r2d

    return R, P, Y


def showPlots():
    plt.show()
