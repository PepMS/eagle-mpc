import numpy as np
import matplotlib.pyplot as plt


from multicopter_mpc.utils.tools import wayPointListToStateArray

colors = ['tab:blue', 'tab:orange', 'tab:red', 'tab:green']


def PlotStates(xs, time, wp_list=None, legend=None):
    PlotPosition(xs, time, wp_list, legend=legend)
    PlotAttitude(xs, time, wp_list, legend=legend)
    PlotVelocityLin(xs, time, wp_list, legend=legend)
    PlotVelocityAng(xs, time, wp_list, legend=legend)


def Plot3DTrajectory(xs, wp_list=None, subplot_axis=0, elev=None, azim=None):
    if isinstance(xs, list):
        n_plots = len(xs)
        fig = plt.figure(figsize=(5 * n_plots, 5))
        xlim_min = min([np.amin(arr[0, :]) for arr in xs])
        xlim_max = max([np.amax(arr[0, :]) for arr in xs])
        ylim_min = min([np.amin(arr[1, :]) for arr in xs])
        ylim_max = max([np.amax(arr[1, :]) for arr in xs])
        zlim_min = min([np.amin(arr[2, :]) for arr in xs])
        zlim_max = max([np.amax(arr[2, :]) for arr in xs])
        axs = []
        for idx, d in enumerate(xs):
            if subplot_axis == 0:
                axs.append(fig.add_subplot(1, n_plots, idx + 1, projection='3d'))
            else:
                axs.append(fig.add_subplot(n_plots, 1, idx + 1, projection='3d'))
            axs[-1].plot(xs=d[0, :], ys=d[1, :], zs=d[2, :], color=colors[idx])
            axs[-1].set_xlim(xlim_min, xlim_max)
            axs[-1].set_ylim(ylim_min, ylim_max)
            axs[-1].set_zlim(xlim_min, zlim_max)
            if elev is not None and azim is not None:
                axs[-1].view_init(elev=elev, azim=azim)
            if wp_list is not None:
                for idx, wp in enumerate(wp_list):
                    plotWpReferenceFrame(axs[-1], wp, idx)
            plt.gca().set_aspect('equal', adjustable='datalim')
    else:
        fig = plt.figure(figsize=(5, 5))
        xlim_min = np.amin(xs[0, :])
        xlim_max = np.amax(xs[0, :])
        ylim_min = np.amin(xs[1, :])
        ylim_max = np.amax(xs[1, :])
        zlim_min = np.amin(xs[2, :])
        zlim_max = np.amax(xs[2, :])
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        ax.plot(xs=xs[0, :], ys=xs[1, :], zs=xs[2, :])
        ax.set_xlim(xlim_min, xlim_max)
        ax.set_ylim(ylim_min, ylim_max)
        ax.set_zlim(xlim_min, zlim_max)
        if elev is not None and azim is not None:
            ax.view_init(elev=elev, azim=azim)
        if wp_list is not None:
            for idx, wp in enumerate(wp_list):
                plotWpReferenceFrame(ax, wp, idx)
        plt.gca().set_aspect('equal', adjustable='datalim')


def PlotControls(us, time, wp_list=None, legend=None):
    if isinstance(us, list):
        n_subplots = us[0].shape[0]
    else:
        n_subplots = us.shape[0]

    fig, axs = plt.subplots(n_subplots, 1, figsize=(15, 10), sharex=True)
    plotTrajectory(us,
                   time,
                   axs,
                   0,
                   4,
                   names=['Rotor 1', 'Rotor 2', 'Rotor3', 'Rotor4'],
                   wp_list=wp_list,
                   legend=legend)


def PlotStateErrors(errors, time, wp_list, fig_title='', legend=None):
    fig, axs = plt.subplots(4, 1, figsize=(15, 10))
    fig.suptitle(fig_title)
    plotTrajectory(errors,
                   time,
                   axs,
                   0,
                   4,
                   names=['Pos. error', 'Att.', 'Vel. lin.', 'Vel. ang.'],
                   wp_list=None,
                   legend=legend)


def PlotPosition(xs, time, wp_list=None, fig_title='', legend=None):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    fig.suptitle(fig_title)
    plotTrajectory(xs, time, axs, 0, 3, names=['X pos', 'Y pos', 'Z pos'], wp_list=wp_list, legend=legend)


def PlotAttitude(xs, time, wp_list=None, legend=None):
    fig, axs = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    plotTrajectory(xs, time, axs, 3, 7, names=['X quat', 'Y quat', 'Z quat', 'W quat'], wp_list=wp_list, legend=legend)


def PlotVelocityLin(xs, time, wp_list=None, legend=None):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    plotTrajectory(xs,
                   time,
                   axs,
                   7,
                   10,
                   names=['X vel. lin.', 'Y vel. lin.', 'Z vel. lin.'],
                   wp_list=wp_list,
                   legend=legend)


def PlotVelocityAng(xs, time, wp_list=None, legend=None):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    plotTrajectory(xs,
                   time,
                   axs,
                   10,
                   13,
                   names=['X vel. ang.', 'Y vel. ang.', 'Z vel. ang.'],
                   wp_list=wp_list,
                   legend=legend)


def PlotMotorSpeed(us, time, wp_list=None):
    fig, axs = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    plotTrajectory(us, time, axs, 0, 4)


def plotTrajectory(data, time, axs, row_init, row_end, names=None, wp_list=None, legend=None):
    if isinstance(data, list):
        for idx, d in enumerate(data):
            knots = np.size(d, 1)
            if isinstance(time, list):
                time_ = time[idx]
            else:
                time_ = time

            for i in range(row_end - row_init):
                # Inneficient search for min
                y_min = min([np.amin(arr[row_init + i, :]) for arr in data])
                y_max = max([np.amax(arr[row_init + i, :]) for arr in data])
                # axs[i].plot(t, d[row_init + i, :], color=colors[idx], marker='v')
                axs[i].plot(time_, d[row_init + i, :], color=colors[idx])
                if names is not None:
                    axs[i].set_title(names[i])
                if wp_list is not None:
                    wp_array, time_array = wayPointListToStateArray(wp_list)
                    axs[i].plot(time_array, wp_array[row_init + i, :], 'r+', label='_nolegend_')
                axs[i].grid(linestyle='--', linewidth=0.5)
                axs[i].margins(x=0, y=0)
                axs[i].set_ylim(y_min - 0.1, y_max + 0.1)
        if legend is not None:
            axs[0].legend(legend)
    else:
        if row_end - row_init == 1:
            axs.plot(time, data[0, :], 'v')
            if names is not None:
                axs.set_title(names[0])
        else:
            for i in range(row_end - row_init):
                y_min = np.amin(data[row_init + i, :])
                y_max = np.amax(data[row_init + i, :])
                axs[i].plot(time, data[row_init + i, :])
                if names is not None:
                    axs[i].set_title(names[i])
                if wp_list is not None:
                    wp_array, time_array = wayPointListToStateArray(wp_list)
                    axs[i].plot(time_array, wp_array[row_init + i, :], 'r+')
                axs[i].grid(linestyle='--', linewidth=0.5)
                axs[i].margins(x=0, y=0, z=0)
                axs[i].set_ylim(y_min * 1.1, y_max * 1.1)


def plotWpReferenceFrame(ax, wp, wp_number=None):
    origin = wp.pose.translation
    x_vec = wp.pose.rotation[:, 0] / 3
    y_vec = wp.pose.rotation[:, 1] / 3
    z_vec = wp.pose.rotation[:, 2] / 3
    ax.quiver(origin[0], origin[1], origin[2], x_vec[0], x_vec[1], x_vec[2], color='r')
    ax.quiver(origin[0], origin[1], origin[2], y_vec[0], y_vec[1], y_vec[2], color='g')
    ax.quiver(origin[0], origin[1], origin[2], z_vec[0], z_vec[1], z_vec[2], color='b')
    if wp_number is not None:
        ax.text(origin[0], origin[1], origin[2], "WP" + str(wp_number))


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
            ax.vlines(k, y_lim[0], y_lim[1], color='silver', linestyle='dashed')


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
    plt.tight_layout()
    plt.show()


def saveFig(name):
    plt.savefig(name, bbox_inches='tight')
