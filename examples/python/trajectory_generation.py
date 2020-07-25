import sys

import multicopter_mpc

import pinocchio
import crocoddyl
import example_robot_data
import yaml_parser

from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR, MULTICOPTER_MPC_MISSION_DIR

WITHDISPLAY = 'display' in sys.argv
WITHPLOT = 'plot' in sys.argv

uav = example_robot_data.loadIris()
yaml_uav = yaml_parser.ParserYAML(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml", "", True)
uav_model = uav.model

# # UAV Params
server_uav = yaml_parser.ParamsServer(yaml_uav.getParams())
mc_params = multicopter_mpc.MultiCopterBaseParams()
mc_params.fill(server_uav)

# Mission
yaml_mission = yaml_parser.ParserYAML(MULTICOPTER_MPC_MISSION_DIR + "/takeoff.yaml", "", True)
server_mission = yaml_parser.ParamsServer(yaml_mission.getParams())
mission = multicopter_mpc.Mission(uav.nq + uav.nv)
mission.fillWaypoints(server_mission)
mission.fillInitialState(server_mission)

# dt = 4e-3
dt = 1e-2
trajectory = multicopter_mpc.TrajectoryGenerator(uav_model, mc_params, dt, mission)
trajectory.createProblem(multicopter_mpc.SolverType.SolverTypeBoxFDDP)
trajectory.setSolverCallbacks(True)
trajectory.solve()

state_trajectory = trajectory.getStateTrajectory(0, trajectory.n_knots - 1)
# control_trajectory = [trajectory.getCon]
time_lst = [trajectory.dt for i in range(0, trajectory.n_knots + 1)]

if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(uav)
    for idx_wp, wp in enumerate(mission.waypoints):
        name = 'world/wp' + str(idx_wp)
        uav.viewer.gui.addXYZaxis(name, [1., 0., 0., 1.], .03, 0.5)
        wp_pose = pinocchio.SE3ToXYZQUATtuple(wp.pose)
        uav.viewer.gui.applyConfiguration(name, wp_pose)

    display.display(state_trajectory, [], [], time_lst, 1)

# crocoddyl.plotOCSolution(state_trajectory, [], figIndex=1, show=False)

