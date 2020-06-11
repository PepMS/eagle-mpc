import sys

import pinocchio
import crocoddyl
import example_robot_data
import yaml_parser

import multicopter_mpc
from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR, MULTICOPTER_MPC_MISSION_DIR

WITHDISPLAY = 'display' in sys.argv
WITHPLOT = 'plot' in sys.argv
HECTOR = 'hector' in sys.argv

if HECTOR:
    uav = example_robot_data.loadHector()
    yaml_uav = yaml_parser.ParserYAML(MULTICOPTER_MPC_MULTIROTOR_DIR + "/hector.yaml", "", True)
else:
    uav = example_robot_data.loadIris()
    link_name = "iris__base_link"
    yaml_uav = yaml_parser.ParserYAML(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml", "", True)

uav_model = uav.model
base_link = uav_model.getFrameId(link_name)

# # UAV Params
server_uav = yaml_parser.ParamsServer(yaml_uav.getParams())
mc_params = multicopter_mpc.MultiCopterBaseParams()
mc_params.fill(server_uav)

# Mission
yaml_mission = yaml_parser.ParserYAML(MULTICOPTER_MPC_MISSION_DIR + "/simple.yaml", "", True)
server_mission = yaml_parser.ParamsServer(yaml_mission.getParams())
mission = multicopter_mpc.Mission(uav.nq + uav.nv)
mission.fillWaypoints(server_mission)
mission.fillInitialState(server_mission)

dt = 1e-2
trajectory = multicopter_mpc.TrajectoryGenerator(uav_model, mc_params, dt, mission)
trajectory.createProblem(multicopter_mpc.SolverType.SolverTypeBoxFDDP)
trajectory.setSolverCallbacks(True)
trajectory.solve()

if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(uav)
    # for idx_wp, wp in enumerate(mission.waypoints):
    #     name = 'world/wp' + str(idx_wp)
    #     uav.viewer.gui.addXYZaxis(name, [1., 0., 0., 1.], .03, 0.5)
    #     wp_pose = pinocchio.SE3ToXYZQUAT(wp.pose)
    #     uav.viewer.gui.applyConfiguration(name, wp_pose.reshape(7).tolist()[0])

    display.displayFromSolver(trajectory.solver)
