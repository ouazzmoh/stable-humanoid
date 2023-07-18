#!/usr/bin/env python3

from visuals import *
from simulations import *

from robot import Robot
from perturbation import Perturbation
from footstep_planner import FootstepPlanner
from controller import MPC
import sys
import meshcat_shapes
import numpy as np
import pinocchio as pin
import qpsolvers
from loop_rate_limiters import RateLimiter
import pinocchio as pin
import pink
from pink.tasks import FrameTask, PostureTask
from com_task import ComTask
from utils import *
from move_foot import move_foot, get_foot_curve, get_com_positions

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description
except ModuleNotFoundError:
    raise ModuleNotFoundError(
        "Examples need robot_descriptions, " "try `pip install robot_descriptions`"
    )

robot = load_robot_description("jvrc_description", root_joint=pin.JointModelFreeFlyer())
viz = pin.visualize.MeshcatVisualizer(
    robot.model, robot.collision_model, robot.visual_model
)
robot.setVisualizer(viz, init=False)
viz.initViewer(open=True)
viz.loadViewerModel()
list_data = [
    8.29756125e-02,
    -9.10813747e-05,
    8.69033893e-03,
    7.94971455e-05,
    -2.00299738e-03,
    -5.32537164e-06,
    9.99997991e-01,
    -1.82130515e-02,
    -7.96762498e-05,
    -2.81783003e-05,
    3.98387475e-01,
    -3.01838539e-05,
    -3.88145027e-01,
    -1.79387328e-02,
    -4.68154345e-05,
    -3.64688202e-05,
    3.97755529e-01,
    -6.64565981e-05,
    -3.87904610e-01,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
    0.00000000e00,
]
q_ref = np.array(list_data)
robot.q0 = q_ref
configuration = pink.Configuration(robot.model, robot.data, robot.q0)
viz.display(configuration.q)
T_pred = 100e-3  # (s)
T_control = 100e-3  # (s)
simulation_time = 10  # (s)
prediction_time = 2  # (s)
g = 9.81
h = (
    pin.centerOfMass(configuration.model, configuration.data, configuration.q)[2]
    + 0.746
)  # (m)
foot_dimensions = [
    0.225,
    np.abs(configuration.get_transform_frame_to_world("r_ankle").copy().translation[1]),
]  # length(x), width(y)
spacing = (
    0.0,
    np.abs(configuration.get_transform_frame_to_world("r_ankle").copy().translation[1] + .6),
)  # lateral spacing between feet
duration_double_init = .8  # (s)
duration_step = .8  # (s)
steps = int(simulation_time / T_control)
alpha = 1  # Weight for jerk
gamma = 1e3  # Weight for zk_ref
beta = 1  # Weight for velocity
average_speed = (0.3, 0)
stop_at = (8, 10)  # (s)

robot_mpc = Robot(h, foot_dimensions, spacing_x=spacing[0], spacing_y=spacing[1])


def move(trajectory_type, debug=False, store=False, perturbations=None):
    # Problem variables
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)
    # Footstep planning
    step_planner = FootstepPlanner(
        robot_mpc,
        simulation_time,
        duration_double_init,
        duration_step,
        trajectory_type=trajectory_type,
        average_speed=average_speed,
        stop_at=stop_at,
    )

    zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref = step_planner.footsteps_to_array(
        0, simulation_time, T_control
    )
    t = np.arange(0, simulation_time, T_control)
    zk_ref_x = (zk_min_x + zk_max_x) / 2
    zk_ref_y = (zk_min_y + zk_max_y) / 2
    plt.plot(t, zk_ref_x, label="zk_ref_x")
    plt.plot(t, zk_ref_y, label="zk_ref_y")
    plt.xlabel("time (s)")
    plt.ylabel("cop_reference (m)")
    plt.title("footstep references in time")
    plt.legend()
    plt.show()

    # Running the MPC
    controller = MPC(
        simulation_time,
        prediction_time,
        T_control,
        T_pred,
        robot_mpc,
        step_planner,
        alpha,
        beta,
        gamma,
        xk_init,
        yk_init,
        write_hdf5=store,
        debug=debug,
        perturbations=perturbations,
    )

    # Initialize state of the robot
    robot_mpc.initialize_position(xk_init, yk_init, g)

    # Parameters
    N = int(prediction_time / T_pred)
    T = T_pred  # Used for intermediate visualizations
    n_iterations = int(simulation_time / T_control)

    # Run the MPC
    com_x, com_y, cop_x, cop_y = [], [], [], []

    for i in range(n_iterations):
        # print(vars(robot_mpc))
        # sys.exit()
        (
            curr_com,
            _,
            _,
            curr_cop,
            curr_left,
            curr_right,
        ) = robot_mpc.get_positional_attributes()
        com_x.append(curr_com[0])
        com_y.append(curr_com[1])
        cop_x.append(curr_cop[0])
        cop_y.append(curr_cop[1])
        # Run the MPC iteration and update the robot state
        controller.MPC_iteration(i, N, T)
    # print(len(com_x))

    seen = []
    res = []
    left_foot = robot_mpc.offline_left_foot_trajectory
    right_foot = robot_mpc.offline_right_foot_trajectory
    com = robot_mpc.offline_com_trajectory

    corresp_com_left = []
    corresp_com_right = []

    left_indices = group_not_none(left_foot)
    right_indices = group_not_none(right_foot)

    for k in range(len(left_indices) - 1):
        corresp_com_left.append(com[left_indices[k][1] : left_indices[k + 1][0]])
    for k in range(len(right_indices) - 1):
        corresp_com_right.append(com[right_indices[k][1] : right_indices[k + 1][0]])

    left_foot_unique = [left_foot[i[0]] for i in left_indices]
    right_foot_unique = [right_foot[i[0]] for i in right_indices]

    print("left foot ->", left_foot_unique)
    print("right foot ->", right_foot_unique)
    print("com left ->", corresp_com_left)
    print("com right -> ", corresp_com_right)

    # todo: Start with left
    left_foot_task = FrameTask("l_ankle", position_cost=100.0, orientation_cost=1.0)

    pelvis_task = FrameTask(
        "PELVIS_S", position_cost=[0.0, 0.0, 1.0], orientation_cost=3.0
    )
    right_foot_task = FrameTask("r_ankle", position_cost=100.0, orientation_cost=1.0)
    com_task = ComTask(position_cost=[1.0, 10.0, 1.0])
    posture_task = PostureTask(
        cost=1e-1,  # 1e-1
    )
    posture_task.set_target(robot.q0)
    tasks = [
        left_foot_task,
        posture_task,
        pelvis_task,
        right_foot_task,
        com_task,
    ]

    pelvis_task.set_target_from_configuration(configuration)
    com_task.set_target_from_configuration(configuration)
    left_foot_task.set_target(configuration.get_transform_frame_to_world("l_ankle"))

    right_foot_task.set_target(configuration.get_transform_frame_to_world("r_ankle"))

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"
    viewer = viz.viewer
    meshcat_shapes.frame(viewer["r_ankle"], opacity=1.0)
    meshcat_shapes.frame(viewer["r_ankle_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["l_ankle"], opacity=1.0)
    meshcat_shapes.frame(viewer["l_ankle_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["com"], opacity=1.0, axis_length=1.0)
    rate = RateLimiter(frequency=50.0)

    src_r = configuration.get_transform_frame_to_world("r_ankle").copy()
    src_r = src_r.translation

    src_l = configuration.get_transform_frame_to_world("l_ankle").copy()
    src_l = src_l.translation
    for i in range(len(right_foot_unique) - 1):
        # left_foot
        dst_l = np.array([*left_foot_unique[i + 1], src_l[2]])
        curve_l = get_foot_curve(src_l, dst_l, dz=0.15)
        com_l = get_com_positions(corresp_com_left[i][0], corresp_com_left[i][-1], 50)
        move_foot(
            configuration,
            tasks,
            left_foot_task,
            com_task,
            curve_l,
            com_l,
            solver,
            rate,
            viz,
        )
        src_l = configuration.get_transform_frame_to_world(
            left_foot_task.body
        ).translation
        # right_foot
        dst_r = np.array([*right_foot_unique[i + 1], src_r[2]])
        curve_r = get_foot_curve(src_r, dst_r, dz=0.15)
        com_r = get_com_positions(corresp_com_right[i][0], corresp_com_right[i][-1], 50)
        move_foot(
            configuration,
            tasks,
            right_foot_task,
            com_task,
            curve_r,
            com_r,
            solver,
            rate,
            viz,
        )
        src_r = configuration.get_transform_frame_to_world(
            right_foot_task.body
        ).translation


def main():
    trajectory_type = "upwards_turning"
    move(trajectory_type, debug=False)


if __name__ == "__main__":
    main()
