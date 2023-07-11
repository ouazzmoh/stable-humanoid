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
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from utils import *
from bezier_curve import BezierCurve

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description
except ModuleNotFoundError:
    raise ModuleNotFoundError(
        "Examples need robot_descriptions, "
        "try `pip install robot_descriptions`"
    )

robot = load_robot_description(
        "jvrc_description", root_joint=pin.JointModelFreeFlyer()
    )
viz = pin.visualize.MeshcatVisualizer(
        robot.model, robot.collision_model, robot.visual_model
    )
robot.setVisualizer(viz, init=False)
viz.initViewer(open=True)
viz.loadViewerModel()
list_data = [ 8.29756125e-02, -9.10813747e-05,  8.69033893e-03,  7.94971455e-05,
                -2.00299738e-03, -5.32537164e-06,  9.99997991e-01, -1.82130515e-02,
                -7.96762498e-05, -2.81783003e-05,  3.98387475e-01, -3.01838539e-05,
                -3.88145027e-01, -1.79387328e-02, -4.68154345e-05, -3.64688202e-05,
                3.97755529e-01, -6.64565981e-05, -3.87904610e-01,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                0.00000000e+00,  0.00000000e+00,  0.00000000e+00
]
q_ref = np.array(list_data)
configuration = pink.Configuration(robot.model, robot.data, q_ref)
viz.display(configuration.q)
T_pred = 100e-3  # (s)
T_control = 100e-3  # (s)
simulation_time = 10  # (s)
prediction_time = 2  # (s)
g = 9.81
h = pin.centerOfMass(robot.model, robot.data, q_ref)[2] + 0.74
foot_dimensions = [
    0.3, 
    np.abs(configuration.get_transform_frame_to_world("r_ankle").copy().translation[1])
    ]  # length(x), width(y)
spacing = (0.0, np.abs(configuration.get_transform_frame_to_world("r_ankle").copy().translation[1]))  # lateral spacing between feet
duration_double_init = 0.8  # (s)
duration_step = 1  # (s)
steps = int(simulation_time / T_control)
alpha = 1  # Weight for jerk
gamma = 1e3  # Weight for zk_ref
beta = 1   # Weight for velocity
average_speed = (0.3, 0)
stop_at = (8, 10)  # (s)

robot_mpc = Robot(h, foot_dimensions, spacing_x=spacing[0], spacing_y=spacing[1])


def move(trajectory_type, debug=False, store=False, perturbations=None):
    # Problem variables
    xk_init = (0, 0, 0)
    yk_init = (0, 0, 0)
    # Footstep planning
    step_planner = FootstepPlanner(robot_mpc, simulation_time, duration_double_init,
                                   duration_step, trajectory_type=trajectory_type, average_speed=average_speed,
                                   stop_at=stop_at)

    zk_min_x, zk_max_x, zk_min_y, zk_max_y, theta_ref = step_planner.footsteps_to_array(0, simulation_time, T_control)
    t = np.arange(0, simulation_time, T_control)
    zk_ref_x = (zk_min_x + zk_max_x)/2
    zk_ref_y = (zk_min_y + zk_max_y)/2
    plt.plot(t, zk_ref_x, label="zk_ref_x")
    plt.plot(t, zk_ref_y, label="zk_ref_y")
    plt.xlabel("time (s)")
    plt.ylabel("cop_reference (m)")
    plt.title("footstep references in time")
    plt.legend()
    plt.show()


    # Running the MPC
    controller = MPC(simulation_time, prediction_time, T_control, T_pred, robot_mpc, step_planner,
                     alpha, beta, gamma, xk_init, yk_init, write_hdf5=store, debug=debug, perturbations=perturbations)

    # Initialize state of the robot
    robot_mpc.initialize_position(xk_init, yk_init, g)

    # Parameters
    N = int(prediction_time / T_pred)
    T = T_pred  # Used for intermediate visualizations
    n_iterations = int(simulation_time / T_control)

    # Run the MPC
    com_x, com_y, cop_x, cop_y = [], [], [], []
    left_foot, right_foot = [], []

    for i in range(n_iterations):
        # print(vars(robot_mpc))
        # sys.exit()
        curr_com, _, _, curr_cop, curr_left, curr_right = robot_mpc.get_positional_attributes()
        com_x.append(curr_com[0])
        com_y.append(curr_com[1])
        cop_x.append(curr_cop[0])
        cop_y.append(curr_cop[1])
        left_foot.append(curr_left)
        right_foot.append(curr_right)
        # Run the MPC iteration and update the robot state
        controller.MPC_iteration(i, N, T)


    left_foot_unique = remove_duplicates(left_foot)
    right_foot_unique = remove_duplicates(right_foot)

    print(left_foot_unique)
    print(right_foot_unique)
    """
    [(0.0, 0.325), (0.175, 0.325), (0.8749999999999999, 0.325), (1.575, 0.325), (2.2749999999999995, 0.325), (2.9749999999999996, 0.325)]
    [(0.0, -0.325), (0.5249999999999999, -0.325), (1.2249999999999999, -0.325), (1.9249999999999998, -0.325), (2.625, -0.325), (3.325, -0.325)]
    """
    # todo: Start with left
    left_foot_task = FrameTask(
        "l_ankle", position_cost=100.0, orientation_cost=1.0
    )
    left_foot_fixed_task = FrameTask(
        "l_ankle", position_cost=100., orientation_cost=1.0
    )
    pelvis_task = FrameTask(
        "PELVIS_S", position_cost=[1e-2, 1e-2, 1.0], orientation_cost=3.0
    )
    right_foot_task = FrameTask(
        "r_ankle", position_cost=100.0, orientation_cost=1.0
    )
    right_foot_fixed_task = FrameTask(
        "r_ankle", position_cost=100., orientation_cost=1.0
    )
    posture_task = PostureTask(
        cost=1.0,
    )
    posture_task.set_target(q_ref)
    tasks = [
        left_foot_task, 
        # left_foot_fixed_task,
        posture_task,
        pelvis_task,
        right_foot_task,
        # right_foot_fixed_task,

    ]
    # pelvis_task.set_target(configuration.get_transform_frame_to_world("PELVIS_S"))
    # posture_task.set_target_from_configuration(configuration)

    pelvis_task.set_target_from_configuration(configuration)

    left_foot_task.set_target(
        configuration.get_transform_frame_to_world("l_ankle")
    )
    left_foot_fixed_task.set_target(
        configuration.get_transform_frame_to_world("l_ankle")
    )

    right_foot_task.set_target(
        configuration.get_transform_frame_to_world("r_ankle")
    )
    right_foot_fixed_task.set_target(
        configuration.get_transform_frame_to_world("r_ankle")
    )

    # pelvis_task.set_target_from_configuration(configuration)
    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"
    viewer = viz.viewer
    meshcat_shapes.frame(viewer["r_ankle"], opacity=1.)
    meshcat_shapes.frame(viewer["r_ankle_target"], opacity=.5)
    meshcat_shapes.frame(viewer["l_ankle"], opacity=1.)
    meshcat_shapes.frame(viewer["l_ankle_target"], opacity=.5)
    rate = RateLimiter(frequency=50.0)
    dt = rate.period
    t = 0.0  # [s]
    i = 0
    # left_foot_fixed_target = left_foot_fixed_task.transform_target_to_world
    # left_foot_fixed_task.set_target(left_foot_fixed_task)
    src_r = configuration.get_transform_frame_to_world("r_ankle").copy()
    src_r = src_r.translation
    # dst_r = src_r.copy()
    # dst_r[0] += .2
    src_l = configuration.get_transform_frame_to_world("l_ankle").copy()
    src_l = src_l.translation
    # dst_l = src_l.copy()        
    # dst_l[0] += .25
    # import time
    # time.sleep(10)
    for i in range(len(right_foot_unique)- 1):
        t = 0.0
        dst_l = np.array([*left_foot_unique[i + 1], src_l[2]])
        control_points_l = get_control_points(src_l, dst_l, dz=.05)
        curve_l = BezierCurve(control_points_l)
        while t <= 1:
            left_foot_target = left_foot_task.transform_target_to_world
            left_foot_target.translation = curve_l.get_position_at(t)
            viewer["l_ankle_target"].set_transform(left_foot_target.np)
            viewer["l_ankle"].set_transform(configuration.get_transform_frame_to_world(left_foot_task.body).np)
            # Compute velocity and integrate it into next configuration
            velocity = solve_ik(configuration, tasks, dt, solver=solver)
            configuration.integrate_inplace(velocity, dt)

            # Visualize result at fixed FPS
            viz.display(configuration.q)
            # rate.sleep()
            t += dt
        # src_l = dst_l.copy()
        src_l = configuration.get_transform_frame_to_world(left_foot_task.body).translation
        dst_r = np.array([*right_foot_unique[i + 1], src_r[2]])
        control_points_r = get_control_points(src_r, dst_r, dz=.05)
        curve_r = BezierCurve(control_points_r)
        t = 0.0
        while t <= 1:
            # Update task targets
            right_foot_target = right_foot_task.transform_target_to_world
            right_foot_target.translation = curve_r.get_position_at(t)
            
            viewer["r_ankle_target"].set_transform(right_foot_target.np)
            viewer["r_ankle"].set_transform(configuration.get_transform_frame_to_world(right_foot_task.body).np)
            # viewer["l_ankle_target"].set_transform(left_foot_fixed_target.np)
            viewer["l_ankle"].set_transform(configuration.get_transform_frame_to_world(left_foot_fixed_task.body).np)
            # Compute velocity and integrate it into next configuration
            velocity = solve_ik(configuration, tasks, dt, solver=solver)
            configuration.integrate_inplace(velocity, dt)

            # Visualize result at fixed FPS
            viz.display(configuration.q)
            # rate.sleep()
            t += dt
        # src_r = dst_r.copy()
        src_r = configuration.get_transform_frame_to_world(right_foot_task.body).translation
        # dst_r[0] += .2
        
        # dst_l[0] += .2
    # print(configuration.q)


def main():
    trajectory_type = "forward"
    move(trajectory_type, debug=False)


if __name__ == "__main__":
    main()

