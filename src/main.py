#!/usr/bin/env python3

from visuals import *
from simulations import *

from robot import Robot
from perturbation import Perturbation
from footstep_planner import FootstepPlanner
from controller import MPC


T_pred = 100e-3  # (s)
T_control = 100e-3  # (s)
simulation_time = 10  # (s)
prediction_time = 2  # (s)
g = 9.81
h = 0.8
foot_dimensions = [0.3, 0.096]  # length(x), width(y)
spacing = (0.1, 0.096)  # lateral spacing between feet
duration_double_init = 0.8  # (s)
duration_step = 0.8  # (s)
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
    
    import meshcat_shapes
    import pinocchio as pin
    import qpsolvers
    from loop_rate_limiters import RateLimiter

    import pink
    from pink import solve_ik
    from pink.tasks import FrameTask, PostureTask
    from bezier_curve import BezierCurve
    from utils import get_control_points

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
    # print([robot.model.names[i] for i in range(len(robot.model.names))])
    # sys.exit()
    viz = pin.visualize.MeshcatVisualizer(
        robot.model, robot.collision_model, robot.visual_model
    )
    robot.setVisualizer(viz, init=False)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    configuration = pink.Configuration(robot.model, robot.data, robot.q0)
    viz.display(configuration.q)

    left_foot_task = FrameTask(
        "l_ankle", position_cost=1.0, orientation_cost=3.0
    )
    left_foot_task_2 = FrameTask(
        "l_ankle", position_cost=3.0, orientation_cost=1.0
    )
    # pelvis_task = FrameTask("PELVIS_S", position_cost=[0.0, 1e-1, 2.0], orientation_cost=2.0)
    pelvis_task = FrameTask("PELVIS_S", position_cost=0.0, orientation_cost=2.0)

    right_foot_task = FrameTask(
        "r_ankle", position_cost=2.0, orientation_cost=3.0
    )
    right_foot_task_2 = FrameTask(
        "r_ankle", position_cost=3.0, orientation_cost=1.0
    )
    left_knee_task = FrameTask("L_KNEE", position_cost=0.0, orientation_cost=1.0)
    right_knee_task = FrameTask("R_KNEE", position_cost=0.0, orientation_cost=1.0)

    posture_task = PostureTask(cost=.5e-1)
    tasks = [left_foot_task, 
             pelvis_task,
            posture_task,
            right_knee_task,
            left_foot_task_2,
            right_foot_task_2,
            left_knee_task,
            right_foot_task, #
            ]

    pelvis_pose = configuration.get_transform_frame_to_world("PELVIS_S").copy()
    meshcat_shapes.frame(viz.viewer["pelvis_pose"])
    viz.viewer["pelvis_pose"].set_transform(pelvis_pose.np)
    pelvis_task.set_target(pelvis_pose)
    viewer = viz.viewer
    transform_l_ankle_target_to_init = pin.SE3(
        np.eye(3), np.array([0.0, 0.0, 0.0])
    )
    transform_r_ankle_target_to_init = pin.SE3(
        np.eye(3), np.array([-0.0, 0.0, 0.0])
    )

    left_foot_task.set_target(
        configuration.get_transform_frame_to_world("l_ankle")
        * transform_l_ankle_target_to_init
    )
    left_foot_task_2.set_target(
        configuration.get_transform_frame_to_world("l_ankle")
        * transform_l_ankle_target_to_init
    )
    right_foot_task.set_target(
        configuration.get_transform_frame_to_world("r_ankle")
        * transform_r_ankle_target_to_init
    )
    right_foot_task_2.set_target(
        configuration.get_transform_frame_to_world("r_ankle")
        * transform_r_ankle_target_to_init
    )
    # pelvis_task.set_target(
    #     configuration.get_transform_frame_to_world("PELVIS_S")
    # )
    pelvis_task.set_target_from_configuration(configuration)
    posture_task.set_target_from_configuration(configuration)
    right_knee_task.set_target_from_configuration(configuration)
    left_knee_task.set_target_from_configuration(configuration)
    # print(configuration.get_transform_frame_to_world("PELVIS_S"))
    # import sys
    # sys.exit()
    meshcat_shapes.frame(viewer["right_foot_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["right_foot"], opacity=1.0)
    meshcat_shapes.frame(viewer["left_foot_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["left_foot"], opacity=1.0)

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"

    rate = RateLimiter(frequency=50.0)
    dt = rate.period
    
    for i in range(max(len(left_foot_unique), len(right_foot_unique)) - 1):
        t = 0.0  # [s]
        # src_r_fixed = np.array([*right_foot_unique[i], -.746])
        src_r_fixed = configuration.get_transform_frame_to_world("r_ankle").translation
        control_points_l = get_control_points(configuration.get_transform_frame_to_world("l_ankle").translation, np.array([*left_foot_unique[i + 1], -.746]), dz=.2)
        curve_l = BezierCurve(control_points_l)
        #left loop
        while t <= 1:
            left_foot_target = left_foot_task.transform_target_to_world
            right_foot_target_2 = right_foot_task_2.transform_target_to_world
            right_foot_target_2.translation = src_r_fixed
            left_foot_target.translation = curve_l.get_position_at(t)
            left_foot_task.set_target(left_foot_target)
            right_foot_task_2.set_target(right_foot_target_2)
            viewer["left_foot_target"].set_transform(left_foot_target.np)
            viewer["left_foot"].set_transform(
            configuration.get_transform_frame_to_world(
                left_foot_task.body
            ).np
        )
            velocity = solve_ik(configuration, tasks, dt, solver=solver)
            configuration.integrate_inplace(velocity, dt)

            # Visualize result at fixed FPS
            viz.display(configuration.q)
            # rate.sleep()
            t += dt
        t = 0.0
        # src_l_fixed = np.array([*left_foot_unique[i + 1], -.746])
        src_l_fixed = configuration.get_transform_frame_to_world("l_ankle").translation
        control_points_r = get_control_points(configuration.get_transform_frame_to_world("r_ankle").translation, np.array([*right_foot_unique[i + 1], -.746]))
        curve_r = BezierCurve(control_points_r)
        while t <= 1:
            right_foot_target = right_foot_task.transform_target_to_world
            left_foot_target_2 = left_foot_task_2.transform_target_to_world
            left_foot_target_2.translation = src_l_fixed
            right_foot_target.translation = curve_r.get_position_at(t)
            right_foot_task.set_target(right_foot_target)
            left_foot_task_2.set_target(left_foot_target_2)
            viewer["right_foot_target"].set_transform(right_foot_target.np)
            viewer["right_foot"].set_transform(
            configuration.get_transform_frame_to_world(
                right_foot_task.body
            ).np
        )
            velocity = solve_ik(configuration, tasks, dt, solver=solver)
            configuration.integrate_inplace(velocity, dt)

            # Visualize result at fixed FPS
            viz.display(configuration.q)
            # rate.sleep()
            t += dt
        #right_loop

    # fig, ax = plt.subplots()
    # ax.plot(cop_x, cop_y, label="cop", color="green")
    # ax.plot(com_x, com_y, label="com", color="red")
    # # Plot footsteps
    # plot_foot_steps(ax, (zk_min_x + zk_max_x) / 2, (zk_min_y + zk_max_y) / 2, theta_ref, foot_dimensions, spacing[1])
    # # Display the plot
    # plt.legend()
    # ax.set_xlabel("x(m)")
    # ax.set_ylabel("y(m)")
    # # ax.set_ylim((-0.03, 0.03))
    # plt.title("Trajectory of robot")
    # plt.show()


def main():
    trajectory_type = "forward"
    move(trajectory_type, debug=False)


if __name__ == "__main__":
    main()

