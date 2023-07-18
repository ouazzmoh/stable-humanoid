
from bezier_curve import BezierCurve
import pinocchio as pin
from pink import solve_ik
from loop_rate_limiters import RateLimiter
from utils import get_control_points
import numpy as np
import pink
from pink.tasks import Task, FrameTask
from typing import List, Optional
from com_task import ComTask

def move_foot(configuration : pink.Configuration,
              tasks: List[Task],  
              foot_task:FrameTask , 
              com_task: ComTask,
              curve: BezierCurve,
              com_positions:np.ndarray,
              orientation:np.ndarray,
              solver:str,
              rate : RateLimiter,
              viz: Optional[pin.visualize.MeshcatVisualizer]=None,
              display:Optional[bool]=True,
              **kwargs):
    """
    Move the foot along a Bezier curve while controlling the Center of Mass (COM)
    and additional tasks.

    Args:
        configuration: Robot configuration.
        tasks: List of tasks controlling the robot.
        foot_task: Task controlling the foot.
        com_task: Task controlling the Center of Mass (COM).
        curve: Bezier curve defining the foot trajectory.
        com_positions: Array of COM positions over time.
        solver: Solver to use for inverse kinematics.
        rate: Rate limiter controlling the visualization frame rate.
        viz: Meshcat visualizer for visualization (default: None).
        display: Flag indicating whether to display the robot configuration 
                (default: True).
        kwargs: Additional tasks and their target values.

    """
    t = 0.0
    dt = rate.period
    frequency = 1.0 / dt
    while t <= 1:
            # Update task targets
            right_foot_target = foot_task.transform_target_to_world
            com_target = com_task.transform_target_to_world
            right_foot_target.translation = curve.get_position_at(t)
            right_foot_target.rotation = pin.AngleAxis(orientation[round(t * frequency)], np.array([0, 0, 1])).toRotationMatrix()
            com_target.translation[0] = com_positions[round(t * frequency)][0] 
            com_target.translation[1] = com_positions[round(t * frequency)][1]
            com_task.set_target(com_target)
            for task, target in kwargs.items():
                task_target = task.transform_target_to_world
                task_target = target
                task.set_target(task_target)
            if viz:
                viz.viewer["com"].set_transform(com_target.np)
                viz.viewer["r_ankle_target"].set_transform(right_foot_target.np)
                viz.viewer["r_ankle"].set_transform(configuration.get_transform_frame_to_world(foot_task.body).np)
            # Compute velocity and integrate it into next configuration
            velocity = solve_ik(configuration, tasks, dt, solver=solver)
            configuration.integrate_inplace(velocity, dt)
            # Visualize result at fixed FPS
            if display and viz:
                viz.display(configuration.q)
            # rate.sleep()
            t += dt
            
def get_foot_curve(src:np.ndarray, dst:np.ndarray, dz:Optional[float]=.15):
    """
    Compute a Bezier curve for the foot trajectory given the source
    and destination positions.

    Args:
        src: Source position of the foot.
        dst: Destination position of the foot.
        dz : Vertical displacement of the curve (default: 0.15).

    Returns:
        BezierCurve: Bezier curve defining the foot trajectory.
    """
    control_points = get_control_points(src, dst, dz=dz)
    return BezierCurve(control_points)

def get_com_positions(src: np.ndarray, dst: np.ndarray, frequency: Optional[float]=50.0):
    """
    Generate an array of center of mass positions between the source
    and destination positions.

    Args:
        src: Source position of the center of mass.
        dst: Destination position of the center of mass.
        frequency: Number of positions to generate per unit of time (default: 50.0).

    Returns:
        np.ndarray: Array of center of mass positions.
    """
    return np.linspace(src, dst, frequency)

def get_orientations(src: np.ndarray, dst: np.ndarray, frequency: Optional[float]=50.0):
    """
    Generate an array of orientations between the source
    and destination orientations.

    Args:
        src: Source orientation.
        dst: Destination orientation.
        frequency: Number of orientations to generate per unit of time (default: 50.0).

    Returns:
        np.ndarray: Array of orientations.
    """
    return np.linspace(src, dst, frequency)