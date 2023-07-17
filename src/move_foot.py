
from bezier_curve import BezierCurve
import pinocchio as pin
from pink import solve_ik
from loop_rate_limiters import RateLimiter
from utils import get_control_points
import numpy as np

def move_foot(configuration,
              tasks,  
              foot_task, 
              com_task,
              curve: BezierCurve,
              com_positions,
              solver,
              rate : RateLimiter,
              viz: pin.visualize.MeshcatVisualizer=None,
              display=True,
              **kwargs):
    t = 0.0
    dt = rate.period
    frequency = 1.0 / dt
    while t <= 1:
            # Update task targets
            right_foot_target = foot_task.transform_target_to_world
            com_target = com_task.transform_target_to_world
            right_foot_target.translation = curve.get_position_at(t)
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
            rate.sleep()
            t += dt
            
def get_foot_curve(src, dst, dz=.15):
    control_points_l = get_control_points(src, dst, dz=dz)
    return BezierCurve(control_points_l)

def get_com_positions(src, dst, frequency=50.0):
    return np.linspace(src, dst, frequency)