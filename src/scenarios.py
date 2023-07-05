import numpy as np
import matplotlib.pyplot as plt
from step import Step
from typing import List, Tuple
from utils import smooth_data

def construct_zmin_zmax_alt(total_duration, duration_double_init, duration_step,
                            foot_size, spacing) -> List[Step]:
    """
    Construct the minimum and maximum for the center of pressure, alternating around 0
    :param total_duration:
    :param duration_double_init:
    :param duration_step:
    :param foot_size:
    :param spacing:
    :return:
    """
    number_of_steps = round((total_duration - duration_double_init) / duration_step)
    footsteps : list[Step] = []
    # Initial double support
    footsteps.append(Step(0,
                          duration_double_init,
                          z_min=-(foot_size + spacing/2),
                          z_max=foot_size + spacing/2,
                          shift=0))
    for step_i in range(number_of_steps):
        if step_i % 2 == 0:
            if step_i == number_of_steps - 1:
                # Left support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=-(foot_size + spacing/2),
                                      z_max=-spacing/2,
                                      shift=0))
            else:
                # Left support
                footsteps.append(Step(step_i*duration_step + duration_double_init,
                                      (step_i+1)*duration_step + duration_double_init,
                                      z_min=-(foot_size + spacing/2),
                                      z_max=-spacing/2,
                                      shift=0))
        else:
            if step_i == number_of_steps - 1:
                # Right support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=spacing/2,
                                      z_max=foot_size + spacing/2,
                                      shift=0))
            else:
                # Right support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      (step_i + 1) * duration_step + duration_double_init,
                                      z_min=spacing/2,
                                      z_max=foot_size + spacing/2,
                                      shift=0))
    return footsteps



def construct_zmin_zmax_forward(total_duration, duration_double_init, duration_step, foot_size, spacing):
    """
    Construct the minimum and maximum for the center of pressure when moving forward
    :param total_duration:
    :param duration_double_init:
    :param duration_step:
    :param foot_size:
    :param spacing:
    :return:
    """

    number_of_steps = round((total_duration - duration_double_init) / duration_step)
    footsteps: list[Step] = []
    # Initial double support
    footsteps.append(Step(0,
                          duration_double_init,
                          z_min=-(foot_size + spacing/2),
                          z_max=foot_size + spacing/2,
                          shift=0))
    # Step on other direction: keep the double support here
    footsteps.append(Step(duration_double_init,
                          duration_double_init+duration_step,
                          z_min=-(foot_size + spacing/2),
                          z_max=foot_size + spacing/2,
                          shift=0))

    for step_i in range(1, number_of_steps):
        if step_i == number_of_steps - 1:
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  total_duration,
                                  z_min=(step_i - 1) * (foot_size + spacing/2),
                                  z_max=step_i * (foot_size + spacing/2),
                                  shift=0))
        else:
            # Complete with the last value until the end of the simulation
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  (step_i + 1) * duration_step + duration_double_init,
                                  z_min=(step_i - 1)*(foot_size + spacing/2),
                                  z_max=step_i*(foot_size + spacing/2),
                                  shift=0))
    return footsteps


def construct_zmin_zmax_alt_forward(total_duration, duration_double_init, duration_step,
                            foot_size, spacing, shift) -> List[Step]:
    """
    Construct the minimum and maximum for the center of pressure, alternating around 0
    :param total_duration:
    :param duration_double_init:
    :param duration_step:
    :param foot_size:
    :param spacing:
    :return:
    """
    number_of_steps = round((total_duration - duration_double_init) / duration_step)
    footsteps : list[Step] = []
    # Initial double support
    footsteps.append(Step(0,
                          duration_double_init,
                          z_min=-(foot_size + spacing/2),
                          z_max=foot_size + spacing/2,
                          shift=0))
    for step_i in range(number_of_steps):
        if step_i % 2 == 0:
            if step_i == number_of_steps - 1:
                # Left support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=-(foot_size + spacing/2) + step_i * shift,
                                      z_max=0 + step_i * shift,
                                      shift=0))
            else:
                # Left support
                footsteps.append(Step(step_i*duration_step + duration_double_init,
                                      (step_i+1)*duration_step + duration_double_init,
                                      z_min=-(foot_size + spacing/2) + step_i * shift,
                                      z_max=0 + step_i * shift,
                                      shift=0))
        else:
            if step_i == number_of_steps - 1:
                # Right support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=0 + step_i * shift,
                                      z_max=foot_size + spacing/2 + step_i * shift,
                                      shift=0))
            else:
                # Right support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      (step_i + 1) * duration_step + duration_double_init,
                                      z_min=0 + step_i * shift,
                                      z_max=foot_size + spacing/2 + step_i * shift,
                                      shift=0))
    return footsteps


def construct_zmin_zmax_forward_backwards(total_duration, duration_double_init, duration_step, foot_size, spacing):
    """
    Construct the minimum and maximum for the center of pressure when moving forward
    :param total_duration:
    :param duration_double_init:
    :param duration_step:
    :param foot_size:
    :param spacing:
    :return:
    """
    number_of_steps = round((total_duration - duration_double_init) / duration_step) + 1
    footsteps: list[Step] = []
    # Initial double support
    footsteps.append(Step(0,
                          duration_double_init,
                          z_min=-(foot_size + spacing/2),
                          z_max=foot_size + spacing/2,
                          shift=0))
    # Step on other direction: keep the double support here
    footsteps.append(Step(duration_double_init,
                          duration_double_init+duration_step,
                          z_min=-(foot_size + spacing/2),
                          z_max=foot_size + spacing/2,
                          shift=0))
    for step_i in range(1, int(0.5*number_of_steps)):
        # Complete with the last value until the end of the simulation
        footsteps.append(Step(step_i * duration_step + duration_double_init,
                              (step_i + 1) * duration_step + duration_double_init,
                              z_min=(step_i - 1) * (foot_size + spacing/2),
                              z_max=step_i * (foot_size + spacing/2),
                              shift=0))


    for step_i in range(int(number_of_steps*0.5), number_of_steps):
        if step_i == number_of_steps - 1:
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  total_duration,
                                  z_min=(number_of_steps - step_i - 1) * (foot_size + spacing/2),
                                  z_max=(number_of_steps - step_i) * (foot_size + spacing/2),
                                  shift=0))
        else:
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  (step_i + 1) * duration_step + duration_double_init,
                                  z_min=(number_of_steps - step_i - 1)*(foot_size + spacing/2),
                                  z_max=(number_of_steps - step_i)*(foot_size + spacing/2),
                                  shift=0))
    return footsteps


zk_ref_x_interactive = []
zk_ref_y_interactive = []
fig, ax = plt.subplots()

def onclick(event):
    global zk_ref_x_interactive, zk_ref_y_interactive, ax
    zk_ref_x_interactive.append(event.xdata)
    zk_ref_y_interactive.append(event.ydata)
    # Add a dot where the user clicked and refresh the plot
    ax.plot(event.xdata, event.ydata, 'go')
    plt.draw()


def construct_zmin_zmax_interactive(x_axis: Tuple,
                                    y_axis: Tuple,
                                    total_duration: float,
                                    duration_double_init: float,
                                    duration_step: float,
                                    foot_dimensions: Tuple,
                                    spacing: Tuple) -> Tuple[List[Step], List[Step]]:
    # Create a new figure and connect the click event to the callback function
    global zk_ref_x_interactive, zk_ref_y_interactive, ax
    if zk_ref_x_interactive:
        zk_ref_x_interactive = []
    if zk_ref_y_interactive:
        zk_ref_y_interactive = []
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    # Add some labels to your plot
    ax.set_title('Click to add reference steps')
    ax.set_xlim(x_axis[0], x_axis[1])
    ax.set_ylim(y_axis[0], y_axis[1])
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # Display the plot
    plt.grid()
    plt.show()

    zk_ref_y_interactive = smooth_data(zk_ref_y_interactive)
    zk_ref_x_interactive = smooth_data(zk_ref_x_interactive)

    zk_min_x = np.array(zk_ref_x_interactive) - foot_dimensions[0]/2
    zk_max_x = np.array(zk_ref_x_interactive) + foot_dimensions[0]/2
    zk_min_x[0] = -spacing[0]/2 - foot_dimensions[0]
    zk_max_x[0] = spacing[0]/2 + foot_dimensions[0]
    zk_min_x[1] = -spacing[0] / 2 - foot_dimensions[0]
    zk_max_x[1] = spacing[0] / 2 + foot_dimensions[0]

    zk_min_y = np.array(zk_ref_y_interactive) - foot_dimensions[1]/2
    zk_max_y = np.array(zk_ref_y_interactive) + foot_dimensions[1]/2
    zk_min_y[0] = -spacing[1]/2 - foot_dimensions[1]
    zk_max_y[0] = spacing[1]/2 + foot_dimensions[1]

    footsteps_x, footsteps_y = [], []

    # Initial Double support
    footsteps_x.append(Step(0, duration_double_init,
                            z_min=zk_min_x[0],
                            z_max=zk_max_x[0],
                            shift=0))
    footsteps_y.append(Step(0, duration_double_init,
                            z_min=zk_min_y[0],
                            z_max=zk_max_y[0],
                            shift=0))
    # Steps
    for i in range(1, len(zk_min_x)):
        if i == len(zk_min_x) - 1:
            footsteps_x.append(Step(i * duration_step + duration_double_init,
                                    total_duration,
                                    z_min=zk_min_x[i],
                                    z_max=zk_max_x[i],
                                    shift=0))
            footsteps_y.append(Step(i * duration_step + duration_double_init,
                                    total_duration,
                                    z_min=zk_min_y[i],
                                    z_max=zk_max_y[i],
                                    shift=0))
        else:
            footsteps_x.append(Step(i*duration_step + duration_double_init,
                                    (i+1)*duration_step + duration_double_init,
                                    z_min=zk_min_x[i],
                                    z_max=zk_max_x[i],
                                    shift=0))
            footsteps_y.append(Step(i*duration_step + duration_double_init,
                                    (i+1)*duration_step + duration_double_init,
                                    z_min=zk_min_y[i],
                                    z_max=zk_max_y[i],
                                    shift=0))
    return footsteps_x, footsteps_y




