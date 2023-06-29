import numpy as np
from step import Step
from typing import List

def construct_zmin_zmax(total_duration, duration_double_init, duration_step,
                        foot_size, spacing) -> List[Step]:
    """
    Construct the minimum and maximum for the center of pressure
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
                          z_min=-(foot_size + spacing),
                          z_max=foot_size + spacing,
                          shift=0))
    for step_i in range(number_of_steps):
        if step_i % 2 == 0:
            # Left support
            footsteps.append(Step(step_i*duration_step + duration_double_init,
                                  (step_i+1)*duration_step + duration_double_init,
                                  z_min=-(foot_size + spacing),
                                  z_max=0,
                                  shift=0))
        else:
            # Right support
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  (step_i + 1) * duration_step + duration_double_init,
                                  z_min=0,
                                  z_max=foot_size + spacing,
                                  shift=0))
    return footsteps



def construct_zmin_zmax_forward(total_duration, duration_double_init, duration_step, foot_size, spacing):
    """
        The robot moves forward
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
        :return: two arrays z_min and z_max
        """

    number_of_steps = round((total_duration - duration_double_init) / duration_step)
    footsteps: list[Step] = []
    # Initial double support
    footsteps.append(Step(0,
                          duration_double_init,
                          z_min=-(foot_size + spacing),
                          z_max=foot_size + spacing,
                          shift=0))
    # Step on other direction: keep the double support here
    footsteps.append(Step(duration_double_init,
                          duration_double_init+duration_step,
                          z_min=-(foot_size + spacing),
                          z_max=foot_size + spacing,
                          shift=0))

    for step_i in range(1, number_of_steps):
        footsteps.append(Step(step_i * duration_step + duration_double_init,
                              (step_i + 1) * duration_step + duration_double_init,
                              z_min=(step_i - 1)*(foot_size + spacing),
                              z_max=step_i*(foot_size + spacing),
                              shift=0))

    return footsteps




def construct_zmin_zmax_moving2(steps, duration_double_init, duration_step,
                                foot_size, spacing, duration_back):
    """
        The robot moves forward to a certain point, then moves backwards
        Construct the minimum and maximum for the center of pressure
        This is for a moving robot
        The duration of the support is expressed as percentage of steps
        The values of the double support are in [min_val_left, max_val_right]
        :param steps: number of steps of the whole simulation
               duration_back: the moment where the robot starts going backwards
        :return: two arrays z_min and z_max
        """

    foot_size += spacing

    # Initial double support
    zk_min = [-foot_size] * int(steps * duration_double_init)
    zk_max = [foot_size] * int(steps * duration_double_init)

    # Lifting foot first step
    zk_min += [-foot_size]*int(steps*duration_step)
    zk_max += [0] * int(steps*duration_step)

    # Number of steps to take
    number_of_steps = int((steps - len(zk_min))/((duration_step)*steps))

    for step_number in range(1, int(duration_back * number_of_steps)):
        # Lifting foot for a step
        zk_min += [(step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [step_number * foot_size] * int(steps * duration_step)

    #The robot starts moving backwards
    for step_number in range(int(duration_back * number_of_steps), number_of_steps):
        # Lifting foot for a step
        zk_min += [(number_of_steps - step_number - 1) * foot_size] * int(steps * duration_step)
        zk_max += [(number_of_steps-1 - step_number+1) * foot_size] * int(steps * duration_step)

    zk_min += [zk_min[-1]] * abs(steps - len(zk_min))
    zk_max += [zk_max[-1]] * abs(steps - len(zk_max))

    return np.array(zk_min), np.array(zk_max)