import numpy as np
from step import Step
from typing import List

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
                          z_min=-(foot_size + spacing),
                          z_max=foot_size + spacing,
                          shift=0))
    for step_i in range(number_of_steps):
        if step_i % 2 == 0:
            if step_i == number_of_steps - 1:
                # Left support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=-(foot_size + spacing),
                                      z_max=0,
                                      shift=0))
            else:
                # Left support
                footsteps.append(Step(step_i*duration_step + duration_double_init,
                                      (step_i+1)*duration_step + duration_double_init,
                                      z_min=-(foot_size + spacing),
                                      z_max=0,
                                      shift=0))
        else:
            if step_i == number_of_steps - 1:
                # Right support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=0,
                                      z_max=foot_size + spacing,
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
        if step_i == number_of_steps - 1:
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  total_duration,
                                  z_min=(step_i - 1) * (foot_size + spacing),
                                  z_max=step_i * (foot_size + spacing),
                                  shift=0))
        else :
            # Complete with the last value until the end of the simulation
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  (step_i + 1) * duration_step + duration_double_init,
                                  z_min=(step_i - 1)*(foot_size + spacing),
                                  z_max=step_i*(foot_size + spacing),
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
                          z_min=-(foot_size + spacing),
                          z_max=foot_size + spacing,
                          shift=0))
    for step_i in range(number_of_steps):
        if step_i % 2 == 0:
            if step_i == number_of_steps - 1:
                # Left support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=-(foot_size + spacing) + step_i * shift,
                                      z_max=0 + step_i * shift,
                                      shift=0))
            else:
                # Left support
                footsteps.append(Step(step_i*duration_step + duration_double_init,
                                      (step_i+1)*duration_step + duration_double_init,
                                      z_min=-(foot_size + spacing) + step_i * shift,
                                      z_max=0 + step_i * shift,
                                      shift=0))
        else:
            if step_i == number_of_steps - 1:
                # Right support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      total_duration,
                                      z_min=0 + step_i * shift,
                                      z_max=foot_size + spacing + step_i * shift,
                                      shift=0))
            else:
                # Right support
                footsteps.append(Step(step_i * duration_step + duration_double_init,
                                      (step_i + 1) * duration_step + duration_double_init,
                                      z_min=0 + step_i * shift,
                                      z_max=foot_size + spacing + step_i * shift,
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
                          z_min=-(foot_size + spacing),
                          z_max=foot_size + spacing,
                          shift=0))
    # Step on other direction: keep the double support here
    footsteps.append(Step(duration_double_init,
                          duration_double_init+duration_step,
                          z_min=-(foot_size + spacing),
                          z_max=foot_size + spacing,
                          shift=0))
    for step_i in range(1, int(0.5*number_of_steps)):
        # Complete with the last value until the end of the simulation
        footsteps.append(Step(step_i * duration_step + duration_double_init,
                              (step_i + 1) * duration_step + duration_double_init,
                              z_min=(step_i - 1) * (foot_size + spacing),
                              z_max=step_i * (foot_size + spacing),
                              shift=0))


    for step_i in range(int(number_of_steps*0.5), number_of_steps):
        if step_i == number_of_steps - 1:
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  total_duration,
                                  z_min=(number_of_steps - step_i - 1) * (foot_size + spacing),
                                  z_max=(number_of_steps - step_i) * (foot_size + spacing),
                                  shift=0))
        else:
            footsteps.append(Step(step_i * duration_step + duration_double_init,
                                  (step_i + 1) * duration_step + duration_double_init,
                                  z_min=(number_of_steps - step_i - 1)*(foot_size + spacing),
                                  z_max=(number_of_steps - step_i)*(foot_size + spacing),
                                  shift=0))
    return footsteps


