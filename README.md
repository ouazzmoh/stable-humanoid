## Stable Walking for humanoid robot
This project implements a stable walking controller for a humanoid robot using **Model Predictive Control (MPC)** for trajectory planning and **Inverse Kinematics (IK)** for joint control. 

## Model Predictive Control (MPC)
The controller allows the robot to maintain its balance while walking at different speeds. The controller also features robustness against disturbances and the ability to react to changes in the environment.
The key steps in the MPC approach are:

1. **Prediction:** Based on the current state of the robot and the control inputs, the MPC predicts the future states of the robot using the system model.
2. **Optimization:** The MPC formulates an optimization problem to minimize the difference between the predicted state and the desired state. The optimization problem includes minimization of the jerk, velocity and proximity to reference trajectories.
3. **Implementation:** The MPC applies the optimal control actions to the robot.

## Inverse Kinematics (IK)

Once the walking trajectory is generated, **Inverse Kinematics** is used to control the robot's joints via the **Pink** library. IK calculates the joint angles required to achieve specific target positions while considering constraints such as joint limits and smooth transitions.

The key steps in Inverse Kinematics (IK) are:

1. Taking the output of the controller (the succession of steps that the robot should follow).
2. Creating smooth foot trajectories between steps using Bézier curves.
3. Solving Quadratic Programming (QP) problems to follow the trajectory while maintaining balance and respecting joint constraints.

## Requirements

- Python 3.10
- Additional dependencies are listed below

### Dependencies

The following packages are required:

- qpsolvers
- loop_rate_limiters
- pinocchio
- pink
- meshcat_shapes
- matplotlib
- h5py
- robot_descriptions

To install these dependencies, you can run the following command:


```pip install -r requirements.txt```

## How to Run the MPC and inverse Kinematics for humanoid robot

### Arguments

- `trajectory_type` (required): The type of trajectory for the humanoid robot. Must be one of `forward`, `upwards`, or `upwards_turning`.
- `--debug` (optional): Include this flag to show intermediate plots during the simulation.
- `--store` (optional): Include this flag to store the QP (Quadratic Programming) data generated during the simulation.
- `--filename` (optional): Specify a filename (e.g., `output.txt`) to save the QP data. If not provided, the QP data will not be saved.

### Usage

Navigate to the directory containing `main.py` and run the script with the following command:

```./main.py trajectory_type [--debug] [--store] [--filename FILENAME]```

### Examples

1. To simulate a forward walking trajectory without debugging plots:

```./main.py forward```

2. To simulate an upwards walking trajectory and save the QP data with a specific filename:

```./main.py upwards --store --filename UpwardsWalk```

3. To simulate an upwards turning trajectory with debugging plots:

```./main.py upwards_turning --debug```


#### N.B: 

There is also a `circle` scenario which is not implemented yet with the inverse kinematics.
To run it 
```bash
./no-viz-circle.py
```


## Results on existing scenarios

**Moving Forward:**

<table>
  <tr>
    <td><img src="./plots/forward1.png" alt="Moving Forward 1" width="400"/></td>
    <td><img src="./plots/forward2.png" alt="Moving Forward 2" width="400"/></td>
  </tr>
  <tr>
    <td><img src="./plots/forward3.png" alt="Moving Forward 3" width="400"/></td>
    <td><img src="./plots/forward4.png" alt="Moving Forward 4" width="400"/></td>
  </tr>
</table>

**Moving Upwards:**

<table>
  <tr>
    <td><img src="./plots/upwards1.png" alt="Moving Forward 1" width="400"/></td>
    <td><img src="./plots/upwards2.png" alt="Moving Forward 2" width="400"/></td>
  </tr>
  <tr>
    <td><img src="./plots/upwards3.png" alt="Moving Forward 3" width="400"/></td>
    <td><img src="./plots/upwards4.png" alt="Moving Forward 4" width="400"/></td>
  </tr>
</table>

**Moving Upwards and Turning:**

<table>g
  <tr>
    <td><img src="./plots/upwards_turning1.png" alt="Moving Forward 1" width="400"/></td>
    <td><img src="./plots/upwards_turning2.png" alt="Moving Forward 2" width="400"/></td>
  </tr>
  <tr>
    <td><img src="./plots/upwards_turning3.png" alt="Moving Forward 3" width="400"/></td>
    <td><img src="./plots/upwards_turning4.png" alt="Moving Forward 4" width="400"/></td>
  </tr>
</table>

**Circular Trajectory:**


<table>
  <tr>
    <td><img src="./plots/circle1.png" alt="Moving Forward 1" width="400"/></td>
    <td><img src="./plots/circle2.png" alt="Moving Forward 2" width="400"/></td>
  </tr>
  <tr>
    <td><img src="./plots/circle3.png" alt="Moving Forward 3" width="400"/></td>
    <td><img src="./plots/circle4.png" alt="Moving Forward 4" width="400"/></td>
  </tr>
</table>



## How to run the benchmark 

Firstly if not done install `qpsolvers_benchmark`

You can install the benchmark and its dependencies in an isolated environment using conda:

```bash
conda create -f environment.yaml
conda activate qpsolvers_benchmark
```
Alternatively, you can install the benchmark on your system using pip:

```bash
pip install qpsolvers_benchmark
```
By default, the benchmark will run all supported solvers it finds.

Then run the benchmark with the following command:

```bash
qpsolvers_benchmark src/mpc_qp.py run
```

This will run the benchmark on the scenarios stored as `hdf5` files in the `data` folder. 
The results will be stored in the `results` folder.
For significant results change the error tolerances in mpc_qp.py based on the specific scenario.


## Project Hierarchy

#### src/bezier_curve.py : 
Generates the curve between two points using the Bezier curve algorithm, which is used for the path of the foot

#### src/com_task.py : 
The pink task used to track the center of mass of the robot

#### src/controller.py:
The MPC controller that generates the optimal control actions for the robot

#### src/footstep_planner.py:
Offers different footstep planning algorithms for the robot, currently it offers : forward, upwards, upwards_turning,
circle(Not stable for inverse kinematics), interactive(Not completely stable)
Could be extended either by merging the existing scenarios or by adding new ones.

#### src/main.py:
Main script that runs the MPC and inverse kinematics for the robot

#### src/move_foot.py:
Applies inverse kinematics on foot to move it to the desired position

#### src/mpc.py: 
Benchmarking script for the MPC

#### src/mpc_qp.py:
Benchmarking script for the MPC and inverse kinematics

#### src/no-viz.py:
Script that runs the MPC without inverse kinematics

#### src/no-viz-circle.py:
Script that runs the MPC without inverse kinematics for the circular trajectory

#### src/perturbation.py:
A class that allows adding perturbations to the robot during the simulation

#### src/qp.py:
Bechmarking script for the inverse kinematics

#### src/robot.py:
Class that represents a simplified version of the robot.

#### src/scenarios.py:
Holds different functions for generating usual COP reference trajectories

#### src/step.py:
Represent a step of the robot during a certain time

#### src/utils.py:
Contains different utility functions used for the MPC

#### src/visuals.py:
Contains multiple visualization functions


## Notes on continuing the project

- The adaptation of the footsteps is in the branch `adapting-3`, it is mostly implemented, but the solution is not stable yet.
The reason is most likely because there should be an additional constraint on the Y of the robot so it doesn't choose the trivial
solution of 0 as a solution. 
- The inverse kinematics of the circular trajectory is not stable. To solve the issue check the dimensions of the foot 
and the initial position of the com.








