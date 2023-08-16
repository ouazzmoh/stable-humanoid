## Stable Walking for humanoid robot

This project provides an implementation of an MPC for bipedal robot walking. The controller allows the robot to maintain its balance while walking at different speeds. The controller also features robustness against disturbances and the ability to react to changes in the environment.
The key steps in the MPC approach are:

1. **Prediction:** Based on the current state of the robot and the control inputs, the MPC predicts the future states of the robot using the system model.
2. **Optimization:** The MPC formulates an optimization problem to minimize the difference between the predicted state and the desired state. The optimization problem includes minimization of the jerk, velocity and proximity to reference trajectories.
3. **Implementation:** The MPC applies the optimal control actions to the robot.

After the MPC is used to generate the stable walking trajectory, we use inverse kinematics via the `pink` library to control the joints of the robot and simulate walking.

## Requirements

- Python 3.x
- Additional dependencies are listed below

### Dependencies

The following packages are required:

- pink
- numpy
- pinocchio
- matplotlib
- qpsolvers
- meshcat_shapes
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

### src

This folder contains the source code for the MPC and inverse kinematics.

#### bezier_curve : 
Generates the curve between two points using the Bezier curve algorithm, which is used for the path of the foot

#### com_task : 
The pink task used to track the center of mass of the robot

#### controller:
The MPC controller that generates the optimal control actions for the robot

#### footstep_planner:
Offers different footstep planning algorithms for the robot, currently it offers : forward, upwards, upwards_turning,
circle(Not stable for inverse kinematics), interactive(Not completely stable)
Could be extended either by merging the existing scenarios or by adding new ones.

#### main:
Main script that runs the MPC and inverse kinematics for the robot

#### move_foot:
Applies inverse kinematics on foot to move it to the desired position

#### mpc: 
Benchmarking script for the MPC

#### mpc_qp:
Benchmarking script for the MPC and inverse kinematics

#### no-viz:
Script that runs the MPC without inverse kinematics

#### no-viz-circle:
Script that runs the MPC without inverse kinematics for the circular trajectory

#### perturbation:
A class that allows adding perturbations to the robot during the simulation

#### qp:
Bechmarking script for the inverse kinematics

#### robot:
Class that represents a simplified version of the robot.

#### scenarios:
Holds different functions for generating usual COP reference trajectories

#### step:
Represent a step of the robot during a certain time

#### utils:
Contains different utility functions used for the MPC

#### visuals:
Contains multiple visualization functions








