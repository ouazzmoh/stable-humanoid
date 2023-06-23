# Stable Humanoid

## Solving the QP problem in one dimension


### Analytical resolution

![Example Image](plots/predictive_control.png)

### QP Resolution

Using the QUADPROG solver from the library `qpsolvers`

![Example Image](plots/qp.png)

### Analytical resolution with perturbations

perturbation of `1 m.s-2` at `2.5s`

![Example Image](plots/perturbations.png)

### QP Resolution with perturbations

perturbation of `1 m.s-2` at `2.5s`
Contrary to the ~~analytical method, the ro~~bot stays stable

![Example Image](plots/qp_perturbations.png)

## Solving the QP problem in 2 dimensions 

### Decoupling lateral (y) and forward (x)

If we consider that the robot alternates in the two dimensions, get the following result
![Example Image](plots/forward.png)
![Example Image](plots/decoupled.png)
Using the QP solver in this solution gives us this result while using the analytical method 
to solve the problem gives a perfect linear result


If we consider that the robot alternates laterally but is moving forward we get the
following result
![Example Image](plots/forward_moving.png)
![Example Image](plots/decoupled_moving.png)

The next step is coupling x and y in the same QP problem and solving it

### Coupling lateral (y) and forward (x)

In this part we will relax the hypothesis that x and y are independant, and solve a QP problem 
that accounts for both dimensions
the new QP that we get is the following:
![Example Image](qp.png)

the results we get are the following 

for `theta = 0` \\
![Example Image](plots/coupled_theta_0.png)

for `theta = pi/4` \\

![Example Image](plots/coupled_theta_pi:4.png)

If we change the reference of steps for the lateral and forward motion to follow this : \\

![Example Image](plots/zkrefs.png)

We get the following results \\

![Example Image](plots/forward2.png)

![Example Image](plots/traj2.png)

#### The results for a circular trajectory

To get a circular trajectory, we create step references that ressemble the sinus and cosinus
functions.

![Example Image](plots/zkref_circle.png)
![Example Image](plots/lateral_sin.png)
![Example_Image](plots/forward_cos.png)
![Example Image](plots/circle.png)
![Example Image](plots/traj3.png)


##### Speed minimization

Minimizing speed alone
![Example Image](plots/speed1.png)

Minimizing speed and smoothing with jerk and step references
![Example Image](plots/spped2.png)
