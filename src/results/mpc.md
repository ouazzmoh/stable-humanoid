# mpc subset

| Version | 1.0.0 |
|:--------|:--------------------|
| Date    | 2023-07-31 17:08:23.894412+00:00 |
| CPU     | arm |
| Run by  | [@ouazzmoh](https://github.com/ouazzmoh/) |

## Contents

* [Description](#description)
* [Solvers](#solvers)
* [Settings](#settings)
* [Known limitations](#known-limitations)
* [Results by settings](#results-by-settings)
    * [Default](#default)
    * [High accuracy](#high-accuracy)
    * [Low accuracy](#low-accuracy)
* [Results by metric](#results-by-metric)
    * [Success rate](#success-rate)
    * [Computation time](#computation-time)
    * [Optimality conditions](#optimality-conditions)
        * [Primal residual](#primal-residual)
        * [Dual residual](#dual-residual)
        * [Duality gap](#duality-gap)
    * [Cost error](#cost-error)

## Description

Subset of the mpc_qp test set 

## Solvers

| solver   | version     |
|:---------|:------------|
| clarabel | 0.5.1       |
| cvxopt   | 1.3.1       |
| daqp     | 0.5.1       |
| ecos     | 2.0.12      |
| highs    | 1.5.3       |
| osqp     | 0.6.2.post9 |
| proxqp   | 0.4.0       |
| quadprog | 0.1.11      |
| scs      | 3.2.3       |

All solvers were called via
[qpsolvers](https://github.com/stephane-caron/qpsolvers)
v3.4.0.

## Settings

There are 3 settings: *default*, *high_accuracy*
and *low_accuracy*. They validate solutions using the following
tolerances:

| tolerance   |   default |   low_accuracy |   high_accuracy |
|:------------|----------:|---------------:|----------------:|
| ``cost``    |    0.0001 |         0.001  |           1e-05 |
| ``dual``    |    0.001  |         0.001  |           1e-07 |
| ``gap``     |    0.001  |         0.0001 |           0.001 |
| ``primal``  |    0.001  |         0.001  |           1e-07 |
| ``runtime`` |    1      |         1      |           1     |

Solvers for each settings are configured as follows:

| solver   | parameter                        | default   |   high_accuracy |   low_accuracy |
|:---------|:---------------------------------|:----------|----------------:|---------------:|
| clarabel | ``tol_feas``                     | -         |           1e-09 |          0.001 |
| clarabel | ``tol_gap_abs``                  | -         |           1e-09 |          0.001 |
| clarabel | ``tol_gap_rel``                  | -         |           0     |          0     |
| cvxopt   | ``feastol``                      | -         |           1e-09 |          0.001 |
| daqp     | ``dual_tol``                     | -         |           1e-09 |          0.001 |
| daqp     | ``primal_tol``                   | -         |           1e-09 |          0.001 |
| ecos     | ``feastol``                      | -         |           1e-09 |          0.001 |
| highs    | ``dual_feasibility_tolerance``   | -         |           1e-09 |          0.001 |
| highs    | ``primal_feasibility_tolerance`` | -         |           1e-09 |          0.001 |
| highs    | ``time_limit``                   | 1         |           1     |          1     |
| osqp     | ``eps_abs``                      | -         |           1e-09 |          0.001 |
| osqp     | ``eps_rel``                      | -         |           0     |          0     |
| osqp     | ``time_limit``                   | 1         |           1     |          1     |
| proxqp   | ``check_duality_gap``            | -         |           1     |          1     |
| proxqp   | ``eps_abs``                      | -         |           1e-09 |          0.001 |
| proxqp   | ``eps_duality_gap_abs``          | -         |           1e-09 |          0.001 |
| proxqp   | ``eps_duality_gap_rel``          | -         |           0     |          0     |
| proxqp   | ``eps_rel``                      | -         |           0     |          0     |
| scs      | ``eps_abs``                      | -         |           1e-09 |          0.001 |
| scs      | ``eps_rel``                      | -         |           0     |          0     |
| scs      | ``time_limit_secs``              | 1         |           1     |          1     |

## Known limitations

The following [issues](https://github.com/qpsolvers/qpsolvers_benchmark/issues)
have been identified as impacting the fairness of this benchmark. Keep them in
mind when drawing conclusions from the results.

- [#60](https://github.com/qpsolvers/qpsolvers_benchmark/issues/60):
  Conversion to SOCP limits performance of ECOS

## Results by settings

### Default

Solvers are compared over the whole test set by [shifted geometric
mean](../README.md#shifted-geometric-mean) (shm). Lower is better.

|          |   [Success rate](#success-rate) (%) |   [Runtime](#computation-time) (shm) |   [Primal residual](#primal-residual) (shm) |   [Dual residual](#dual-residual) (shm) |   [Duality gap](#duality-gap) (shm) |   [Cost error](#cost-error) (shm) |
|:---------|------------------------------------:|-------------------------------------:|--------------------------------------------:|----------------------------------------:|------------------------------------:|----------------------------------:|
| clarabel |                                 0.0 |                                 17.2 |                                         1.0 |                                   592.5 |                           2631082.5 |                               1.0 |
| cvxopt   |                                 0.0 |                                 73.1 |                                    428402.2 |                                   346.6 |                         223064762.3 |                               1.0 |
| daqp     |                                 0.0 |                                  1.8 |                                         1.0 |                                     1.0 |                                 1.1 |                               1.0 |
| ecos     |                                 0.0 |                                 40.5 |                                         1.5 |                          482593713425.1 |                      445816347931.1 |                               1.0 |
| highs    |                                 0.0 |                                 21.7 |                                         1.0 |                             746010230.2 |                         415277150.6 |                               1.0 |
| osqp     |                                 0.0 |                                  9.9 |                               11949397027.2 |                         1337953324034.4 |                      655836079253.6 |                               1.0 |
| proxqp   |                                 0.0 |                                  4.0 |                                 325191884.2 |                                505805.8 |                       21881895888.4 |                               1.0 |
| quadprog |                                 0.0 |                                  1.0 |                                         1.0 |                                     1.1 |                                 1.0 |                               1.0 |
| scs      |                                 0.0 |                                 14.9 |                                6372608248.0 |                           77418305564.8 |                       85879410892.3 |                               1.0 |

### High accuracy

Solvers are compared over the whole test set by [shifted geometric
mean](../README.md#shifted-geometric-mean) (shm). Lower is better.

|          |   [Success rate](#success-rate) (%) |   [Runtime](#computation-time) (shm) |   [Primal residual](#primal-residual) (shm) |   [Dual residual](#dual-residual) (shm) |   [Duality gap](#duality-gap) (shm) |   [Cost error](#cost-error) (shm) |
|:---------|------------------------------------:|-------------------------------------:|--------------------------------------------:|----------------------------------------:|------------------------------------:|----------------------------------:|
| clarabel |                                 0.0 |                                 20.4 |                                         1.0 |                                     1.8 |                                71.2 |                               1.0 |
| cvxopt   |                                 0.0 |                                 76.8 |                                      4708.5 |                                   374.8 |                          71470904.8 |                               1.0 |
| daqp     |                                 0.0 |                                  1.7 |                                         1.0 |                                     1.0 |                                 1.1 |                               1.0 |
| ecos     |                                 0.0 |                                112.1 |                                     70370.2 |                          135544662217.8 |                      128321762313.2 |                               1.0 |
| highs    |                                 0.0 |                                 22.3 |                                         1.0 |                             746010230.2 |                         415277150.6 |                               1.0 |
| osqp     |                                 0.0 |                                 27.3 |                                      1228.5 |                                  1257.6 |                              6355.0 |                               1.0 |
| proxqp   |                                 0.0 |                                  4.2 |                                        18.5 |                                    26.2 |                                89.9 |                               1.0 |
| quadprog |                                 0.0 |                                  1.0 |                                         1.0 |                                     1.1 |                                 1.0 |                               1.0 |
| scs      |                                 0.0 |                                100.5 |                                     70407.2 |                                  1801.9 |                           1913502.2 |                               1.0 |

### Low accuracy

Solvers are compared over the whole test set by [shifted geometric
mean](../README.md#shifted-geometric-mean) (shm). Lower is better.

|          |   [Success rate](#success-rate) (%) |   [Runtime](#computation-time) (shm) |   [Primal residual](#primal-residual) (shm) |   [Dual residual](#dual-residual) (shm) |   [Duality gap](#duality-gap) (shm) |   [Cost error](#cost-error) (shm) |
|:---------|------------------------------------:|-------------------------------------:|--------------------------------------------:|----------------------------------------:|------------------------------------:|----------------------------------:|
| clarabel |                                 0.0 |                                 17.5 |                                         1.0 |                                  2439.0 |                          71987231.2 |                               1.0 |
| cvxopt   |                                 0.0 |                                 75.7 |                                   1457151.5 |                                   346.4 |                         409596677.3 |                               1.0 |
| daqp     |                                 0.0 |                                  1.8 |                                         1.0 |                                     1.0 |                                 1.1 |                               1.0 |
| ecos     |                                 0.0 |                                 43.4 |                                         1.5 |                          592670148204.4 |                      538653849575.4 |                               1.0 |
| highs    |                                 0.0 |                                 22.2 |                                         1.0 |                             746010230.2 |                         415277150.6 |                               1.0 |
| osqp     |                                 0.0 |                                 15.7 |                                 359126675.0 |                            1185433395.6 |                        5336954484.3 |                               1.0 |
| proxqp   |                                 0.0 |                                  3.7 |                                  31101127.0 |                              23144156.7 |                         135976618.3 |                               1.0 |
| quadprog |                                 0.0 |                                  1.0 |                                         1.0 |                                     1.1 |                                 1.0 |                               1.0 |
| scs      |                                 0.0 |                                 20.9 |                                  20403489.2 |                             464285423.6 |                         100781986.9 |                               1.0 |

## Results by metric

### Success rate

Precentage of problems each solver is able to solve:

|          |   default |   high_accuracy |   low_accuracy |
|:---------|----------:|----------------:|---------------:|
| clarabel |         0 |               0 |              0 |
| cvxopt   |         0 |               0 |              0 |
| daqp     |         0 |               0 |              0 |
| ecos     |         0 |               0 |              0 |
| highs    |         0 |               0 |              0 |
| osqp     |         0 |               0 |              0 |
| proxqp   |         0 |               0 |              0 |
| quadprog |         0 |               0 |              0 |
| scs      |         0 |               0 |              0 |

Rows are [solvers](#solvers) and columns are [settings](#settings). We consider
that a solver successfully solved a problem when (1) it returned with a success
status and (2) its solution satisfies optimality conditions within
[tolerance](#settings). The second table below summarizes the frequency at
which solvers return success (1) and the corresponding solution did indeed pass
tolerance checks.

Percentage of problems where "solved" return codes are correct:

|          |   default |   high_accuracy |   low_accuracy |
|:---------|----------:|----------------:|---------------:|
| clarabel |         0 |               0 |              0 |
| cvxopt   |         0 |               0 |              0 |
| daqp     |         0 |               0 |              0 |
| ecos     |         0 |               0 |              0 |
| highs    |         0 |               0 |              0 |
| osqp     |         0 |               0 |              0 |
| proxqp   |         0 |               0 |              0 |
| quadprog |         0 |               0 |              0 |
| scs      |         0 |               0 |              0 |

### Computation time

We compare solver computation times over the whole test set using the shifted
geometric mean. Intuitively, a solver with a shifted-geometric-mean runtime of
Y is Y times slower than the best solver over the test set. See
[Metrics](../README.md#metrics) for details.

Shifted geometric mean of solver computation times (1.0 is the best):

|          |   default |   high_accuracy |   low_accuracy |
|:---------|----------:|----------------:|---------------:|
| clarabel |      17.2 |            20.4 |           17.5 |
| cvxopt   |      73.1 |            76.8 |           75.7 |
| daqp     |       1.8 |             1.7 |            1.8 |
| ecos     |      40.5 |           112.1 |           43.4 |
| highs    |      21.7 |            22.3 |           22.2 |
| osqp     |       9.9 |            27.3 |           15.7 |
| proxqp   |       4.0 |             4.2 |            3.7 |
| quadprog |       1.0 |             1.0 |            1.0 |
| scs      |      14.9 |           100.5 |           20.9 |

Rows are solvers and columns are solver settings. The shift is $sh = 10$. As in
the OSQP and ProxQP benchmarks, we assume a solver's run time is at the [time
limit](#settings) when it fails to solve a problem.

### Optimality conditions

#### Primal residual

The primal residual measures the maximum (equality and inequality) constraint
violation in the solution returned by a solver. We use the shifted geometric
mean to compare solver primal residuals over the whole test set. Intuitively, a
solver with a shifted-geometric-mean primal residual of Y is Y times less
precise on constraints than the best solver over the test set. See
[Metrics](../README.md#metrics) for details.

Shifted geometric means of primal residuals (1.0 is the best):

|          |       default |   high_accuracy |   low_accuracy |
|:---------|--------------:|----------------:|---------------:|
| clarabel |           1.0 |             1.0 |            1.0 |
| cvxopt   |      428402.2 |          4708.5 |      1457151.5 |
| daqp     |           1.0 |             1.0 |            1.0 |
| ecos     |           1.5 |         70370.2 |            1.5 |
| highs    |           1.0 |             1.0 |            1.0 |
| osqp     | 11949397027.2 |          1228.5 |    359126675.0 |
| proxqp   |   325191884.2 |            18.5 |     31101127.0 |
| quadprog |           1.0 |             1.0 |            1.0 |
| scs      |  6372608248.0 |         70407.2 |     20403489.2 |

Rows are solvers and columns are solver settings. The shift is $sh = 10$. A
solver that fails to find a solution receives a primal residual equal to the
full [primal tolerance](#settings).

#### Dual residual

The dual residual measures the maximum violation of the dual feasibility
condition in the solution returned by a solver. We use the shifted geometric
mean to compare solver dual residuals over the whole test set. Intuitively, a
solver with a shifted-geometric-mean dual residual of Y is Y times less precise
on the dual feasibility condition than the best solver over the test set. See
[Metrics](../README.md#metrics) for details.

Shifted geometric means of dual residuals (1.0 is the best):

|          |         default |   high_accuracy |   low_accuracy |
|:---------|----------------:|----------------:|---------------:|
| clarabel |           592.5 |             1.8 |         2439.0 |
| cvxopt   |           346.6 |           374.8 |          346.4 |
| daqp     |             1.0 |             1.0 |            1.0 |
| ecos     |  482593713425.1 |  135544662217.8 | 592670148204.4 |
| highs    |     746010230.2 |     746010230.2 |    746010230.2 |
| osqp     | 1337953324034.4 |          1257.6 |   1185433395.6 |
| proxqp   |        505805.8 |            26.2 |     23144156.7 |
| quadprog |             1.1 |             1.1 |            1.1 |
| scs      |   77418305564.8 |          1801.9 |    464285423.6 |

Rows are solvers and columns are solver settings. The shift is $sh = 10$. A
solver that fails to find a solution receives a dual residual equal to the full
[dual tolerance](#settings).

#### Duality gap

The duality gap measures the consistency of the primal and dual solutions
returned by a solver. A duality gap close to zero ensures that the
complementarity slackness optimality condition is satisfied. We use the shifted
geometric mean to compare solver duality gaps over the whole test set.
Intuitively, a solver with a shifted-geometric-mean duality gap of Y is Y times
less precise on the complementarity slackness condition than the best solver
over the test set. See [Metrics](../README.md#metrics) for details.

Shifted geometric means of duality gaps (1.0 is the best):

|          |        default |   high_accuracy |   low_accuracy |
|:---------|---------------:|----------------:|---------------:|
| clarabel |      2631082.5 |            71.2 |     71987231.2 |
| cvxopt   |    223064762.3 |      71470904.8 |    409596677.3 |
| daqp     |            1.1 |             1.1 |            1.1 |
| ecos     | 445816347931.1 |  128321762313.2 | 538653849575.4 |
| highs    |    415277150.6 |     415277150.6 |    415277150.6 |
| osqp     | 655836079253.6 |          6355.0 |   5336954484.3 |
| proxqp   |  21881895888.4 |            89.9 |    135976618.3 |
| quadprog |            1.0 |             1.0 |            1.0 |
| scs      |  85879410892.3 |       1913502.2 |    100781986.9 |

Rows are solvers and columns are solver settings. The shift is $sh = 10$. A
solver that fails to find a solution receives a duality gap equal to the full
[gap tolerance](#settings).

### Cost error

The cost error measures the difference between the known optimal objective and
the objective at the solution returned by a solver. We use the shifted
geometric mean to compare solver cost errors over the whole test set.
Intuitively, a solver with a shifted-geometric-mean cost error of Y is Y times
less precise on the optimal cost than the best solver over the test set. See
[Metrics](../README.md#metrics) for details.

Shifted geometric means of solver cost errors (1.0 is the best):

|          |   default |   high_accuracy |   low_accuracy |
|:---------|----------:|----------------:|---------------:|
| clarabel |       1.0 |             1.0 |            1.0 |
| cvxopt   |       1.0 |             1.0 |            1.0 |
| daqp     |       1.0 |             1.0 |            1.0 |
| ecos     |       1.0 |             1.0 |            1.0 |
| highs    |       1.0 |             1.0 |            1.0 |
| osqp     |       1.0 |             1.0 |            1.0 |
| proxqp   |       1.0 |             1.0 |            1.0 |
| quadprog |       1.0 |             1.0 |            1.0 |
| scs      |       1.0 |             1.0 |            1.0 |

Rows are solvers and columns are solver settings. The shift is $sh = 10$. A
solver that fails to find a solution receives a cost error equal to the [cost
tolerance](#settings).
