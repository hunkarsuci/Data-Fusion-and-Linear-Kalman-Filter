# Linear Kalman Filter Implementation

This directory contains the Python exercises for linear Kalman filtering.

## Assignment 1: vehicle tracking

Assignment 1 estimates two-dimensional position and velocity from noisy position measurements. The state is

```text
x = [position_x, position_y, velocity_x, velocity_y]^T
```

Files:

- `assignment1_initial_conditions.py` — initialization exercise.
- `assignment1_prediction.py` — state and covariance prediction.
- `assignment1_update.py` — measurement update.
- `assignment1_answer.py` — completed example.
- `kfsims/kfmodels.py` — filter state and covariance accessors.
- `kfsims/kftracker2d.py` — two-dimensional Kalman filter model.
- `kfsims/tracker2d.py` — simulation, plotting, and animation.
- `kfsims/vehiclemodel2d.py` — simulated vehicle motion model.

Run the completed example:

```powershell
cd LinearKalmanFilter_Implementation
python assignment1_answer.py
```

## Assignment 2: pendulum estimation

Assignment 2 simulates a nonlinear pendulum and estimates its angle and angular velocity with a linearized Kalman filter. The state is

```text
x = [angle, angular_velocity]^T
```

The linearized model is

```text
A = [[0, 1], [-9.81 / length, 0]]
F = expm(A * time_step)
H = [[1, 0]]
```

Only the pendulum angle is measured. The simulation reports innovation standard deviation, position mean-squared error, and velocity mean-squared error.

Files:

- `LinearKF_Pendelum/assignment2_filter.py` — completed filter and main entry point.
- `LinearKF_Pendelum/assignment2_filter_answer.py` — reference filter.
- `LinearKF_Pendelum/assignment2_sims.py` — standalone nonlinear/linear simulation.
- `LinearKF_Pendelum/assignment2_answer.py` — reference simulation entry point.
- `LinearKF_Pendelum/kfpendulum.py` — simulation loop, statistics, plots, and animation.
- `LinearKF_Pendelum/pendulum.py` — nonlinear pendulum model.
- `LinearKF_Pendelum/kfsims/kfmodels.py` — Kalman-filter base class.

Run Assignment 2 from the repository root:

```powershell
python LinearKalmanFilter_Implementation\LinearKF_Pendelum\assignment2_filter.py
```

Set `draw_plots` and `draw_animation` to `False` in `sim_options` when graphical output is not needed.

## Kalman filter lifecycle

Both assignments use the standard sequence:

```text
x_predict = F x_previous
P_predict = F P_previous F^T + Q
y = z - H x_predict
S = H P_predict H^T + R
K = P_predict H^T S^-1
x_update = x_predict + K y
P_update = (I - K H) P_predict
```

`Q` controls confidence in the motion model. `R` controls confidence in the measurement. Increasing `Q` makes the filter respond faster to model mismatch; increasing `R` makes it rely more on prediction.

## Dependencies

```powershell
python -m pip install numpy scipy matplotlib
```

See the [main README](../README.md) for the complete project overview and notebook instructions.
