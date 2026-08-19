# Data Fusion and Linear Kalman Filter

[![CI](https://github.com/hunkarsuci/Data-Fusion-and-Linear-Kalman-Filter/actions/workflows/ci.yml/badge.svg)](https://github.com/hunkarsuci/Data-Fusion-and-Linear-Kalman-Filter/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-orange?logo=jupyter&logoColor=white)](https://jupyter.org/)
[![NumPy](https://img.shields.io/badge/NumPy-supported-013243?logo=numpy&logoColor=white)](https://numpy.org/)
[![SciPy](https://img.shields.io/badge/SciPy-supported-8CAAE6?logo=scipy&logoColor=white)](https://scipy.org/)
[![Matplotlib](https://img.shields.io/badge/Matplotlib-supported-11557C?logo=matplotlib&logoColor=white)](https://matplotlib.org/)

Examples and Python implementations for dynamic-system simulation, data fusion, least-squares estimation, and linear Kalman filtering.

## Quick start

Install the dependencies:

```powershell
python -m pip install numpy scipy matplotlib jupyter
```

Run the notebooks with `jupyter notebook`, or run the completed pendulum filter from the repository root:

```powershell
python LinearKalmanFilter_Implementation\LinearKF_Pendelum\assignment2_filter.py
```

## Contents

| Path | Purpose |
| --- | --- |
| [`ContinousTimeSimulation.ipynb`](ContinousTimeSimulation.ipynb) | Continuous-time mass-spring-damper simulation using forward Euler integration and comparison with the exact response. |
| [`DiscreteTimeSimulation.ipynb`](DiscreteTimeSimulation.ipynb) | Exact discretization and discrete-time simulation of the mass-spring-damper model. |
| [`LeastSquareEstimation.ipynb`](LeastSquareEstimation.ipynb) | Ordinary and weighted least-squares estimation with measurement uncertainty. |
| [`LinearKalmanFilter_Implementation/`](LinearKalmanFilter_Implementation/) | Python exercises for vehicle tracking and pendulum estimation. |

## Assignment 1: two-dimensional vehicle tracking

Assignment 1 estimates

```text
x = [position_x, position_y, velocity_x, velocity_y]^T
```

from noisy two-dimensional position measurements using a constant-velocity model. The scripts are `assignment1_initial_conditions.py`, `assignment1_prediction.py`, `assignment1_update.py`, and the completed `assignment1_answer.py`.

Run the completed example:

```powershell
cd LinearKalmanFilter_Implementation
python assignment1_answer.py
```

## Assignment 2: pendulum estimation

Assignment 2 compares a nonlinear pendulum simulation with a linearized Kalman filter. The state is

```text
x = [angle, angular_velocity]^T
```

The nonlinear model uses `angle_acceleration = -(g / length) * sin(angle)`. The linear filter uses

```text
A = [[0, 1], [-g / length, 0]]
F = expm(A * time_step)
H = [[1, 0]]
```

Only the angle is measured. The simulation reports innovation standard deviation, position mean-squared error, and velocity mean-squared error, and can display analysis plots and an animation.

Important Assignment 2 files:

- `assignment2_filter.py`: completed filter and main entry point.
- `assignment2_filter_answer.py`: reference filter.
- `assignment2_sims.py`: standalone nonlinear-versus-linear simulation.
- `assignment2_answer.py`: reference simulation entry point.
- `kfpendulum.py`: simulation runner, plots, and animation.
- `pendulum.py`: nonlinear pendulum model.
- `kfsims/kfmodels.py`: Kalman-filter base class.

Set `draw_plots` and `draw_animation` to `False` in `sim_options` when graphical output is not needed.

## Kalman filter cycle

Both assignments use the standard recursion:

```text
x_predict = F x_previous
P_predict = F P_previous F^T + Q
y = z - H x_predict
S = H P_predict H^T + R
K = P_predict H^T S^-1
x_update = x_predict + K y
P_update = (I - K H) P_predict
```

`Q` controls confidence in the motion model; `R` controls confidence in the measurement. Increasing `Q` makes the filter respond faster to model mismatch, while increasing `R` makes it rely more on prediction.

## Repository structure

```text
.
├── ContinousTimeSimulation.ipynb
├── DiscreteTimeSimulation.ipynb
├── LeastSquareEstimation.ipynb
├── LinearKalmanFilter_Implementation
│   ├── README.md
│   ├── assignment1_*.py
│   └── LinearKF_Pendelum
│       ├── assignment2_filter.py
│       ├── assignment2_filter_answer.py
│       ├── assignment2_sims.py
│       ├── kfpendulum.py
│       ├── pendulum.py
│       └── kfsims
│           └── kfmodels.py
├── LICENSE
└── README.md
```

## Troubleshooting

Run Assignment 2 from the repository root using the command above. The pendulum runner is in `kfpendulum.py`, and the physical model is in `pendulum.py`; older references to `kfsims.pendulum` are no longer valid.

See [`LICENSE`](LICENSE) for licensing information.
