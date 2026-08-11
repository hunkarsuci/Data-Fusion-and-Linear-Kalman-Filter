# Data Fusion and Linear Kalman Filter

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Jupyter](https://img.shields.io/badge/Jupyter-notebooks-F37626?logo=jupyter&logoColor=white)](https://jupyter.org/)
[![NumPy](https://img.shields.io/badge/NumPy-supported-013243?logo=numpy&logoColor=white)](https://numpy.org/)
[![SciPy](https://img.shields.io/badge/SciPy-supported-8CAAE6?logo=scipy&logoColor=white)](https://scipy.org/)
[![GitHub last commit](https://img.shields.io/github/last-commit/hunkarsuci/Data-Fusion-and-Linear-Kalman-Filter)](https://github.com/hunkarsuci/Data-Fusion-and-Linear-Kalman-Filter/commits/master)

Examples and implementations for dynamic-system modeling, state-space simulation, data fusion, and linear state estimation.

## Contents

| Resource | Description |
| --- | --- |
| [`ContinousTimeSimulation.ipynb`](ContinousTimeSimulation.ipynb) | Forward Euler simulation of a continuous-time mass-spring-damper system, compared with its exact response. |
| [`DiscreteTimeSimulation.ipynb`](DiscreteTimeSimulation.ipynb) | Exact discretization and simulation of the same mass-spring-damper model. |
| [`LeastSquareEstimation.ipynb`](LeastSquareEstimation.ipynb) | Ordinary and weighted least-squares estimation using measurement covariance. |
| [`LinearKalmanFilter_Implementation/`](LinearKalmanFilter_Implementation/) | Linear Kalman filter for two-dimensional vehicle tracking. |

## Topics

- Continuous-time and discrete-time state-space models
- Numerical simulation and exact discretization
- Least-squares and weighted least-squares estimation
- Linear Kalman filtering and data fusion
- State prediction and covariance propagation
- Measurement updates and uncertainty weighting

## Linear Kalman Filter state estimation

The implementation estimates a vehicle's two-dimensional position and velocity from noisy position measurements. It uses a linear dynamic model and a linear measurement model. At every time step, the filter predicts the state and its uncertainty, then corrects both using the new measurement.

### 1. Dynamic state model

The state vector is

```math
\mathbf{x}_k =
\begin{bmatrix}
p_{x,k} \\
p_{y,k} \\
v_{x,k} \\
v_{y,k}
\end{bmatrix},
```

where `p_x` and `p_y` are positions and `v_x` and `v_y` are velocities. The stochastic discrete-time model is

```math
\mathbf{x}_k = \mathbf{F}_k\mathbf{x}_{k-1} + \mathbf{w}_k,
\qquad
\mathbf{w}_k \sim \mathcal{N}(\mathbf{0},\mathbf{Q}_k).
```

For a constant-velocity model with sampling interval `dt`,

```math
\mathbf{F}_k =
\begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
```

The process noise `w_k` represents acceleration and other motion that the constant-velocity model does not predict. Its covariance is `Q_k`.

For independent continuous white acceleration in the `x` and `y` directions, with spectral density `q_a`, the standard discrete process-noise covariance is

```math
\mathbf{Q}_k = q_a
\begin{bmatrix}
\frac{\Delta t^3}{3} & 0 & \frac{\Delta t^2}{2} & 0 \\
0 & \frac{\Delta t^3}{3} & 0 & \frac{\Delta t^2}{2} \\
\frac{\Delta t^2}{2} & 0 & \Delta t & 0 \\
0 & \frac{\Delta t^2}{2} & 0 & \Delta t
\end{bmatrix}.
```

The current Python implementation uses a simplified diagonal tuning matrix:

```math
\mathbf{Q}_{\mathrm{code}} = \sigma_a^2
\mathrm{diag}\!\left(
\frac{\Delta t^2}{2},
\frac{\Delta t^2}{2},
\Delta t,
\Delta t
\right).
```

Here, `accel_std` supplies the tuning value `sigma_a`. Increasing it makes the filter less confident in constant-velocity motion and more responsive to maneuvers.

### 2. Measurement model

The sensor measures position but not velocity:

```math
\mathbf{z}_k = \mathbf{H}_k\mathbf{x}_k + \mathbf{v}_k,
\qquad
\mathbf{v}_k \sim \mathcal{N}(\mathbf{0},\mathbf{R}_k),
```

with

```math
\mathbf{z}_k =
\begin{bmatrix}
z_{x,k} \\
z_{y,k}
\end{bmatrix},
\qquad
\mathbf{H}_k =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}.
```

For independent measurement errors with standard deviation `sigma_m`,

```math
\mathbf{R}_k =
\begin{bmatrix}
\sigma_m^2 & 0 \\
0 & \sigma_m^2
\end{bmatrix}.
```

The code parameter `meas_std` supplies `sigma_m`. A larger `R_k` gives the sensor less influence; a smaller `R_k` gives it more influence.

### 3. Prior and posterior estimates

The notation distinguishes the estimate before and after the measurement at time `k`:

```math
\hat{\mathbf{x}}_{k\mid k-1}
= \mathbb{E}[\mathbf{x}_k\mid\mathbf{z}_{1:k-1}]
```

is the prior (predicted) estimate, while

```math
\hat{\mathbf{x}}_{k\mid k}
= \mathbb{E}[\mathbf{x}_k\mid\mathbf{z}_{1:k}]
```

is the posterior (corrected) estimate. Their error covariances are

```math
\mathbf{P}_{k\mid j}
= \mathbb{E}\!\left[
(\mathbf{x}_k-\hat{\mathbf{x}}_{k\mid j})
(\mathbf{x}_k-\hat{\mathbf{x}}_{k\mid j})^T
\mid\mathbf{z}_{1:j}
\right],
\qquad j\in\{k-1,k\}.
```

The diagonal entries of `P` are the state-error variances. Its off-diagonal entries represent correlations between errors in position and velocity.

### 4. Prediction and covariance propagation

The prediction step propagates the previous posterior estimate through the dynamic model:

```math
\hat{\mathbf{x}}_{k\mid k-1}
= \mathbf{F}_k\hat{\mathbf{x}}_{k-1\mid k-1}.
```

The prior covariance is propagated as

```math
\mathbf{P}_{k\mid k-1}
= \mathbf{F}_k\mathbf{P}_{k-1\mid k-1}\mathbf{F}_k^T
+ \mathbf{Q}_k.
```

The first term transports the previous uncertainty through the dynamics. Adding `Q_k` accounts for new uncertainty caused by unmodelled motion.

### 5. Measurement prediction and innovation

The predicted measurement is

```math
\hat{\mathbf{z}}_k
= \mathbf{H}_k\hat{\mathbf{x}}_{k\mid k-1}.
```

The innovation, or measurement residual, is

```math
\mathbf{y}_k
= \mathbf{z}_k-\hat{\mathbf{z}}_k
= \mathbf{z}_k-\mathbf{H}_k\hat{\mathbf{x}}_{k\mid k-1}.
```

Its covariance is

```math
\mathbf{S}_k
= \mathbf{H}_k\mathbf{P}_{k\mid k-1}\mathbf{H}_k^T
+ \mathbf{R}_k.
```

The implementation stores these quantities as `innovation` and `innovation_covariance`.

### 6. Kalman gain and measurement update

The Kalman gain is

```math
\mathbf{K}_k
= \mathbf{P}_{k\mid k-1}\mathbf{H}_k^T\mathbf{S}_k^{-1}.
```

It weights the correction according to the uncertainty in the prediction and the measurement. The posterior state estimate is

```math
\hat{\mathbf{x}}_{k\mid k}
= \hat{\mathbf{x}}_{k\mid k-1}+\mathbf{K}_k\mathbf{y}_k.
```

Although the sensor measures only position, position-velocity correlations in `P` allow the correction to update velocity as well.

The covariance update used by the implementation is

```math
\mathbf{P}_{k\mid k}
= (\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}_{k\mid k-1}.
```

An algebraically equivalent and more numerically robust form is the Joseph update:

```math
\mathbf{P}_{k\mid k}
= (\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)
\mathbf{P}_{k\mid k-1}
(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)^T
+ \mathbf{K}_k\mathbf{R}_k\mathbf{K}_k^T.
```

The complete cycle is therefore: predict the state, propagate its covariance, predict the measurement, calculate the innovation, calculate the Kalman gain, and correct the state and covariance.

## Getting started

### Requirements

- Python 3.8 or later
- Jupyter Notebook or JupyterLab, or VS Code with the Jupyter extension

Install the dependencies with:

```bash
python -m pip install numpy scipy matplotlib jupyter
```

### Run the notebooks

From the repository root:

```bash
jupyter notebook
```

Then open any notebook listed in the contents table.

### Run the Kalman filter implementation

```bash
cd LinearKalmanFilter_Implementation
python assignment1_answer.py
```

The reusable components are in `LinearKalmanFilter_Implementation/kfsims/`.

## Project structure

```text
.
|-- ContinousTimeSimulation.ipynb
|-- DiscreteTimeSimulation.ipynb
|-- LeastSquareEstimation.ipynb
|-- LinearKalmanFilter_Implementation/
|   |-- README.md
|   |-- assignment1_*.py
|   `-- kfsims/
|       |-- __init__.py
|       |-- kfmodels.py
|       |-- kftracker2d.py
|       |-- tracker2d.py
|       `-- vehiclemodel2d.py
|-- .gitignore
|-- LICENSE
`-- README.md
```

## License

This project is distributed under the MIT License. See [`LICENSE`](LICENSE) for details.
