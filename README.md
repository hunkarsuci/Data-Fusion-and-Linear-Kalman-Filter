# Data Fusion and Linear Kalman Filter

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Jupyter](https://img.shields.io/badge/Jupyter-notebooks-F37626?logo=jupyter&logoColor=white)](https://jupyter.org/)
[![NumPy](https://img.shields.io/badge/NumPy-supported-013243?logo=numpy&logoColor=white)](https://numpy.org/)
[![SciPy](https://img.shields.io/badge/SciPy-supported-8CAAE6?logo=scipy&logoColor=white)](https://scipy.org/)
[![GitHub last commit](https://img.shields.io/github/last-commit/hunkarsuci/Data-Fusion-and-Linear-Kalman-Filter)](https://github.com/hunkarsuci/Data-Fusion-and-Linear-Kalman-Filter/commits/master)

Examples and exercises for modeling dynamic systems, simulating state-space models, and estimating unknown quantities from measurements.

## Contents

| Notebook | Description |
| --- | --- |
| [`ContinousTimeSimulation.ipynb`](ContinousTimeSimulation.ipynb) | Forward Euler simulation of a continuous-time mass-spring-damper system, compared with its exact response. |
| [`DiscreteTimeSimulation.ipynb`](DiscreteTimeSimulation.ipynb) | Exact discretization and simulation of the same mass-spring-damper model. |
| [`LeastSquareEstimation.ipynb`](LeastSquareEstimation.ipynb) | Ordinary least-squares and weighted least-squares estimation using measurement covariance. |
| [`LinearKalmanFilter_Implementation/`](LinearKalmanFilter_Implementation/) | Python implementation of a linear Kalman filter for 2D vehicle tracking, including prediction, measurement update, and assignment examples. |

## Topics

- First- and second-order dynamic system modeling
- Continuous-time and discrete-time state-space representations
- Forward Euler integration and exact matrix-exponential discretization
- Linear state estimation and data fusion concepts
- Ordinary least squares (LSE) and weighted least squares (WLS)
- Measurement uncertainty and covariance-based weighting

## Model example

The simulation notebooks use a mass-spring-damper system:

$$
m\ddot{x}(t) + b\dot{x}(t) + kx(t) = f(t)
$$

The second-order equation is expressed as a first-order state-space model with position and velocity as the state variables. The continuous-time model is then simulated numerically and converted to an equivalent discrete-time model.

The estimation notebook uses the linear measurement model

$$
\mathbf{y} = \mathbf{H}\mathbf{x} + \mathbf{v}
$$

For weighted least squares, the measurement covariance matrix \(\mathbf{R}\) determines the relative influence of each measurement:

$$
\hat{\mathbf{x}} = (\mathbf{H}^T\mathbf{R}^{-1}\mathbf{H})^{-1}\mathbf{H}^T\mathbf{R}^{-1}\mathbf{y}
$$

## Extended Kalman-filter mathematics and physics

The implementation estimates the motion of a vehicle moving in a two-dimensional plane. It assumes a constant-velocity model over one sample interval, while allowing unknown acceleration to perturb that model.

### Physical state model

The state vector contains position and velocity in the horizontal and vertical directions:

$$
\mathbf{x}_k = \begin{bmatrix}p_x & p_y & v_x & v_y\end{bmatrix}^T
$$

For a time step \(\Delta t\), the kinematic equations are

$$
p_{x,k+1}=p_{x,k}+\Delta t\,v_{x,k}, \qquad
p_{y,k+1}=p_{y,k}+\Delta t\,v_{y,k}
$$

$$
v_{x,k+1}=v_{x,k}, \qquad v_{y,k+1}=v_{y,k}
$$

In matrix form, the deterministic constant-velocity model is

$$
\mathbf{x}_{k+1}=\mathbf{F}\mathbf{x}_k+\mathbf{w}_k, \qquad
\mathbf{F}=\begin{bmatrix}
1&0&\Delta t&0\\
0&1&0&\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}
$$

The process-noise vector \(\mathbf{w}_k\) represents unmodelled acceleration, such as changes in throttle, steering, friction, or vehicle motion. It is assumed to have zero mean and covariance \(\mathbf{Q}\). A continuous white-acceleration model uses

$$
\mathbf{G}=\begin{bmatrix}\frac{1}{2}\Delta t^2&0\\0&\frac{1}{2}\Delta t^2\\\Delta t&0\\0&\Delta t\end{bmatrix}, \qquad
\mathbf{Q}=\sigma_a^2\mathbf{G}\mathbf{G}^T
$$

where \(\sigma_a\) is the acceleration standard deviation. The implementation uses the same position and velocity scaling in a diagonal approximation of \(\mathbf{Q}\); increasing `accel_std` makes the filter adapt more quickly to maneuvers.

### Measurement model

The tracker receives noisy position measurements, not direct velocity measurements:

$$
\mathbf{z}_k=\mathbf{H}\mathbf{x}_k+\mathbf{v}_k, \qquad
\mathbf{H}=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix}
$$

Here \(\mathbf{v}_k\) is zero-mean measurement noise with covariance \(\mathbf{R}\). For independent position sensors with standard deviation \(\sigma_m\), the code sets \(\mathbf{R}=\operatorname{diag}(\sigma_m^2,\sigma_m^2)\). Larger measurement uncertainty causes the filter to trust its motion prediction more; smaller uncertainty causes it to follow measurements more closely.

### Prediction and correction cycle

The filter stores an estimated state \(\hat{\mathbf{x}}\) and covariance \(\mathbf{P}\), where \(\mathbf{P}\) describes uncertainty in position, velocity, and their correlations.

1. **Prediction:**

   $$
   \hat{\mathbf{x}}^-_k=\mathbf{F}\hat{\mathbf{x}}_{k-1}, \qquad
   \mathbf{P}^-_k=\mathbf{F}\mathbf{P}_{k-1}\mathbf{F}^T+\mathbf{Q}
   $$

2. **Innovation:** compare the sensor reading with the predicted measurement:

   $$
   \mathbf{y}_k=\mathbf{z}_k-\mathbf{H}\hat{\mathbf{x}}^-_k, \qquad
   \mathbf{S}_k=\mathbf{H}\mathbf{P}^-_k\mathbf{H}^T+\mathbf{R}
   $$

3. **Measurement update:** compute the Kalman gain and combine prediction with measurement:

   $$
   \mathbf{K}_k=\mathbf{P}^-_k\mathbf{H}^T\mathbf{S}_k^{-1}
   $$

   $$
   \hat{\mathbf{x}}_k=\hat{\mathbf{x}}^-_k+\mathbf{K}_k\mathbf{y}_k, \qquad
   \mathbf{P}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H})\mathbf{P}^-_k
   $$

The Kalman gain is the balance between model uncertainty and sensor uncertainty. The implementation records \(\mathbf{y}_k\) as `innovation` and \(\mathbf{S}_k\) as `innovation_covariance`, which are useful for diagnosing unexpectedly large or inconsistent measurements.

The equations and parameter meanings are also documented in [`LinearKalmanFilter_Implementation/README.md`](LinearKalmanFilter_Implementation/README.md).

## Getting started

### Requirements

- Python 3.8 or later
- Jupyter Notebook or JupyterLab, or VS Code with the Jupyter extension

Install the Python dependencies with:

```bash
python -m pip install numpy scipy matplotlib jupyter
```

### Run the notebooks

From the repository root, launch Jupyter:

```bash
jupyter notebook
```

Then open any notebook listed above. Individual notebooks can also be launched directly:

```bash
jupyter notebook ContinousTimeSimulation.ipynb
jupyter notebook DiscreteTimeSimulation.ipynb
jupyter notebook LeastSquareEstimation.ipynb
```

Run the notebook cells from top to bottom so that imports, model parameters, calculations, and plots are initialized in order.

### Run the Kalman-filter implementation

The implementation is contained in `LinearKalmanFilter_Implementation/`. The reusable filter and tracking components are in the `kfsims/` package:

- `kfmodels.py` contains the base Kalman-filter interface and state accessors.
- `kftracker2d.py` implements the 2D constant-velocity Kalman-filter model.
- `tracker2d.py` and `vehiclemodel2d.py` provide tracking and vehicle-model utilities.
- The `assignment1_*.py` files demonstrate filter initialization, prediction, and update steps.

From the implementation directory, run an example with:

```bash
cd LinearKalmanFilter_Implementation
python assignment1_answer.py
```

## Project structure

```text
.
├── ContinousTimeSimulation.ipynb
├── DiscreteTimeSimulation.ipynb
├── LeastSquareEstimation.ipynb
├── LinearKalmanFilter_Implementation/
│   ├── README.md
│   ├── assignment1_*.py
│   └── kfsims/
│       ├── kfmodels.py
│       ├── kftracker2d.py
│       ├── tracker2d.py
│       └── vehiclemodel2d.py
├── .gitignore
├── LICENSE
└── README.md
```

## License

This project is distributed under the MIT License. See [`LICENSE`](LICENSE) for details.
