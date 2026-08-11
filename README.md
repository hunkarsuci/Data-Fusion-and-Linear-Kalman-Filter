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

The simulation notebooks use a mass-spring-damper system to introduce continuous-time and discrete-time state-space modeling. The Kalman-filter implementation below focuses on vehicle state estimation and its prediction/measurement cycle.

## Linear Kalman-filter state estimation

The implementation estimates the state of a vehicle moving in a two-dimensional plane. It is a **linear** Kalman filter: both the state-transition model and the measurement model are linear. The filter combines a physics-based prediction with noisy sensor measurements while propagating the uncertainty of both.

### 1. State and system model

The state at sample \(k\) is the position and velocity in the two coordinate directions:

$$
\mathbf{x}_k =
\begin{bmatrix}p_{x,k}\\p_{y,k}\\v_{x,k}\\v_{y,k}\end{bmatrix}
\in \mathbb{R}^{4}
$$

The discrete-time stochastic state model is

$$
\mathbf{x}_k = \mathbf{F}_k\mathbf{x}_{k-1} + \mathbf{w}_k
$$

where \(\mathbf{w}_k\) is zero-mean process noise. For the constant-velocity assumption and sample time \(\Delta t\),

$$
\mathbf{F}_k =
\begin{bmatrix}
1&0&\Delta t&0\\
0&1&0&\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}.
$$

This represents the kinematic equations \(p_{x,k}=p_{x,k-1}+\Delta t\,v_{x,k-1}\), \(p_{y,k}=p_{y,k-1}+\Delta t\,v_{y,k-1}\), with velocity held constant between samples. The process noise accounts for acceleration and other effects not represented by this simplified model.

### 2. State estimate and covariance

At each time step, the filter maintains:

$$
\hat{\mathbf{x}}_k = \mathbb{E}[\mathbf{x}_k\mid\mathbf{z}_{1:k}],
\qquad
\mathbf{P}_k = \mathrm{Cov}(\mathbf{x}_k-\hat{\mathbf{x}}_k)
$$

Here \(\hat{\mathbf{x}}_k\) is the best linear-Gaussian estimate after using measurements through \(k\), and \(\mathbf{P}_k\in\mathbb{R}^{4\times4}\) is its error covariance. The diagonal entries are the variances of \(p_x,p_y,v_x,v_y\); off-diagonal entries describe correlations between state errors.

The process-noise covariance is

$$
\mathbf{Q}_k=\mathrm{Cov}(\mathbf{w}_k).
$$

For a continuous white-acceleration model, a physically derived covariance is

$$
\mathbf{G}=\begin{bmatrix}
\frac{1}{2}\Delta t^2&0\\
0&\frac{1}{2}\Delta t^2\\
\Delta t&0\\
0&\Delta t
\end{bmatrix},
\qquad
\mathbf{Q}_k=\sigma_a^2\mathbf{G}\mathbf{G}^T,
$$

where \(\sigma_a\) is the acceleration standard deviation. The implementation uses a diagonal tuning approximation based on the same \(\Delta t^2/2\) position and \(\Delta t\) velocity scaling. Increasing `accel_std` increases predicted uncertainty and allows the estimate to respond more quickly to maneuvers.

### 3. Measurement model

The sensor provides a noisy measurement of position only:

$$
\mathbf{z}_k = \mathbf{H}_k\mathbf{x}_k + \mathbf{v}_k,
\qquad
\mathbf{z}_k=\begin{bmatrix}z_{x,k}\\z_{y,k}\end{bmatrix}
$$

The measurement matrix selects position from the state:

$$
\mathbf{H}_k=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix}.
$$

The measurement noise \(\mathbf{v}_k\) is assumed zero mean and independent of the process noise, with covariance

$$
\mathbf{R}_k=\mathrm{Cov}(\mathbf{v}_k)
 =\begin{bmatrix}\sigma_m^2&0\\0&\sigma_m^2\end{bmatrix}.
$$

`meas_std` is \(\sigma_m\). A larger \(\mathbf{R}_k\) means less confidence in the sensor; a larger predicted covariance means less confidence in the motion model.

### 4. Prediction and covariance propagation

Before receiving the measurement at time \(k\), the filter propagates the previous posterior estimate forward:

$$
\hat{\mathbf{x}}^-_k=\mathbf{F}_k\hat{\mathbf{x}}_{k-1}
$$

The superscript \((-\)) denotes the prior, or predicted, quantity. The predicted state is the physical model applied to the previous estimate. The covariance propagation follows from the predicted error
\(\mathbf{e}^-_k=\mathbf{x}_k-\hat{\mathbf{x}}^-_k=\mathbf{F}_k\mathbf{e}_{k-1}+\mathbf{w}_k\):

$$
\mathbf{P}^-_k=\mathbf{F}_k\mathbf{P}_{k-1}\mathbf{F}_k^T+\mathbf{Q}_k.
$$

The term \(\mathbf{F}_k\mathbf{P}_{k-1}\mathbf{F}_k^T\) transports existing uncertainty through the dynamics. The added \(\mathbf{Q}_k\) represents new uncertainty introduced by unmodelled acceleration.

### 5. Measurement prediction and innovation

The predicted measurement is

$$
\hat{\mathbf{z}}_k=\mathbf{H}_k\hat{\mathbf{x}}^-_k.
$$

The innovation, also called the measurement residual, is the difference between the actual and predicted measurement:

$$
\boldsymbol{\nu}_k=\mathbf{z}_k-\hat{\mathbf{z}}_k
 =\mathbf{z}_k-\mathbf{H}_k\hat{\mathbf{x}}^-_k.
$$

Its covariance is

$$
\mathbf{S}_k=\mathbf{H}_k\mathbf{P}^-_k\mathbf{H}_k^T+\mathbf{R}_k.
$$

This combines uncertainty in the predicted position with sensor uncertainty. In the implementation, \(\boldsymbol{\nu}_k\) is stored as `innovation` and \(\mathbf{S}_k\) as `innovation_covariance`.

### 6. Measurement update and state estimation

The Kalman gain weights the innovation according to the relative uncertainty of the prediction and measurement:

$$
\mathbf{K}_k=\mathbf{P}^-_k\mathbf{H}_k^T\mathbf{S}_k^{-1}.
$$

The posterior state estimate is obtained by correcting the predicted state:

$$
\hat{\mathbf{x}}_k=\hat{\mathbf{x}}^-_k+\mathbf{K}_k\boldsymbol{\nu}_k.
$$

Because the measurement contains only position, the update also improves velocity through the position-velocity correlations in \(\mathbf{P}^-_k\). This is how the filter estimates velocity without a direct velocity sensor.

The posterior covariance is updated as

$$
\mathbf{P}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}^-_k.
$$

This reduces uncertainty in directions informed by the measurement. For numerical implementations, the equivalent Joseph form is often preferred:

$$
\mathbf{P}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}^-_k(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)^T+\mathbf{K}_k\mathbf{R}_k\mathbf{K}_k^T.
$$

The repository implementation uses the simplified covariance equation above.

The same state-estimation model is documented in [`LinearKalmanFilter_Implementation/README.md`](LinearKalmanFilter_Implementation/README.md).

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
