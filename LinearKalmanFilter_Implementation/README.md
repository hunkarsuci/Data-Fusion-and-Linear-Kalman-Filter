# Linear Kalman Filter Implementation

This directory implements a linear Kalman filter for estimating two-dimensional position and velocity from noisy position measurements.

## Models

The state and measurement equations are

```math
\mathbf{x}_k=\mathbf{F}_k\mathbf{x}_{k-1}+\mathbf{w}_k,
\qquad
\mathbf{z}_k=\mathbf{H}_k\mathbf{x}_k+\mathbf{v}_k,
```

where

```math
\mathbf{x}_k=
\begin{bmatrix}
p_{x,k} \\
p_{y,k} \\
v_{x,k} \\
v_{y,k}
\end{bmatrix},
\quad
\mathbf{F}_k=
\begin{bmatrix}
1&0&\Delta t&0 \\
0&1&0&\Delta t \\
0&0&1&0 \\
0&0&0&1
\end{bmatrix},
\quad
\mathbf{H}_k=
\begin{bmatrix}
1&0&0&0 \\
0&1&0&0
\end{bmatrix}.
```

The noises are modeled as

```math
\mathbf{w}_k\sim\mathcal{N}(\mathbf{0},\mathbf{Q}_k),
\qquad
\mathbf{v}_k\sim\mathcal{N}(\mathbf{0},\mathbf{R}_k).
```

The code uses

```math
\mathbf{Q}_{\mathrm{code}}=\sigma_a^2
\mathrm{diag}\!\left(
\frac{\Delta t^2}{2},
\frac{\Delta t^2}{2},
\Delta t,
\Delta t
\right),
\qquad
\mathbf{R}_k=
\begin{bmatrix}
\sigma_m^2&0 \\
0&\sigma_m^2
\end{bmatrix}.
```

`accel_std` supplies `sigma_a`, and `meas_std` supplies `sigma_m`.

## Kalman Filter recursion

### Prediction

```math
\hat{\mathbf{x}}_{k\mid k-1}
=\mathbf{F}_k\hat{\mathbf{x}}_{k-1\mid k-1}
```

```math
\mathbf{P}_{k\mid k-1}
=\mathbf{F}_k\mathbf{P}_{k-1\mid k-1}\mathbf{F}_k^T+\mathbf{Q}_k
```

### Measurement prediction and innovation

```math
\hat{\mathbf{z}}_k=\mathbf{H}_k\hat{\mathbf{x}}_{k\mid k-1}
```

```math
\mathbf{y}_k=\mathbf{z}_k-\hat{\mathbf{z}}_k
```

```math
\mathbf{S}_k
=\mathbf{H}_k\mathbf{P}_{k\mid k-1}\mathbf{H}_k^T+\mathbf{R}_k
```

### Measurement update

```math
\mathbf{K}_k
=\mathbf{P}_{k\mid k-1}\mathbf{H}_k^T\mathbf{S}_k^{-1}
```

```math
\hat{\mathbf{x}}_{k\mid k}
=\hat{\mathbf{x}}_{k\mid k-1}+\mathbf{K}_k\mathbf{y}_k
```

```math
\mathbf{P}_{k\mid k}
=(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}_{k\mid k-1}
```

The implementation stores `y_k` as `innovation` and `S_k` as `innovation_covariance`. See the repository's [main README](../README.md) for the full interpretation of each equation and the standard continuous white-acceleration covariance.

## Implementation files

- `kfsims/kfmodels.py`: state, covariance, innovation, and innovation-covariance accessors.
- `kfsims/kftracker2d.py`: complete two-dimensional linear Kalman filter.
- `kfsims/tracker2d.py`: simulation and plotting utilities.
- `kfsims/vehiclemodel2d.py`: simulated vehicle motion model.
- `assignment1_*.py`: initialization, prediction, and measurement-update examples.

Run an example from this directory:

```bash
python assignment1_answer.py
```
