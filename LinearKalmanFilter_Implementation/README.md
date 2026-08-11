# Linear Kalman Filter Implementation

This directory implements a linear Kalman filter for estimating a vehicle's two-dimensional position and velocity from noisy position measurements.

## State-space model

The state is

$$
\mathbf{x}_k=\begin{bmatrix}p_{x,k}\\p_{y,k}\\v_{x,k}\\v_{y,k}\end{bmatrix}.
$$

The constant-velocity process model is

$$
\mathbf{x}_k=\mathbf{F}_k\mathbf{x}_{k-1}+\mathbf{w}_k,
\qquad
\mathbf{F}_k=\begin{bmatrix}
1&0&\Delta t&0\\
0&1&0&\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix},
$$

where \(\mathbf{w}_k\sim\mathcal{N}(0,\mathbf{Q}_k)\) represents unmodelled acceleration and other model error. The standard continuous white-acceleration covariance is

$$
\mathbf{Q}_k=\sigma_a^2\mathbf{G}_k\mathbf{G}_k^T,
\qquad
\mathbf{G}_k=\begin{bmatrix}
\frac{1}{2}\Delta t^2&0\\
0&\frac{1}{2}\Delta t^2\\
\Delta t&0\\
0&\Delta t
\end{bmatrix}.
$$

The implementation uses a diagonal approximation of this process-noise model, controlled by `accel_std`.

## Measurement model

The sensor measures position:

$$
\mathbf{z}_k=\mathbf{H}_k\mathbf{x}_k+\mathbf{v}_k,
\qquad
\mathbf{H}_k=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix},
$$

where \(\mathbf{v}_k\sim\mathcal{N}(0,\mathbf{R}_k)\). With independent position errors,

$$
\mathbf{R}_k=\begin{bmatrix}\sigma_m^2&0\\0&\sigma_m^2\end{bmatrix},
$$

and `meas_std` is \(\sigma_m\).

## State estimate and covariance

The filter maintains the posterior estimate and its error covariance:

$$
\hat{\mathbf{x}}_k=\mathbb{E}[\mathbf{x}_k\mid\mathbf{z}_{1:k}],
\qquad
\mathbf{P}_k=\operatorname{Cov}(\mathbf{x}_k-\hat{\mathbf{x}}_k).
$$

The diagonal of \(\mathbf{P}_k\) contains the variances of position and velocity. Its off-diagonal terms describe correlations, allowing a position measurement to improve the velocity estimate.

## Prediction and covariance propagation

The prediction step propagates the previous posterior state and covariance to the current time before using the new measurement:

$$
\hat{\mathbf{x}}^-_k=\mathbf{F}_k\hat{\mathbf{x}}_{k-1},
$$

$$
\mathbf{P}^-_k=\mathbf{F}_k\mathbf{P}_{k-1}\mathbf{F}_k^T+\mathbf{Q}_k.
$$

The first term transports the old uncertainty through the motion model. The second term adds uncertainty caused by acceleration and model mismatch.

## Measurement prediction and update

First, the predicted measurement and innovation are calculated:

$$
\hat{\mathbf{z}}_k=\mathbf{H}_k\hat{\mathbf{x}}^-_k,
\qquad
\boldsymbol{\nu}_k=\mathbf{z}_k-\hat{\mathbf{z}}_k.
$$

The innovation covariance is

$$
\mathbf{S}_k=\mathbf{H}_k\mathbf{P}^-_k\mathbf{H}_k^T+\mathbf{R}_k.
$$

The Kalman gain determines how strongly the innovation changes the prediction:

$$
\mathbf{K}_k=\mathbf{P}^-_k\mathbf{H}_k^T\mathbf{S}_k^{-1}.
$$

The corrected state estimate and covariance are

$$
\hat{\mathbf{x}}_k=\hat{\mathbf{x}}^-_k+\mathbf{K}_k\boldsymbol{\nu}_k,
$$

$$
\mathbf{P}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}^-_k.
$$

The implementation records \(\boldsymbol{\nu}_k\) as `innovation` and \(\mathbf{S}_k\) as `innovation_covariance`. The update is performed in `update_step()` after `prediction_step()`.

## Implementation files

- `kfsims/kfmodels.py`: state, covariance, innovation, and innovation-covariance accessors.
- `kfsims/kftracker2d.py`: complete 2D linear Kalman-filter model.
- `kfsims/tracker2d.py`: vehicle simulation and plotting utilities.
- `kfsims/vehiclemodel2d.py`: simulated vehicle motion model.
- `assignment1_*.py`: examples showing initialization, prediction, and measurement-update stages.

Run an example from this directory:

```bash
python assignment1_answer.py
```
