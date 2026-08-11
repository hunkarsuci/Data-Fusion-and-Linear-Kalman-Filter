# Linear Kalman Filter Implementation

This directory contains a 2D constant-velocity Kalman filter and step-by-step assignment examples.

## State and physics

The estimated state is

$$
\mathbf{x}_k=[p_x,p_y,v_x,v_y]^T
$$

where \(p_x,p_y\) are position and \(v_x,v_y\) are velocity. Assuming constant velocity during a time interval \(\Delta t\):

$$
\mathbf{x}_{k+1}=\mathbf{F}\mathbf{x}_k+\mathbf{w}_k,
\quad
\mathbf{F}=\begin{bmatrix}1&0&\Delta t&0\\0&1&0&\Delta t\\0&0&1&0\\0&0&0&1\end{bmatrix}
$$

The process noise \(\mathbf{w}_k\) models unknown acceleration. Its covariance \(\mathbf{Q}\) is controlled by `accel_std`. Larger values allow the estimate to respond more rapidly to changes in motion.

## Sensor model

The simulated sensor measures position:

$$
\mathbf{z}_k=\mathbf{H}\mathbf{x}_k+\mathbf{v}_k,
\quad
\mathbf{H}=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix}
$$

The measurement-noise covariance is \(\mathbf{R}=\operatorname{diag}(\sigma_m^2,\sigma_m^2)\), where `meas_std` is \(\sigma_m\).

## Algorithm

For every prediction/update cycle:

$$
\hat{\mathbf{x}}^-_k=\mathbf{F}\hat{\mathbf{x}}_{k-1},
\qquad
\mathbf{P}^-_k=\mathbf{F}\mathbf{P}_{k-1}\mathbf{F}^T+\mathbf{Q}
$$

$$
\mathbf{y}_k=\mathbf{z}_k-\mathbf{H}\hat{\mathbf{x}}^-_k,
\quad
\mathbf{S}_k=\mathbf{H}\mathbf{P}^-_k\mathbf{H}^T+\mathbf{R}
$$

$$
\mathbf{K}_k=\mathbf{P}^-_k\mathbf{H}^T\mathbf{S}_k^{-1}
$$

$$
\hat{\mathbf{x}}_k=\hat{\mathbf{x}}^-_k+\mathbf{K}_k\mathbf{y}_k,
\qquad
\mathbf{P}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H})\mathbf{P}^-_k
$$

In code, these operations are implemented by `prediction_step()` and `update_step()` in `kfsims/kftracker2d.py` and demonstrated progressively in the `assignment1_*.py` files.

## Main files

- `kfsims/kfmodels.py`: base filter state and uncertainty accessors.
- `kfsims/kftracker2d.py`: complete 2D filter implementation.
- `kfsims/tracker2d.py`: vehicle simulation and plotting utilities.
- `kfsims/vehiclemodel2d.py`: simulated vehicle motion model.
- `assignment1_initial_conditions.py`: state and covariance initialization.
- `assignment1_prediction.py`: state-transition and covariance prediction.
- `assignment1_update.py`: measurement update with innovation and Kalman gain.
- `*_answer.py`: completed versions of the assignment steps.

Run an example from this directory:

```bash
python assignment1_answer.py
```
