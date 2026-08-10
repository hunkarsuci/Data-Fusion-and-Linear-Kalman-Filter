# Data Fusion and Linear Kalman Filter

Examples and exercises for modeling dynamic systems, simulating state-space models, and estimating unknown quantities from measurements.

## Contents

| Notebook | Description |
| --- | --- |
| [`ContinousTimeSimulation.ipynb`](ContinousTimeSimulation.ipynb) | Forward Euler simulation of a continuous-time mass-spring-damper system, compared with its exact response. |
| [`DiscreteTimeSimulation.ipynb`](DiscreteTimeSimulation.ipynb) | Exact discretization and simulation of the same mass-spring-damper model. |
| [`LeastSquareEstimation.ipynb`](LeastSquareEstimation.ipynb) | Ordinary least-squares and weighted least-squares estimation using measurement covariance. |

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

## Project structure

```text
.
├── ContinousTimeSimulation.ipynb
├── DiscreteTimeSimulation.ipynb
├── LeastSquareEstimation.ipynb
├── .gitignore
├── LICENSE
└── README.md
```

## License

This project is distributed under the MIT License. See [`LICENSE`](LICENSE) for details.
