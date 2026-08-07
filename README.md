# Data Fusion and Linear Kalman Filter

Simulation and state estimation of dynamic systems using continuous-time and discrete-time models, with an emphasis on linear Kalman filtering and data fusion.

## Project Structure

```
.
├── ContinousTimeSimulation.ipynb   # Continuous-time mass-spring-damper simulation
├── DiscreteTimeSimulation.ipynb    # Discrete-time dynamic system simulation
├── .gitignore
├── LICENSE
└── README.md
```

## Overview

Topics covered in this repository:

- Dynamic system modeling: second-order mass-spring-damper systems
- Continuous-time simulation via numerical integration (Runge-Kutta and related methods)
- Discrete-time simulation using state-space discretization and time-stepping
- Linear Kalman filtering for state estimation in noisy dynamic systems
- Data fusion: combining sensor measurements with model-based predictions

Both the continuous-time and discrete-time dynamic system simulations are included as
completed exercises in their respective notebooks.

### Mass-Spring-Damper System

The governing differential equation:

$$
m\ddot{x}(t) + b\dot{x}(t) + kx(t) = f(t)
$$

where $m$ is mass, $b$ is the damping coefficient, $k$ is spring stiffness, and $f(t)$ is the applied force. The second-order equation is reduced to a first-order state-space representation and solved both analytically and numerically.

## Getting Started

### Prerequisites

- Python 3.8 or later
- Jupyter Notebook, or VS Code with the Jupyter extension

### Dependencies

```bash
pip install numpy scipy matplotlib jupyter
```

### Running the Notebook

Continuous-time simulation:

```bash
jupyter notebook ContinousTimeSimulation.ipynb
```

Discrete-time simulation:

```bash
jupyter notebook DiscreteTimeSimulation.ipynb
```

## License

This project is distributed under the MIT License. See [LICENSE](LICENSE) for details.
