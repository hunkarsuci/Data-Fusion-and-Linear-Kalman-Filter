# Data Fusion & Linear Kalman Filter

A project exploring data fusion techniques and linear Kalman filtering through dynamic system simulation.

---

## 📁 Project Structure

```
.
├── ContinousTimeSimulation.ipynb   # Mass-spring-damper system simulation (continuous & discrete time)
├── .gitignore
├── LICENSE
└── README.md
```

---

## 📖 Overview

This repository contains Jupyter notebooks covering:

- **Dynamic system modeling** — second-order mass-spring-damper systems
- **Continuous-time simulation** — solving ODEs via numerical integration (e.g., Runge-Kutta)
- **Discrete-time simulation** — state-space discretization and time-stepping
- **Linear Kalman filtering** — state estimation for noisy dynamic systems
- **Data fusion** — combining sensor measurements with model predictions

### Mass–Spring–Damper System

The governing differential equation is:

$$
m\ddot{x}(t) + b\dot{x}(t) + kx(t) = f(t)
$$

Converted to state-space form for simulation and filtering.

---

## 🚀 Getting Started

### Prerequisites

- **Python** 3.8+
- **VS Code** with the Jupyter extension, or Jupyter Notebook

### Installation

```bash
pip install numpy scipy matplotlib jupyter
```

### Usage

Open the notebook in VS Code or Jupyter:

```bash
jupyter notebook ContinousTimeSimulation.ipynb
```

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
