# Practical Optimization Examples


````markdown

A collection of mini-projects and examples demonstrating mathematical optimization and control techniques. This repository is divided into two distinct sections: **Python** (using the CVXPY library) and **MATLAB** (using the CasADi framework).

## 📂 Repository Structure

The repository is organized into two folders based on the language/library used:

```text
practical-optimization-examples/
├── python-cvxpy/          # Jupyter Notebooks using CVXPY
│   ├── Linear Programming.ipynb
│   ├── Mixed-Integer Programming.ipynb
│   ├── Nonlinear Programming.ipynb
│   ├── Optimal Control.ipynb
│   └── Quadratic Programming.ipynb
│
└── matlab-casadi/         # MATLAB Scripts using CasADi
    ├── E1.m
    ├── E2.m
    ├── E3.m
    ├── E4.m
    ├── E5.m
    └── E6.m
````

-----

## 🐍 Python Examples (CVXPY)

Located in the `python-cvxpy/` folder. These notebooks demonstrate various optimization formulations.

| Notebook File | Topic | Description |
| :--- | :--- | :--- |
| **Linear Programming.ipynb** | Linear Programming (LP) | Profit maximization with resource constraints (Production Planning). |
| **Mixed-Integer Programming.ipynb** | Mixed-Integer (MIP) | Supply chain and vehicle routing logistics (solving for discrete choices). |
| **Nonlinear Programming.ipynb** | Geometric Programming (GP) | Geometric programming to maximize box volume under physical constraints. |
| **Quadratic Programming.ipynb** | Quadratic Programming (QP) | Solving standard form QP problems with affine constraints. |
| **Optimal Control.ipynb** | Convex Control | Trajectory optimization for a dynamic system (e.g., rocket thrusters). |

### Requirements (Python)

To run the notebooks, install the necessary libraries:

```bash
pip install cvxpy numpy matplotlib jupyter
```

-----

## 🔢 MATLAB Examples (CasADi)

Located in the `matlab-casadi/` folder. These scripts focus on nonlinear programming and model predictive control.

| Script File | Topic | Description |
| :--- | :--- | :--- |
| **E1.m** | Unconstrained Optimization | Basic scalar minimization of a quadratic function. |
| **E2.m** | Non-Convex Optimization | Minimization of a multimodal function `sin(x)` to demonstrate local minima. |
| **E3.m** | Parameter Estimation | Least Squares regression to fit a line `y=mx+c` to a dataset. |
| **E4.m** | Quadratic Programming | Optimization with linear inequality and equality constraints. |
| **E5.m** | Nonlinear Programming | Maximizing geometric volume (similar to the Python example) using CasADi. |
| **E6.m** | Model Predictive Control | **Robot Trajectory Optimization:** Controls a differential drive robot to track a reference posture. Includes real-time animation. |

### Requirements (MATLAB)

1.  A recent version of **MATLAB**.
2.  **CasADi Library**:
      * Download the binaries from [CasADi Downloads](https://web.casadi.org/get/).
      * Unzip the folder.
      * Add the folder to your MATLAB Path using:
        ```matlab
        addpath('/path/to/casadi')
        ```

-----

## 🚀 Getting Started

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/your-username/practical-optimization-examples.git
    ```

2.  **Navigate to the desired example:**

    ```bash
    cd python-cvxpy
    # OR
    cd matlab-casadi
    ```

3.  Run the Jupyter Notebooks or execute the MATLAB scripts to see the solvers in action.

<!-- end list -->

```
```
