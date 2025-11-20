
# Practical Optimization Examples



A collection of mini-projects and examples demonstrating mathematical optimization and control techniques. This repository is divided into two distinct sections: **Python** (using the CVXPY library) and **MATLAB** (using the CasADi framework).## 📂 Repository Structure



The repository is organized into two folders based on the language/library used:```text

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

🐍 Python Examples (CVXPY)

Located in the python-cvxpy/ folder. These notebooks demonstrate various optimization formulations.

Notebook FileTopicDescriptionLinear Programming.ipynbLinear Programming (LP)Profit maximization with resource constraints (Production Planning).Mixed-Integer Programming.ipynbMixed-Integer (MIP)Supply chain and vehicle routing logistics (solving for discrete choices).Nonlinear Programming.ipynbGeometric Programming (GP)Geometric programming to maximize box volume under physical constraints.Quadratic Programming.ipynbQuadratic Programming (QP)Solving standard form QP problems with affine constraints.Optimal Control.ipynbConvex ControlTrajectory optimization for a dynamic system (e.g., rocket thrusters).Requirements (Python)

To run the notebooks, install the necessary libraries:

Bash



pip install cvxpy numpy matplotlib jupyter

🔢 MATLAB Examples (CasADi)

Located in the matlab-casadi/ folder. These scripts focus on nonlinear programming and model predictive control.

Script FileTopicDescriptionE1.mUnconstrained OptimizationBasic scalar minimization of a quadratic function.E2.mNon-Convex OptimizationMinimization of a multimodal function ($\sin(x)$) to demonstrate local minima.E3.mParameter EstimationLeast Squares regression to fit a line ($y=mx+c$) to a dataset.E4.mQuadratic ProgrammingOptimization with linear inequality and equality constraints.E5.mNonlinear ProgrammingMaximizing geometric volume (similar to the Python example) using CasADi.E6.mModel Predictive ControlRobot Trajectory Optimization: Controls a differential drive robot to track a reference posture. Includes real-time animation.Requirements (MATLAB)

A recent version of MATLAB.

CasADi Library:

Download the binaries from CasADi Downloads.

Unzip the folder.

Add the folder to your MATLAB Path using addpath('/path/to/casadi').

🚀 Getting Started

Clone the repository:

Bash



git clone [https://github.com/your-username/practical-optimization-examples.git](https://github.com/your-username/practical-optimization-examples.git)

Navigate to the desired example:

Bash



cd python-cvxpy# ORcd matlab-casadi

Run the Jupyter Notebooks or execute the MATLAB scripts to see the solvers in action.



***



### Your Next Step

Do you have the folders created on your computer yet? If not, I can give you the quick command line instructions to create the folders and move your files into them so they match the README structure. Would you like that?
