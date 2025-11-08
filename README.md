# PID-Controller-Simulation
A complete simulation and validation of a PID controller for a 2nd-Order DC Motor using Python, MATLAB, and Simulink.

The core objective is to correct the system's large steady-state error and poor response by designing a controller that is fast, stable, and accurate.

The solution is developed in **Python** and the validated using **MATLAB** and **Simulink** to demonstrate the design and compare results.

## System Model & Problem
The system to be controlled (the "plant") is a 2nd-order DC motor model with the following transfer function $G(s)$:

$$G(s) = \frac{5}{s^2 + 11s + 10}$$

When a step input (a target speed of 1.0) is applied to the uncontrolled system, it fails to reach the setpoint, resulting in a large 50% steady-state error. This is the core problem to be solved.

## Python Implementation and Tuning
The initial controller design and tuning were performed using the python-control library. The script systematically tunes each PID component to build an intuition for its effect.

* P-Tuning: Shows that $K_p$ (Proportional) gain speeds up the response but causes overshoot and cannot fix the steady-state error.
* PI-Tuning: Shows that $K_i$ (Integral) gain successfully eliminates the steady-state error but at the cost of significant overshoot and oscillation.
* PD-Tuning: Shows that $K_d$ (Derivative) gain acts as a "damper" to reduce overshoot but cannot fix the steady-state error on its own.
* PID-Tuning: By combining all three terms, a "manual tune" is developed that balances all objectives: zero steady-state error, fast response, and low overshoot.

## MATLAB and Simulink Validation
To verify the Python results and demonstrate proficiency in industry-standard tools, the entire system was re-implemented in MATLAB and Simulink.

### A. MATLAB Script Validation
A MATLAB script (matlab_simulation.m) was written to define the same plant and PID gains. The script generates a final comparison plot, which perfectly matches the results from the Python simulation, validating the model and the controller design.

### B. Simulink Model Validation
A graphical model of the closed-loop system was built in Simulink (pid_simulink.slx) to simulate the system dynamics, which is a common workflow in aerospace and automotive industries.  
The model consists of a Step input, a PID Controller block, and a Transfer Fcn block for the plant.  
The Scope block output from the Simulink simulation confirms the identical response, further validating the "manual tune" gains.

## Performance Analysis and Results
The performance metrics for the final controllers were calculated and compared. The "Manual Tune" was found to have a similar overshoot to the Ziegler-Nichols tune but with a more stable and less aggressive response.

| Controller | Kp | Ki | Kd | Overshoot (OS) | Settling Time (Ts) | Steady-State Error (Ess) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Uncontrolled** | 0.0 | 0.0 | 0.0 | 0.0% | N/A | 0.504 |
| **Ziegler-Nichols** | 14.5 | 48.6 | 1.1 | 18.6% | 1.018s | 0.000 |
| **Manual Tune** | 10.0 | 40.0 | 1.5 | 18.9% | `1.XXXs` | 0.000 |

## Key Concepts Demonstrated

* System Modeling: Representing a physical DC motor as a Transfer Function.
* Control Theory: Implementing and tuning P, PI, PD, and PID controllers.
* Step Response Analysis: Quantifying performance using Rise Time, Settling Time, Overshoot, and Steady-State Error.
* Tuning Trade-offs: Demonstrating the intuitive "give-and-take" relationship between $K_p$ (speed), $K_i$ (accuracy), and $K_d$ (stability).
