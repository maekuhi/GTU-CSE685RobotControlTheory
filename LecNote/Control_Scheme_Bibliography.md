# Control Scheme Bibliography

This file is a reading map for the control schemes used throughout the lecture notes. The lecture notes remain the main narrative; the books below anchor the standard forms, assumptions, and stability arguments.

## Main Course Textbooks

**[R1]** Frank L. Lewis, Darren M. Dawson, and Chaouki T. Abdallah, *Robot Manipulator Control: Theory and Practice*, Second Edition, Revised and Expanded, Marcel Dekker, 2004.

Best used for:

- manipulator dynamics in the `M(q)`, `V_m(q,qdot)`, `G(q)` notation,
- feedback linearization and computed-torque control,
- PD, PID, and gravity-compensation ideas,
- filtered tracking error and Lyapunov robot-control design,
- robust control with bounded uncertainty,
- adaptive computed-torque control,
- passivity-based and composite adaptive control.

**[R2]** Carlos Canudas de Wit, Bruno Siciliano, and Georges Bastin, editors, *Theory of Robot Control*, Springer, 1996.

Best used for:

- structural properties of robot dynamics,
- joint-space inverse dynamics and passivity-based controllers,
- task-space kinematic and dynamic control,
- redundancy, pseudoinverse, and singularity handling,
- elastic-joint modeling and two-time-scale control,
- wheeled mobile robot kinematics, mobility, steerability, manoeuvrability, and nonholonomic structure.

## Control Scheme Map

| Control scheme | Main idea | Best textbook support |
|---|---|---|
| Feedback linearization | Use the model to cancel nonlinear robot dynamics and expose a double-integrator form. | [R1], [R2] |
| Computed-torque control | Put feedback linearization into the standard manipulator equation and design a linear outer-loop error equation. | [R1] |
| PD plus gravity | Keep the stabilizing energy structure without cancelling all velocity-dependent dynamics. | [R1], [R2] |
| Filtered-error control | Replace second-order tracking error by `r = edot + Lambda e`, then design using an energy Lyapunov function. | [R1] |
| Robust control | Treat model mismatch as bounded uncertainty and add a term large enough to dominate it in `Vdot`. | [R1] |
| Variable-structure / sliding control | Use sign-based action on the filtered error to force motion toward a sliding surface. | [R1] |
| RISE | Integrate sign-error information to retain robustness with a smoother applied input. | Lecture notes primary; [R1] supports the robust Lyapunov structure |
| Adaptive computed torque | Use linear parameterization and update parameter estimates so Lyapunov cross terms cancel. | [R1] |
| Passivity-based adaptive control | Use the robot energy/passivity structure to design tracking and adaptation together. | [R1], [R2] |
| Composite adaptation | Add prediction error to tracking error so parameter learning uses more information. | [R1] |
| Task-space control | Move the objective from joint variables to end-effector coordinates using the Jacobian. | [R2] |
| Direct task-space dynamics | Express inertia, velocity, and gravity effects in Cartesian/task coordinates. | [R2] |
| Elastic-joint control | Separate link and motor coordinates, then handle fast elastic dynamics and slow link motion. | [R2] |
| Mobile robot posture control | Transform posture errors to expose a stabilizable nonholonomic error structure. | [R2]; Lecture notes primary for the unified derivation |

## How To Read The Scheme Progression

The course control schemes form a ladder:

1. **Exact model known:** feedback linearization and computed torque.
2. **Exact model not fully needed:** PD plus gravity and passivity-based arguments.
3. **Model uncertain but bounded:** robust and variable-structure control.
4. **Model uncertain but parametrically structured:** adaptive control.
5. **Learning needs more information:** persistency of excitation and composite adaptation.
6. **Task specified outside joint space:** task-space control.
7. **Robot mechanics change the coordinates:** elastic joints and mobile robots.

The unifying pattern is always the same: choose an error, expose its dynamics, inject damping or cancellation, then prove the result with a Lyapunov or passivity argument.
