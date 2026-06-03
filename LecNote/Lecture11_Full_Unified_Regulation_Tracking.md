# Lecture 11 - Unified Solution to Regulation and Tracking Problems

Original handwritten source: `Lex/lecture11.pdf`

Reference check:

- The table of contents marks this lecture as `/?`.
- This file follows the handwritten lecture notes and checks formulas against standard transformed-error and mechanical-system control logic.

## Big Picture

Regulation and tracking look different on the surface: one aims for a fixed point, the other follows a moving target. This lecture shows that both can be treated as stabilization problems after the right error variables are chosen.

That is the quiet power of coordinate design. If the error system is built intelligently, tracking becomes regulation of the error, and the same Lyapunov machinery can do the heavy lifting.

## Learning Path

1. Define regulation and tracking objectives.
2. Transform physical errors into useful coordinates.
3. Derive open-loop error dynamics.
4. Add auxiliary variables for layered control design.
5. Use Lyapunov functions to prove convergence.

```mermaid
flowchart LR
    TASK["regulation or tracking task"] --> ERR["define transformed error"]
    ERR --> DYN["derive error dynamics"]
    DYN --> AUX["introduce auxiliary variables"]
    AUX --> CTRL["choose stabilizing control"]
    CTRL --> LYAP["Lyapunov proof"]
    LYAP --> CONV["error convergence"]
```

## Textbook Guide

This lecture is mostly driven by the handwritten derivation. Ref. [R2] supports the mobile-robot modeling background, nonholonomic structure, and transformed posture-control viewpoint. Earlier robot-control Lyapunov patterns from [R1] explain why the same regulation/tracking proof style keeps reappearing. See [Control Scheme Bibliography](Control_Scheme_Bibliography.md).

## Main Topic

The lecture presents a unified way to treat regulation and tracking problems, especially for robotic or mechanical systems where the error dynamics can be transformed into a useful form.

Regulation:

```math
x(t)\to x_d
```

where `x_d` is constant.

Tracking:

```math
x(t)\to x_d(t)
```

where `x_d(t)` is time varying.

The goal is to define error coordinates that cover both cases.

## Mobile Robot Reminder

The lecture begins from the unicycle/mobile-robot kinematics:

```math
\begin{bmatrix}
\dot{x}_c\\
\dot{y}_c\\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta & 0\\
\sin\theta & 0\\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v_l\\
v_\omega
\end{bmatrix}
```

where:

- `v_l` is linear velocity,
- `v_\omega` is angular velocity,
- `(x_c,y_c,\theta)` is the Cartesian posture of the robot.

For a reference trajectory:

```math
(x_{rc},y_{rc},\theta_r)
```

define raw posture errors:

```math
\tilde{x}=x_c-x_{rc}
```

```math
\tilde{y}=y_c-y_{rc}
```

```math
\tilde{\theta}=\theta-\theta_r
```

These raw errors are not the best coordinates for a global control design. The lecture therefore introduces a globally invertible transformation from:

```math
(\tilde{x},\tilde{y},\tilde{\theta})
```

to:

```math
(w_1,z_1,z_2)
```

The exact matrix form in the notes is chosen so that:

```math
(w_1,z_1,z_2)\to 0
\quad\Longrightarrow\quad
(\tilde{x},\tilde{y},\tilde{\theta})\to 0
```

and bounded transformed errors imply bounded raw posture errors. In other words, the transformation is not decoration; it is the reason the unified design can attack both regulation and tracking with the same stabilization logic.

The transformed variables satisfy comparison properties of the form:

```math
\|(w_1,z_1,z_2)\|\le \gamma_0 e^{-\lambda t}
```

which imply exponential decay of the physical posture error for positive constants.

## Open-Loop Transformed Mobile-Robot Error

After differentiating the transformed variables and substituting the robot kinematics, the notes isolate the subsystem:

```math
z=
\begin{bmatrix}
z_1\\
z_2
\end{bmatrix}
```

with dynamics of the form:

```math
\dot{z}=T^{-1}V-d_r
```

where:

```math
V=
\begin{bmatrix}
v_l\\
v_\omega
\end{bmatrix}
```

is the physical velocity input and `d_r` is a feedforward term built from the reference velocities and the transformed error coordinates. The handwritten derivation shows this explicitly by first obtaining equations like:

```math
\dot{z}_1=v_\omega-v_{r\omega}
```

and:

```math
\dot{z}_2
=
v_l(-\sin\theta\,\tilde{x}+\cos\theta\,\tilde{y})
+d_{\mathrm{ref}}
```

where `d_{\mathrm{ref}}` collects the reference-dependent terms.

The important structural step is:

```math
\dot{z}=T^{-1}V-d_r
```

so define a new input:

```math
u=T^{-1}V-d_r
```

Then:

```math
\dot{z}=u
```

and the actual robot velocity command can be recovered as:

```math
V=T(u+d_r)
```

This is the first half of the unification: the difficult posture-tracking problem is transformed into a simpler input-output system in `z`.

## The `w_1` Dynamics and the Nonholonomic Integrator

The remaining transformed coordinate has dynamics of the form:

```math
\dot{w}_1=u^TJ^Tz+f
```

where:

```math
J=
\begin{bmatrix}
0 & -1\\
1 & 0
\end{bmatrix}
```

is the constant skew-symmetric matrix, and `f` is a known feedforward/nonlinear term depending on reference motion and transformed errors.

The matrix `J` satisfies:

```math
J^T=-J
```

```math
J^TJ=I
```

```math
\xi^TJ\xi=0,\qquad \forall \xi\in\mathbb{R}^2
```

The open-loop transformed system is therefore:

```math
\dot{w}_1=u^TJ^Tz+f
```

```math
\dot{z}=u
```

This is similar to Brockett's nonholonomic integrator. That connection matters because it explains why restricted-mobility mobile robots are controllable but cannot be globally stabilized by an ordinary smooth time-invariant static feedback. The controller must use a more careful construction.

```mermaid
flowchart LR
    RAW["raw posture error"] --> TR["global transformation"]
    TR --> WZ["w1, z1, z2"]
    WZ --> OPEN["w1dot = u^T J^T z + f; zdot = u"]
    OPEN --> NHI["nonholonomic-integrator structure"]
    NHI --> CTRL["auxiliary trajectory/control design"]
```

## Mobile-Robot Control Development

The control objective is:

```math
(w_1,z_1,z_2)\to 0
```

The lecture defines an auxiliary desired signal:

```math
\tilde{z}=z_d-z
```

where:

```math
z_d\in\mathbb{R}^2
```

is an auxiliary error trajectory designed by the controller. The input is chosen as:

```math
u=u_a-K_z\tilde{z}
```

or equivalently as a stabilizing term plus an auxiliary command, depending on whether the notes write the error as `z_d-z` or `z-z_d`. The point is that the `z` subsystem is made to follow a designed `z_d`, while `z_d` is chosen to make the `w_1` dynamics decay.

The auxiliary command has the form:

```math
u_a=
\left(
\frac{k_1w_1+f}{\delta_d}
\right)Jz_d+\Omega_1z_d
```

where `\delta_d(t)` is a positive exponentially decaying signal:

```math
\delta_d=\delta_0e^{-\alpha t}
```

and `\Omega_1` is chosen to compensate terms that arise from differentiating `z_d`. The lecture notes choose `z_d` from a nonlinear oscillator-like equation so that:

```math
z_d^Tz_d=\delta_d^2
```

which means:

```math
\|z_d\|=|\delta_d|
```

Because `\delta_d` decays exponentially, the auxiliary signal also shrinks exponentially. The gain choices are then selected so that the actual `z` follows `z_d` while `w_1` is driven down.

The logic is:

1. Make `z` track a carefully designed shrinking auxiliary signal.
2. Use the skew-symmetric coupling `u^TJ^Tz` to affect `w_1`.
3. Let the auxiliary signal shrink to zero.
4. Conclude that both `z` and `w_1` converge to zero.

This avoids the impossible demand of stabilizing the nonholonomic structure with a naive smooth static law.

## Mobile-Robot Lyapunov Sketch

A typical Lyapunov function for the transformed mobile problem is:

```math
V=\frac{1}{2}w_1^2+\frac{1}{2}\tilde{z}^T\tilde{z}
```

or a close variant including the auxiliary signal energy. Differentiating gives:

```math
\dot{V}
=
w_1\dot{w}_1+\tilde{z}^T\dot{\tilde{z}}
```

After substituting the selected `u`, `u_a`, and `z_d` dynamics, the cross terms involving `J` cancel or become bounded terms dominated by gains, using:

```math
\xi^TJ\xi=0
```

The final inequality has the exam-relevant shape:

```math
\dot{V}\le -c_1w_1^2-c_2\|\tilde{z}\|^2+d(t)
```

where `d(t)` denotes exponentially decaying terms.

Since the extra terms decay exponentially through `\delta_d`, the transformed errors converge. Then the invertibility of the coordinate transformation gives convergence of the physical posture errors.

## Error Coordinates

For posture-like systems, define errors in a body or transformed frame.

The lecture notes show transformations similar to:

```math
e=T(\theta)(x_d-x)
```

where `T(\theta)` is a rotation or coordinate transformation matrix.

The reason for transforming the error is that the resulting dynamics become easier to control.

## Open-Loop Error Dynamics

The notes derive the open-loop error system before choosing control:

```math
\dot{e}=f_e(e,t)+g_e(e,t)u
```

The controller is then designed to stabilize this transformed error system.

## Control Design Pattern

The general pattern is:

1. Define error variables.
2. Differentiate the error variables.
3. Substitute the system dynamics.
4. Choose control inputs to cancel or dominate unwanted terms.
5. Construct a Lyapunov function.
6. Prove convergence or boundedness.

This is the same pattern used earlier in computed torque, robust control, and adaptive control.

## Exponential Desired Decay

Some parts of the lecture choose auxiliary functions that decay exponentially:

```math
d(t)=d_0e^{-\alpha t}
```

or similar. These functions help transition between regulation and tracking behavior.

If an auxiliary signal decays to zero and the error dynamics are stable, then the main tracking or regulation error also converges.

## Dynamic Surface / Auxiliary Variable Idea

The notes introduce auxiliary variables that convert a difficult problem into a sequence of easier stabilizing steps.

For example, define:

```math
z_1=e
```

and then:

```math
z_2=\dot{e}+\Lambda e
```

This is analogous to the filtered tracking error:

```math
r=\dot{e}+\Lambda e
```

The controller is designed to stabilize `z_2`, which then stabilizes `z_1`.

## Mechanical-System Dynamics

The lecture returns to the manipulator-like dynamic model:

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=\tau
```

or a transformed version:

```math
\bar{M}(q)\ddot{\eta}+\bar{C}(q,\dot{q})\dot{\eta}+\bar{G}(q)=\bar{\tau}
```

The transformed inertia matrix preserves useful properties:

```math
\bar{M}=\bar{M}^T>0
```

and a skew-symmetry property similar to:

```math
\dot{\bar{M}}-2\bar{C}
```

being skew-symmetric.

## Lyapunov Function

The lecture uses energy-like Lyapunov functions:

```math
V=
\frac{1}{2}z^T M z
+
\frac{1}{2}e^T K e
```

or:

```math
V=
\frac{1}{2}\|z\|^2
+
P(e)
```

where `P(e)` denotes the potential-like terms.

The controller is chosen so:

```math
\dot{V}\le -z^TKz
```

or:

```math
\dot{V}\le -c\|z\|^2
```

which implies convergence or exponential convergence.

## Bounded Auxiliary Terms

The notes repeatedly require certain auxiliary terms to be bounded. Typical assumptions:

```math
q_d,\dot{q}_d,\ddot{q}_d
```

are bounded.

Nonlinear functions are bounded on compact sets.

Gain choices must dominate bounded uncertainty or coupling terms.

## Regulation as a Special Case of Tracking

Tracking becomes regulation when:

```math
q_d(t)=q_d
```

with `q_d` constant.

Then:

```math
\dot{q}_d=0,\qquad \ddot{q}_d=0
```

Thus a controller written for tracking should reduce to a regulation controller under constant desired trajectories.

## Tracking as Regulation in Error Coordinates

The lecture emphasizes that tracking can be converted into regulation by defining:

```math
e=q_d(t)-q(t)
```

Then the control objective becomes:

```math
e(t)\to 0
```

This is regulation of the error dynamics.

## Final Stability Result

The notes conclude with a stability statement of the general form:

If the control gains are chosen sufficiently large and the auxiliary functions satisfy the required boundedness conditions, then:

```math
e(t)\to 0
```

and the relevant transformed errors remain bounded or converge.

In stronger cases:

```math
\|e(t)\|\le c e^{-\lambda t}\|e(0)\|
```

for positive constants `c,\lambda`.
