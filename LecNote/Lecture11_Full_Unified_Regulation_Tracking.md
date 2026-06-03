# Lecture 11 - Unified Solution to Regulation and Tracking Problems

Original handwritten source: `Lex/lecture11.pdf`

Reference check:

- The table of contents marks this lecture as `/?`.
- This file follows the handwritten lecture notes and checks formulas against standard transformed-error and mechanical-system control logic.

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
\text{potential-like terms}
```

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
q_d(t)=q_d=\text{constant}
```

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

## Relation to the Project

This lecture is conceptually related to the project because the project is a tracking problem. In the report, each controller can be explained using the same unified pattern:

1. define tracking error,
2. rewrite closed-loop error dynamics,
3. choose gains/control terms,
4. verify stability,
5. compare simulation performance.

