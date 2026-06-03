# Session 05 - PD, PID, Filtered Tracking Error, and Optimal Outer Loops

Sources:
- Lecture note: `Lex/Lecture04-05-06.pdf`, Lecture 5 block
- Reference: Ref.1, PDF pages 202-260
- `Filtered Tracking Error` was marked `/?` in `Table of Contents.pdf`; this note follows the lecture notes.

## PD Outer Loop

The computed-torque controller often uses:

```math
u=\ddot{q}_d+K_d\dot{e}+K_p e
```

which gives:

```math
\ddot{e}+K_d\dot{e}+K_p e=0
```

The main design issue is choosing gains that achieve small tracking error without excessive torque.

## PID Outer Loop

An integral term can reduce steady-state error:

```math
u=\ddot{q}_d+K_d\dot{e}+K_p e+K_i\int_0^t e(\sigma)d\sigma
```

This introduces third-order error dynamics. It can improve robustness against constant disturbances, but it can also create windup or slow transient behavior if not tuned carefully.

## PD Plus Gravity

For regulation, a simpler noncomputed-torque-like controller can be:

```math
\tau=G(q)+K_p(q_d-q)-K_d\dot{q}
```

For constant `q_d`, this can stabilize the robot without full cancellation of Coriolis/inertia terms. It is less model intensive than computed torque because it mainly uses gravity compensation.

## Filtered Tracking Error

A filtered tracking error combines position and velocity error:

```math
r=\dot{e}+\Lambda e
```

where `\Lambda` is positive definite. If `r -> 0` and the filter is stable, then `e -> 0`.

Using `r` is common in robust and adaptive robot control because the Lyapunov derivative can be written in terms of `r^T M r` and because desired filtered reference signals can be defined as:

```math
\dot{q}_r=\dot{q}_d+\Lambda e
```

```math
\ddot{q}_r=\ddot{q}_d+\Lambda \dot{e}
```

or with the opposite sign depending on the chosen definition of `e`.

## Optimal Outer-Loop Design

The reference material connects computed torque with linear quadratic regulation. Once feedback linearization gives a linear double-integrator model, one can design the outer loop using LQR-style state feedback:

```math
u=\ddot{q}_d-Kx_e
```

where `x_e=[e^T,\dot{e}^T]^T`, with sign adjusted to the error convention.

## Why This Session Matters for the Project

For the project, filtered error will be especially useful if we select an adaptive or robust controller. It also gives a clean way to define performance metrics:

- `max |e_i|`
- RMS tracking error
- `\|r\|`
- torque magnitude

