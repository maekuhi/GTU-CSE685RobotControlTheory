# Session 01 - Control Theory Preliminaries

Sources:
- Lecture note: `Lex/Lecture01.pdf`
- Reference: Ref.1, PDF pages 40-70 according to `Table of Contents.pdf`

## Big Picture

This session builds the mathematical language used later for robot control. The robot manipulator is ultimately treated as a nonlinear dynamical system, but many controller designs first rewrite tracking error dynamics into a linear or linear-like state-space form.

## Control System Description

A control system is described by input, state, and output variables. The state vector contains enough information to predict the future behavior of the system when the input is known.

Continuous-time state-space form:

```math
\dot{x}(t) = A x(t) + B u(t), \qquad y(t)=C x(t)+D u(t)
```

For nonlinear systems:

```math
\dot{x}(t) = f(x(t),u(t),t), \qquad y(t)=h(x(t),u(t),t)
```

In robot control, the common state choice is:

```math
x =
\begin{bmatrix}
q \\
\dot{q}
\end{bmatrix}
```

where `q` is the joint-position vector and `\dot{q}` is the joint-velocity vector.

## Transfer Functions

For linear time-invariant systems, the transfer function maps input to output in the Laplace domain:

```math
G(s)=C(sI-A)^{-1}B+D
```

The lecture notes emphasize that transfer functions are useful for linear systems, while robot dynamics usually require state-space and Lyapunov tools because the dynamics are nonlinear and configuration dependent.

## Discrete-Time Systems

Discrete-time systems use sequences:

```math
x(k+1)=A x(k)+B u(k), \qquad y(k)=C x(k)+D u(k)
```

This matters for simulation and digital control. Even if the controller is designed in continuous time, Matlab simulation uses numerical integration or discrete time samples.

## Nonlinear Systems and Equilibria

An equilibrium point `x_e` satisfies:

```math
f(x_e,0,t)=0
```

For autonomous systems:

```math
\dot{x}=f(x)
```

the equilibrium is found from `f(x_e)=0`. Later, tracking control is converted into an equilibrium problem for the tracking error: make `e=0`, `\dot{e}=0` stable.

## Vector Spaces and Norms

Norms measure the size of vectors, matrices, functions, and signals.

Common vector norms:

```math
\|x\|_1=\sum_i |x_i|, \qquad
\|x\|_2=\sqrt{x^T x}, \qquad
\|x\|_\infty=\max_i |x_i|
```

Matrix norms are used to bound system behavior and prove stability. Important matrix concepts:

- Positive definite: `x^T A x > 0` for all nonzero `x`.
- Positive semidefinite: `x^T A x >= 0`.
- Symmetric positive definite matrices are often used as Lyapunov weights.

## Why This Session Matters for the Project

The 3-DOF robot project will use:

- State vector `x=[q^T, \dot{q}^T]^T`.
- Tracking error `e=q_d-q` or `e=q-q_d`, depending on chosen convention.
- Norms such as `\|e\|` and `\|\dot{e}\|` to compare controller performance.
- Positive definite gain matrices such as `K_p`, `K_d`, `\Lambda`, and adaptation gains.

