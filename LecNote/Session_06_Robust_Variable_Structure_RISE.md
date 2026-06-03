# Session 06 - Robust Control, Variable Structure Control, and RISE

Sources:
- Lecture note: `Lex/Lecture04-05-06.pdf`, Lecture 6 block
- Supplement: `Lex/Lecture06Suplementary.PDF`
- Reference locations for robust control, variable-structure control, and RISE were marked `/?` in `Table of Contents.pdf`; this note follows the lecture notes.

## Robust Control Motivation

Computed torque assumes exact model knowledge. Robust control accepts model uncertainty and designs the control input so that tracking error remains bounded or converges despite bounded uncertainty.

For robot dynamics:

```math
M(q)\ddot{q}+N(q,\dot{q})=\tau
```

we may only know estimates:

```math
\hat{M}(q),\qquad \hat{N}(q,\dot{q})
```

and uncertainty terms remain after cancellation.

## Sliding or Variable-Structure Control

Define filtered error:

```math
r=\dot{e}+\Lambda e
```

A sliding-mode-like design often includes:

```math
\tau=\hat{M}(q)\ddot{q}_r+\hat{N}(q,\dot{q})+K r+\rho\,\mathrm{sgn}(r)
```

or a related sign/saturation term. The discontinuous term is chosen large enough to dominate bounded uncertainty.

Important tradeoff:

- Strong robustness against uncertainty.
- Possible chattering due to discontinuous sign control.
- Saturation or boundary layers can reduce chattering but may leave small steady-state error.

## Lyapunov Pattern

A common candidate is:

```math
V=\frac{1}{2}r^T M(q)r
```

Using robot skew-symmetry properties, the derivative can often be bounded like:

```math
\dot{V}\le -r^T K r + \|r\|\|\Delta\| - \rho\|r\|
```

If `\rho` is larger than the uncertainty bound, then the error decreases.

## RISE Controller

RISE stands for Robust Integral of the Sign of the Error. It is used to obtain continuous control while retaining robustness properties similar to sign-based control.

The supplement uses a filtered error structure and an auxiliary signal such as:

```math
s=r+\alpha e
```

or related filtered variables depending on notation. The controller includes an integral of a sign-type signal rather than a raw discontinuous torque:

```math
\int_0^t \mathrm{sgn}(r(\sigma))\,d\sigma
```

The aim is to avoid direct discontinuity in the actual torque while still rejecting bounded nonlinear uncertainty.

## Why This Session Matters for the Project

Robust or RISE control could be one of the three project controllers, but it is more delicate than computed torque:

- We need clear uncertainty assumptions.
- We need choose gains carefully.
- We should report whether chattering or large torques appear in simulation.

For a step-by-step project, I suggest implementing robust/sliding control only after the robot model and computed-torque baseline are verified.

