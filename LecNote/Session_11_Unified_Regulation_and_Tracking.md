# Session 11 - Unified Solution to Regulation and Tracking Problems

Sources:
- Lecture note: `Lex/lecture11.pdf`
- Reference location was marked `/?`; this note follows the lecture notes.

## Main Theme

This session develops a unified way to handle regulation and tracking for constrained/mechanical systems. The lecture notes use transformed coordinates and Lyapunov arguments so that a controller can cover both:

- Regulation: converge to a fixed target.
- Tracking: follow a time-varying desired trajectory.

## Error Transformation

A repeated idea is to define the error in a coordinate frame where the system structure becomes easier to control. This is similar to mobile robot tracking, where errors are expressed in the body frame.

The transformed error dynamics are then written in a form suitable for Lyapunov design:

```math
\dot{\eta}=f_\eta(\eta,t)+g_\eta(\eta,t)u
```

## Control Design Pattern

The general structure is:

1. Define an error variable that combines regulation and tracking cases.
2. Derive transformed error dynamics.
3. Choose a stabilizing input or virtual input.
4. Use a Lyapunov function to prove boundedness or convergence.

The notes also revisit bounded auxiliary terms and exponential-type estimates.

## Dynamics-Level Form

For mechanical systems, the control can again be shaped around:

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=\tau
```

or a transformed version of it. The proof usually depends on positive definiteness of inertia and cancellation or bounding of nonlinear terms.

## Why This Session Matters for the Project

The term project is a tracking problem because `q_d(t)` is time varying. The unified view helps frame the project report:

- Define the desired trajectory.
- Define the tracking error.
- Derive each controller from the same plant model.
- Compare convergence and robustness using the same metrics.

