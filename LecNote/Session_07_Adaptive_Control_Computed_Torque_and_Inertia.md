# Session 07 - Adaptive Control: Computed-Torque and Inertia-Related Approaches

Sources:
- Lecture note: `Lex/lecture07.pdf`
- Reference: Ref.1, PDF pages 343 onward

## Motivation

Adaptive control is used when the robot structure is known but physical parameters are uncertain. Instead of using fixed nominal parameters, the controller estimates parameters online.

The robot dynamics are written in linearly parameterized form:

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)+F_d\dot{q}
=Y(q,\dot{q},\dot{q}_r,\ddot{q}_r)\theta
```

where:

- `Y` is the regressor matrix.
- `\theta` is the constant parameter vector.
- `\hat{\theta}` is the online estimate.
- `\tilde{\theta}=\theta-\hat{\theta}` is the parameter error.

## Adaptive Computed-Torque Idea

The model-based torque is computed with estimated parameters:

```math
\tau=Y(q,\dot{q},q_d,\dot{q}_d,\ddot{q}_d)\hat{\theta}+K_d\dot{e}+K_p e
```

or with a filtered reference acceleration depending on the derivation.

The parameter update law is chosen from the Lyapunov proof, not guessed independently.

## Standard Lyapunov Form

A common candidate is:

```math
V=\frac{1}{2}r^T M(q)r+\frac{1}{2}\tilde{\theta}^T\Gamma^{-1}\tilde{\theta}
```

where `\Gamma` is a positive definite adaptation gain matrix.

The update law is typically shaped as:

```math
\dot{\hat{\theta}}=\Gamma Y^T r
```

or the negative of this depending on the sign convention for `e` and `r`.

## Inertia-Related Approach

Instead of directly adapting the computed-torque controller, another approach builds the adaptive design around filtered error and the inertia matrix properties.

The key point is still the same:

1. Write uncertain dynamics as `Y\tilde{\theta}`.
2. Choose torque using `Y\hat{\theta}`.
3. Choose `\dot{\hat{\theta}}` to cancel parameter-error terms in `\dot{V}`.

## Parameter Convergence

Adaptive control can make tracking error converge even when the parameter estimates do not converge to the true parameters. Parameter convergence usually requires persistency of excitation, which is treated in the next session.

## Why This Session Matters for the Project

The term project gives a nine-parameter vector:

```math
\theta=[\beta_1,\beta_2,\beta_3,p_1,p_2,p_3,f_{d1},f_{d2},f_{d3}]^T
```

This makes adaptive control a very relevant candidate for one of the three controllers, but the hard part will be deriving or coding the regressor `Y` correctly.

