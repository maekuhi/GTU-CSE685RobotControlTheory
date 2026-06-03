# Session 08 - Passivity-Based Adaptive Control, PE, and Composite Adaptation

Sources:
- Lecture note: `Lex/lecture08.pdf`
- Reference: Ref.1, PDF pages 363-380 area

## Passivity-Based Adaptive Control

Passivity-based adaptive control uses the natural robot structure. Define:

```math
r=\dot{e}+\Lambda e
```

and reference signals:

```math
\dot{q}_r=\dot{q}_d+\Lambda e,\qquad
\ddot{q}_r=\ddot{q}_d+\Lambda\dot{e}
```

Then the torque can be written:

```math
\tau=Y(q,\dot{q},\dot{q}_r,\ddot{q}_r)\hat{\theta}+K_r r
```

with an adaptation law chosen to make the Lyapunov derivative nonpositive.

## General Adaptive Update Rule

The standard update has the structure:

```math
\dot{\hat{\theta}}=\Gamma Y^T r
```

The sign depends on whether `e=q_d-q` or `e=q-q_d`. The important idea is that the update law cancels the mixed term involving `r^T Y\tilde{\theta}`.

## Persistency of Excitation

Tracking convergence does not automatically imply parameter convergence. Parameter estimates converge to true values only if the regressor is sufficiently exciting.

Informally, `Y(t)` is persistently exciting if over every finite time window it contains enough independent information about the parameters:

```math
\int_t^{t+T}Y^T(\sigma)Y(\sigma)d\sigma \ge \alpha I
```

for positive constants `T` and `\alpha`.

## Composite Adaptive Control

Composite adaptation uses both tracking error and prediction error. The lecture notes mention:

- Estimation based on prediction error.
- Filtered-based estimation.
- Torque filtering.
- Least-squares estimation.

The benefit is that parameter estimates can improve faster because the update law uses more information than tracking error alone.

## Filtering Idea

Measured torque and dynamic regressors can be filtered so that an algebraic prediction error is formed:

```math
\epsilon = \bar{\tau}-\bar{Y}\hat{\theta}
```

Then the update may include both terms:

```math
\dot{\hat{\theta}}=\Gamma(Y^T r+\bar{Y}^T\epsilon)
```

again with sign adjusted to the convention.

## Why This Session Matters for the Project

For the project, a basic passivity-based adaptive controller is more realistic than a full composite adaptive controller for a first implementation. Composite adaptation could be a bonus milestone if the basic adaptive controller works cleanly.

