# Lecture 08 - Passivity-Based Adaptive Control, Persistency of Excitation, and Composite Adaptation

Original handwritten source: `Lex/lecture08.pdf`

Reference check:

- Checked against Ref.1 sections on passivity-based adaptive controllers, general update rules, persistency of excitation, and composite adaptive control.
- Formula signs are kept consistent with the lecture convention `e=q_d-q`.

## Page 1 - Adaptive Control Based on Passivity

The lecture begins with passivity-based adaptive control.

For a robot manipulator:

```math
M(q)\ddot{q}+V_m(q,\dot{q})\dot{q}+G(q)+F_d\dot{q}=\tau
```

Define tracking error:

```math
e=q_d-q
```

and filtered tracking error:

```math
r=\dot{e}+\Lambda e
```

The controller is designed so that the map from parameter error to filtered error is passive or energy bounded.

## Page 2 - Passivity Integral

The notes use an inequality of the form:

```math
\int_0^t r^T(\sigma)Y(\sigma)\tilde{\theta}(\sigma)d\sigma
```

and choose the update law so the parameter-estimation term does not inject net energy into the tracking system.

The update rule is connected to the passivity condition.

## Page 3 - Passivity-Based Lyapunov Argument

Use:

```math
V=
\frac{1}{2}r^TM(q)r
\frac{1}{2}\tilde{\theta}^T\Gamma^{-1}\tilde{\theta}
```

Then, after substitution:

```math
\dot{V}
=-r^TK_rr
r^TY\tilde{\theta}
\tilde{\theta}^T\Gamma^{-1}\dot{\tilde{\theta}}
```

Choose the adaptation rule to cancel:

```math
r^TY\tilde{\theta}
```

so:

```math
\dot{V}=-r^TK_rr\le 0
```

## Page 4 - General Adaptive Update Rule

The notes describe a general adaptive law:

```math
\dot{\hat{\theta}}=\Gamma Y^T r
```

or a normalized/filtered variant.

The update law:

- uses tracking error information,
- keeps parameter estimates bounded under the Lyapunov assumptions,
- is not guaranteed to identify true parameters unless excitation is sufficient.

## Page 5 - Persistency of Excitation

The lecture introduces the problem:

Tracking can converge even if parameter estimates do not converge.

Parameter convergence requires persistency of excitation.

A regressor `Y(t)` is persistently exciting if there exist constants:

```math
\alpha_1>0,\quad \alpha_2>0,\quad T>0
```

such that:

```math
\alpha_1 I
\le
\int_t^{t+T}Y^T(\sigma)Y(\sigma)d\sigma
\le
\alpha_2 I
```

for all `t`.

## Page 6 - Prediction Error

The notes introduce prediction error for parameter estimation.

Suppose a measurable signal can be written:

```math
y=W\theta
```

where:

- `W` is a known regressor,
- `\theta` is the unknown parameter vector.

Prediction using estimates:

```math
\hat{y}=W\hat{\theta}
```

Prediction error:

```math
\epsilon=y-\hat{y}=W\tilde{\theta}
```

This error can be used for parameter adaptation.

## Page 7 - Filtering for Estimation

Robot dynamics contain accelerations and torques that may not be directly convenient for estimation.

Filters are introduced so that a regression equation is obtained without requiring noisy acceleration measurements.

Filtered form:

```math
\bar{\tau}=\bar{Y}\theta
```

Prediction error:

```math
\epsilon=\bar{\tau}-\bar{Y}\hat{\theta}
```

## Page 8 - Filtering the Dynamics

The lecture notes show filtering of the robot dynamics through a stable filter such as:

```math
H(s)=\frac{\lambda}{s+\lambda}
```

or:

```math
\frac{1}{s+a}
```

The goal is to transform:

```math
\tau=Y\theta
```

into:

```math
\tau_f=Y_f\theta
```

where filtered signals are measurable or easier to compute.

## Page 9 - Composite Adaptive Control

Composite adaptation uses both:

1. tracking error,
2. prediction error.

A typical update law:

```math
\dot{\hat{\theta}}
=
\Gamma\left(Y^Tr+\bar{Y}^T\epsilon\right)
```

with sign adjusted to the convention.

The tracking term helps control performance. The prediction-error term improves parameter estimation.

## Page 10 - Least-Squares Estimation

The notes discuss least-squares estimation.

For:

```math
y=W\theta
```

estimate `\theta` by minimizing:

```math
J=(y-W\hat{\theta})^T(y-W\hat{\theta})
```

The normal equation is:

```math
W^TW\hat{\theta}=W^Ty
```

If `W^TW` is invertible:

```math
\hat{\theta}=(W^TW)^{-1}W^Ty
```

This again requires enough excitation or rank in the regressor.

## Page 11 - Composite Lyapunov Argument

The composite adaptive proof adds a prediction-error term to the Lyapunov derivative.

If:

```math
\epsilon=\bar{Y}\tilde{\theta}
```

then:

```math
\tilde{\theta}^T\bar{Y}^T\epsilon
=
\epsilon^T\epsilon
```

The update law can make:

```math
\dot{V}
=
-r^TK_rr-\epsilon^T\epsilon
```

or a similar negative semidefinite expression.

This improves parameter convergence when excitation is available.

## Page 12 - Example Adaptive Controller

The notes show an example adaptive controller with an update law and Lyapunov proof.

The general structure is:

```math
\tau=Y\hat{\theta}+K_rr
```

```math
\dot{\hat{\theta}}=\Gamma Y^Tr
```

For composite control:

```math
\dot{\hat{\theta}}
=
\Gamma(Y^Tr+\bar{Y}^T\epsilon)
```

The proof follows the same cancellation pattern.

## Page 13 - Parameter Convergence Conditions

If the regressor is persistently exciting, then:

```math
\tilde{\theta}(t)\to 0
```

If not, tracking may still converge:

```math
e(t)\to 0
```

but:

```math
\tilde{\theta}(t)\not\to 0
```

This is not a failure of the controller; it means the trajectory did not contain enough information to identify every parameter.

## Pages 14-16 - Scalar Estimation Example

The notes include a scalar example with a filtered or first-order system:

```math
y=\theta x
```

or a similar regression.

Prediction:

```math
\hat{y}=\hat{\theta}x
```

Error:

```math
\epsilon=y-\hat{y}=x\tilde{\theta}
```

An update law of the form:

```math
\dot{\hat{\theta}}=\gamma x\epsilon
```

drives the prediction error down when `x` is sufficiently exciting.

The example illustrates the same mechanism used in robot adaptive control.

## Relation to the Project

For the project, this lecture suggests two possible adaptive levels:

1. Basic passivity-based adaptive control using `Y(q,\dot{q},\dot{q}_r,\ddot{q}_r)\hat{\theta}`.
2. Composite adaptation if we want better parameter estimation.

For a strong but manageable project, the basic passivity-based adaptive controller is the better choice. Composite adaptation can be discussed as an extension unless the project time allows full implementation.

