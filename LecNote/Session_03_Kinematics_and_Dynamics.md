# Session 03 - Robot Kinematics and Dynamics

Sources:
- Lecture note: `Lex/Lecture02-03.pdf`, kinematics/dynamics block
- Reference: Ref.2, PDF pages 3 onward for kinematics and dynamics
- Reference: Ref.1, PDF pages 129 and 143 areas for Lagrange dynamics and robot equation properties

## Kinematics

Kinematics describes motion without considering forces.

Direct kinematics:

```math
x = h(q)
```

where `q` is joint space and `x` is task space.

Differential kinematics:

```math
\dot{x}=J(q)\dot{q}
```

The Jacobian maps joint velocities to end-effector velocity. Singularities occur when the Jacobian loses rank.

Inverse kinematics tries to find:

```math
q=h^{-1}(x)
```

which may be nonunique, nonexistent, or singularity-sensitive.

## Dynamics from Lagrange Equations

The Lagrangian is:

```math
L(q,\dot{q})=K(q,\dot{q})-P(q)
```

where `K` is kinetic energy and `P` is potential energy.

For generalized coordinate `q_i`:

```math
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right)
-\frac{\partial L}{\partial q_i}
=\tau_i
```

For an `n`-DOF robot, the standard form is:

```math
M(q)\ddot{q}+V_m(q,\dot{q})\dot{q}+G(q)+F_d\dot{q}=\tau
```

or more compactly:

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)+F(q,\dot{q})=\tau
```

## Structural Properties of Robot Dynamics

Important properties used later:

- `M(q)` is symmetric positive definite.
- `M(q)` is bounded above and below for physical robots.
- The Coriolis/centrifugal terms can be chosen so that `\dot{M}(q)-2C(q,\dot{q})` is skew-symmetric.
- Robot dynamics are linearly parameterizable:

```math
M(q)\ddot{q}_r+C(q,\dot{q})\dot{q}_r+G(q)+F_d\dot{q}
=Y(q,\dot{q},\dot{q}_r,\ddot{q}_r)\theta
```

This linear-in-parameters form is the backbone of adaptive control.

## Why This Session Matters for the Project

The term project gives a specific 3-DOF planar robot:

```math
M(q)\ddot{q}+V_m(q,\dot{q})\dot{q}+G(q)+F_d\dot{q}=\tau
```

The controller functions in Matlab should reflect this structure:

- `M = massMatrix(q,params)`
- `Vm = coriolisMatrix(q,dq,params)`
- `G = gravityVector(q,params)` if nonzero in the project model
- `Fd = frictionMatrix(params)`
- simulation state derivative from solving `M(q)\ddot{q}=...`

