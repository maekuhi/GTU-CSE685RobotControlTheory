# Lecture 09 - Task-Space Control

Original handwritten source: `Lex/lecture09.pdf`

Reference check:

- Checked against Ref.2 task-space control pages listed in `Table of Contents.pdf`.
- Some table entries were marked `/?`; handwritten lecture notes remain primary.

## Motivation

Up to this point, many controllers were written in joint space:

```math
q(t)\rightarrow q_d(t)
```

In many robot tasks, however, the desired motion is specified in task space:

```math
x(t)\rightarrow x_d(t)
```

where `x` describes end-effector position and/or orientation.

The task-space relation is:

```math
x=h(q)
```

Differentiating:

```math
\dot{x}=J(q)\dot{q}
```

where `J(q)` is the manipulator Jacobian.

## Kinematic Control

Kinematic control uses the differential kinematic relation to generate joint commands from task-space commands.

If the Jacobian is square and nonsingular:

```math
\dot{q}=J^{-1}(q)\dot{x}
```

For desired task velocity:

```math
\dot{x}_d
```

one may choose:

```math
\dot{q}=J^{-1}(q)\left(\dot{x}_d+K_x(x_d-x)\right)
```

Then task-space error:

```math
e_x=x_d-x
```

can be made to satisfy:

```math
\dot{e}_x+K_xe_x=0
```

if the inverse is exact and nonsingular.

## Pseudoinverse

If `J` is not square, use the pseudoinverse:

```math
\dot{q}=J^\dagger(q)\dot{x}
```

For a full row-rank Jacobian:

```math
J^\dagger=J^T(JJ^T)^{-1}
```

For a full column-rank Jacobian:

```math
J^\dagger=(J^TJ)^{-1}J^T
```

The pseudoinverse gives a least-squares or minimum-norm solution depending on the dimensions and rank.

## Redundant Manipulators

A manipulator is kinematically redundant if it has more joints than needed for the task:

```math
n>m
```

where:

- `n`: number of joints,
- `m`: task dimension.

Then the null space of `J` is nonempty.

The general velocity solution is:

```math
\dot{q}=J^\dagger\dot{x}+(I-J^\dagger J)\dot{q}_0
```

The term:

```math
(I-J^\dagger J)\dot{q}_0
```

does not affect task-space velocity because:

```math
J(I-J^\dagger J)=0
```

for the ideal full-rank case.

## Null-Space Objectives

The lecture notes mention that the null-space term can be used for secondary objectives:

- avoid joint limits,
- avoid obstacles,
- avoid singularities,
- optimize manipulability,
- reduce energy,
- choose a comfortable posture.

A common form:

```math
\dot{q}_0=k_0\nabla H(q)
```

where `H(q)` is a performance or avoidance function.

## Damped Least-Squares Inverse

Near singularities, the pseudoinverse can produce very large joint velocities.

Damped least squares modifies the inverse:

```math
J^\#=J^T(JJ^T+\lambda^2I)^{-1}
```

where:

```math
\lambda>0
```

The damping improves numerical behavior near singularities but introduces tracking approximation error.

## Jacobian Transpose Method

Instead of using the inverse Jacobian, task-space forces can be mapped to joint torques:

```math
\tau=J^T(q)F
```

where `F` is a task-space force-like control input.

For task-space regulation:

```math
F=K_p(x_d-x)-K_d\dot{x}
```

Then:

```math
\tau=J^T(q)\left(K_p(x_d-x)-K_d\dot{x}\right)
```

The transpose method avoids matrix inversion and can be robust near singularities, but the convergence behavior depends on Jacobian rank and gain choices.

## Cartesian Dynamics

Joint dynamics:

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=\tau
```

Task velocity:

```math
\dot{x}=J(q)\dot{q}
```

Task acceleration:

```math
\ddot{x}=J(q)\ddot{q}+\dot{J}(q,\dot{q})\dot{q}
```

The task-space dynamics can be written in a Cartesian form involving an operational-space inertia matrix. The notes emphasize that the transformation is not just `J^{-1}` because of the `\dot{J}\dot{q}` term and because singularities matter.

## Task-Space Control of Nonredundant Manipulators

For nonredundant systems, if `J` is invertible:

```math
\ddot{q}=J^{-1}(\ddot{x}-\dot{J}\dot{q})
```

This can be combined with joint-space inverse dynamics to construct task-space computed torque.

## Task-Space Control of Redundant Manipulators

For redundant manipulators:

```math
\ddot{q}=J^\dagger(\ddot{x}-\dot{J}\dot{q})+(I-J^\dagger J)\ddot{q}_0
```

The null-space term handles secondary objectives while the main term follows the task.

## Stability Comments

Task-space stability depends on:

- nonsingularity or rank of `J`,
- positive definite gains,
- correct handling of `\dot{J}\dot{q}`,
- avoiding or damping singularities,
- consistency between task-space force and joint torque.

## Relation to the Project

The term project gives a joint-level desired trajectory, so task-space control is not required for the main implementation. However, task-space ideas can be useful for optional plots of end-effector motion for the 3-DOF planar robot.

