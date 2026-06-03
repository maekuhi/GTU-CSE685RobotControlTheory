# Session 09 - Task-Space Control

Sources:
- Lecture note: `Lex/lecture09.pdf`
- Reference: Ref.2, PDF pages 115 onward
- Some subtopics were marked `/?`; this note follows lecture notes plus the task-space chapter.

## Joint Space vs Task Space

Joint-space control tracks `q_d(t)`.

Task-space control tracks end-effector variables:

```math
x=h(q)
```

with velocity relation:

```math
\dot{x}=J(q)\dot{q}
```

## Kinematic Control

Given desired task velocity `\dot{x}_d`, choose joint velocity:

```math
\dot{q}=J^{-1}(q)\dot{x}_d
```

when `J` is square and nonsingular.

For non-square or redundant systems:

```math
\dot{q}=J^\dagger(q)\dot{x}_d
```

where `J^\dagger` is the pseudoinverse.

## Redundancy Resolution

When the robot has more joints than task dimensions, the null space of `J` can be used:

```math
\dot{q}=J^\dagger \dot{x}_d+(I-J^\dagger J)\dot{q}_0
```

The second term does not change the task velocity. It can be used for:

- Joint-limit avoidance.
- Obstacle avoidance.
- Singularity avoidance.
- Energy or posture optimization.

## Jacobian Transpose Control

A common task-space regulation idea uses:

```math
\tau=J^T(q)F
```

where `F` is a task-space force-like command. For example:

```math
F=K_p(x_d-x)-K_d\dot{x}
```

This avoids direct inversion of `J`, but performance depends strongly on the Jacobian and gains.

## Cartesian Dynamics

Task-space dynamics are obtained by transforming joint dynamics through the Jacobian. The lecture notes emphasize the relation between joint torque and task force:

```math
\tau=J^T(q)F
```

and the fact that `\dot{J}(q,\dot{q})\dot{q}` terms appear when differentiating `\dot{x}=J\dot{q}`.

## Why This Session Matters for the Project

The term project trajectory is already given at joint level, so the main project does not require task-space control. Still, this session helps if we add optional end-effector plots for the 3-DOF planar arm.

