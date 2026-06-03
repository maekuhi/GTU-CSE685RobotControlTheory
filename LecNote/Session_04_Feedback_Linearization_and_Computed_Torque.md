# Session 04 - Feedback Linearization and Computed-Torque Control

Sources:
- Lecture note: `Lex/Lecture04-05-06.pdf`, Lecture 4 block
- Reference: Ref.1, PDF pages 163-166 and 202 onward

## Feedback Linearization

For a nonlinear system, the goal is to choose a control input that cancels nonlinearities and produces a desired linear input-output relation.

For robot manipulators:

```math
M(q)\ddot{q}+N(q,\dot{q})=\tau
```

where:

```math
N(q,\dot{q})=V_m(q,\dot{q})\dot{q}+G(q)+F_d\dot{q}
```

Choose:

```math
\tau=M(q)u+N(q,\dot{q})
```

Then the closed-loop plant becomes:

```math
\ddot{q}=u
```

This is the inner nonlinear cancellation loop.

## Computed-Torque Control

Computed torque applies feedback linearization to robot tracking.

Let:

```math
e=q_d-q,\qquad \dot{e}=\dot{q}_d-\dot{q}
```

Choose the outer-loop acceleration command:

```math
u=\ddot{q}_d+K_d\dot{e}+K_p e
```

Then:

```math
\tau=M(q)(\ddot{q}_d+K_d\dot{e}+K_p e)+N(q,\dot{q})
```

If the model is exact:

```math
\ddot{e}+K_d\dot{e}+K_p e=0
```

With positive gains, the tracking error converges like a stable second-order system.

## Gain Selection

For diagonal gains, each joint error behaves approximately as:

```math
\ddot{e}_i+k_{d,i}\dot{e}_i+k_{p,i}e_i=0
```

Common design:

```math
k_{p,i}=\omega_{n,i}^2,\qquad k_{d,i}=2\zeta_i\omega_{n,i}
```

Critical damping uses `\zeta_i=1`.

## Limitations

Computed torque is powerful but model dependent:

- It requires accurate `M`, `V_m`, `G`, and friction terms.
- Large gains may increase torque demand.
- Exact cancellation is fragile under uncertainty.

## Why This Session Matters for the Project

Computed torque is a natural first controller for the 3-DOF robot project. It will also be the baseline for comparing robust and adaptive controllers.

