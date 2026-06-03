# Lecture 03 - Robot Kinematics and Dynamics

Original handwritten source: `Lex/Lecture02-03.pdf`, Lecture 3 portion

Reference check:

- Checked against Ref.2 kinematics/dynamics pages and Ref.1 Lagrange/robot-equation properties.
- The notes below follow the handwritten sequence and use the references only to verify standard notation.

## Robot Manipulator Modeling

Robotic manipulator modeling is divided into:

- Kinematics: motion without forces.
- Dynamics: motion with forces and torques.

A robot consists of links connected by joints. The configuration is described by generalized coordinates:

```math
q=\begin{bmatrix}q_1&q_2&\cdots&q_n\end{bmatrix}^T
```

For revolute joints, `q_i` is an angle. For prismatic joints, `q_i` is a displacement.

## Direct Kinematics

Direct kinematics maps joint variables to end-effector pose:

```math
x=h(q)
```

where `x` may include position and orientation.

Using homogeneous transformations:

```math
T=
\begin{bmatrix}
R&p\\
0&1
\end{bmatrix}
```

where:

- `R` is a rotation matrix,
- `p` is a position vector.

For an open-chain robot:

```math
T_n^0=T_1^0T_2^1\cdots T_n^{n-1}
```

## Denavit-Hartenberg Parameters

The lecture notes introduce DH-style parameters:

- `a_i`: link length
- `\alpha_i`: link twist
- `d_i`: link offset
- `\theta_i`: joint angle

A common transformation is:

```math
T_i^{i-1}
=
R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i)
```

The full robot pose is found by multiplying the link transformations.

## Inverse Kinematics

Inverse kinematics asks:

```math
q=h^{-1}(x)
```

Given a desired end-effector pose `x_d`, find joint variables `q`.

The solution may be:

- unique,
- multiple,
- nonexistent,
- singularity-sensitive.

This is harder than direct kinematics because nonlinear equations must be solved.

## Joint Space and Task Space

Joint space:

```math
q\in\mathbb{R}^n
```

Task space:

```math
x\in\mathbb{R}^m
```

The robot may be controlled in joint space or task space. The term project gives a joint-space trajectory, so joint-space control is the main project path.

## Differential Kinematics

Differentiate:

```math
x=h(q)
```

to obtain:

```math
\dot{x}=J(q)\dot{q}
```

where:

```math
J(q)=\frac{\partial h(q)}{\partial q}
```

is the Jacobian matrix.

The Jacobian maps joint velocities into task-space velocities.

## Acceleration-Level Kinematics

Differentiating again:

```math
\ddot{x}=J(q)\ddot{q}+\dot{J}(q,\dot{q})\dot{q}
```

The extra term:

```math
\dot{J}\dot{q}
```

is important in task-space control and dynamics-level transformations.

## Dynamic Modeling Methods

Two main approaches are discussed:

1. Newton-Euler method
2. Lagrange method

Newton-Euler:

- force and moment balance,
- efficient recursive computation,
- useful for numerical inverse dynamics.

Lagrange:

- energy based,
- compact symbolic derivation,
- convenient for controller analysis.

## Lagrange Equations

Define:

```math
L(q,\dot{q})=K(q,\dot{q})-P(q)
```

where:

- `K`: kinetic energy,
- `P`: potential energy.

For each generalized coordinate:

```math
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right)
-
\frac{\partial L}{\partial q_i}
=
\tau_i
```

In vector form, this produces the manipulator dynamics.

## Energy Example for a Planar Manipulator

The handwritten notes work through a planar manipulator example.

Kinetic energy has the general form:

```math
K=\frac{1}{2}\dot{q}^TM(q)\dot{q}
```

Potential energy is built from the heights of the links:

```math
P=\sum_i m_i g h_i(q)
```

The Lagrangian:

```math
L=K-P
```

is substituted into Lagrange's equation to derive torques:

```math
\tau_i=
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right)
-
\frac{\partial L}{\partial q_i}
```

## Standard Robot Equation

The standard manipulator equation is:

```math
M(q)\ddot{q}+V_m(q,\dot{q})\dot{q}+G(q)+F_d\dot{q}=\tau
```

or:

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)+F(\dot{q})=\tau
```

where:

- `M(q)`: inertia matrix,
- `V_m(q,\dot{q})\dot{q}` or `C(q,\dot{q})\dot{q}`: Coriolis and centrifugal effects,
- `G(q)`: gravity,
- `F_d\dot{q}`: viscous friction,
- `\tau`: applied joint torque.

## Structural Properties of Robot Dynamics

Important properties:

### Inertia matrix

```math
M(q)=M^T(q)>0
```

and there exist positive constants `m_1,m_2` such that:

```math
m_1\|x\|^2\le x^TM(q)x\le m_2\|x\|^2
```

### Skew-symmetry

The Coriolis matrix can be chosen so that:

```math
\dot{M}(q)-2V_m(q,\dot{q})
```

is skew-symmetric.

Therefore:

```math
x^T\left(\dot{M}-2V_m\right)x=0
```

This is a key identity in Lyapunov proofs.

### Friction and gravity bounds

Viscous friction:

```math
F_d\dot{q}
```

Gravity is bounded for typical manipulators:

```math
\|G(q)\|\le g_b
```

## Linear Parameterization

Robot dynamics can often be written as:

```math
M(q)\ddot{q}+V_m(q,\dot{q})\dot{q}+G(q)+F_d\dot{q}
=Y(q,\dot{q},\ddot{q})\theta
```

where:

- `Y` is a known regressor,
- `\theta` is a constant parameter vector.

This property is the basis of adaptive robot control.

## Relation to the Project

The project robot is already given in standard manipulator form. Lecture 03 tells us how to interpret and implement:

- `M(q)`,
- `V_m(q,\dot{q})`,
- `F_d`,
- the parameter vector,
- the state-space simulation form.

The same structural properties will be used later to justify robust and adaptive controllers.

