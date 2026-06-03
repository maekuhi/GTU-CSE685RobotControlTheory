# Lecture 01 - Control Theory Preliminaries

Original handwritten source: `Lex/Lecture01.pdf`

## Page 1 - Introduction to Control Theory

Modern industrial robots are widely used in manufacturing and many practical engineering systems. The general control objective is to make a system behave according to a desired operation. In robotics, this usually means:

- Drive the robot to a desired joint or task-space position.
- Track a desired trajectory.
- Reject disturbances and modeling uncertainty.
- Keep the motion stable and physically meaningful.

The notes introduce control theory as a set of mathematical tools used to design controllers for such systems.

### Linear state-variable systems

A linear system with state `x`, input `u`, and output `y` can be described by differential equations. The state is the minimum set of variables needed to describe the future behavior of the system when the input is known.

The state-space representation is central because it applies naturally to mechanical systems.

## Page 2 - Continuous-Time Systems and Block Diagrams

The lecture starts with continuous-time systems. A system can be described by an input-output relation:

```math
u(t) \rightarrow \text{system} \rightarrow y(t)
```

For single-input single-output systems, the relation may be represented by a differential equation:

```math
a_n\frac{d^n y(t)}{dt^n}+a_{n-1}\frac{d^{n-1}y(t)}{dt^{n-1}}+\cdots+a_0y(t)
=
b_m\frac{d^m u(t)}{dt^m}+\cdots+b_0u(t)
```

The note distinguishes:

- SISO: single input, single output.
- MIMO: multiple inputs, multiple outputs.
- Continuous-time: variables depend on continuous `t`.
- Linear systems: superposition holds.

## Page 3 - State Definition and Spring-Mass Example

The state of a system is a set of variables whose values at an initial time, together with the input, determine the future behavior.

Example: mass-spring-damper system

```math
M\ddot{x}+B\dot{x}+Kx=F(t)
```

Choose:

```math
x_1=x,\qquad x_2=\dot{x}
```

Then:

```math
\dot{x}_1=x_2
```

```math
\dot{x}_2=-\frac{K}{M}x_1-\frac{B}{M}x_2+\frac{1}{M}F(t)
```

The state vector is:

```math
X=\begin{bmatrix}x_1\\x_2\end{bmatrix}
```

## Page 4 - State-Space Form of the Spring-Mass System

Using the previous example:

```math
\dot{X}=AX+Bu
```

```math
y=CX+Du
```

For the mass-spring-damper system:

```math
A=
\begin{bmatrix}
0 & 1\\
-K/M & -B/M
\end{bmatrix},
\qquad
B=
\begin{bmatrix}
0\\
1/M
\end{bmatrix}
```

If the output is position:

```math
C=\begin{bmatrix}1&0\end{bmatrix},
\qquad D=0
```

This is called the state-space form.

## Page 5 - Alternative State Choices

The same physical system can be represented using different state coordinates. For example, another choice of state may be:

```math
\bar{x}_1=x,\qquad \bar{x}_2=x+\dot{x}
```

Then the state equations change, but the physical input-output behavior remains the same. This emphasizes that a state representation is not unique.

The notes show that different state variables can lead to a different `A`, `B`, `C`, and `D`, while representing the same physical process.

## Page 6 - General nth-Order Differential Equation to State Space

For an nth-order differential equation:

```math
\frac{d^n y}{dt^n}
a_{n-1}\frac{d^{n-1}y}{dt^{n-1}}
\cdots+a_1\dot{y}+a_0y=u
```

define phase variables:

```math
x_1=y,\quad x_2=\dot{y},\quad \ldots,\quad x_n=y^{(n-1)}
```

Then:

```math
\dot{x}_1=x_2,\quad \dot{x}_2=x_3,\quad \ldots,\quad \dot{x}_{n-1}=x_n
```

```math
\dot{x}_n=-a_0x_1-a_1x_2-\cdots-a_{n-1}x_n+u
```

This gives the controllable canonical form.

## Page 7 - Double Integrator Example

For the system:

```math
\ddot{y}=u
```

choose:

```math
x_1=y,\qquad x_2=\dot{y}
```

Then:

```math
\dot{x}_1=x_2,\qquad \dot{x}_2=u
```

So:

```math
\dot{x}
=
\begin{bmatrix}
0&1\\
0&0
\end{bmatrix}x

\begin{bmatrix}
0\\
1
\end{bmatrix}u
```

```math
y=\begin{bmatrix}1&0\end{bmatrix}x
```

This double-integrator structure will later appear after computed-torque feedback linearization.

## Page 8 - Transfer Function Derivation

For the LTI state-space system:

```math
\dot{x}=Ax+Bu,\qquad y=Cx+Du
```

take the Laplace transform:

```math
sX(s)-x(0)=AX(s)+BU(s)
```

Assuming zero initial condition:

```math
(sI-A)X(s)=BU(s)
```

```math
X(s)=(sI-A)^{-1}BU(s)
```

Substitute into the output equation:

```math
Y(s)=\left[C(sI-A)^{-1}B+D\right]U(s)
```

Therefore:

```math
G(s)=\frac{Y(s)}{U(s)}=C(sI-A)^{-1}B+D
```

## Page 9 - Block Diagram Representation

The transfer function can be represented by a block diagram. The system state-space representation can be pictured as:

- `B` maps input into the state derivative.
- `1/s` integrates state derivatives into states.
- `A` feeds state back into the derivative.
- `C` maps states into output.
- `D` gives direct feedthrough.

The final transfer relation is:

```math
Y(s)=G(s)U(s)
```

## Page 10 - Double Integrator Transfer Function and Discrete Time

Example:

```math
\ddot{y}=Ku
```

Taking the Laplace transform with zero initial condition:

```math
s^2Y(s)=KU(s)
```

Therefore:

```math
G(s)=\frac{Y(s)}{U(s)}=\frac{K}{s^2}
```

### Discrete-time systems

In discrete time, the system is described at sample instants:

```math
x(k+1)=Ax(k)+Bu(k)
```

```math
y(k)=Cx(k)+Du(k)
```

The discrete transfer function is:

```math
G(z)=C(zI-A)^{-1}B+D
```

## Page 11 - Nonlinear State-Variable Systems

Many systems, especially robots, are nonlinear. A continuous-time nonlinear system is:

```math
\dot{x}=f(x,u,t)
```

```math
y=h(x,u,t)
```

For autonomous nonlinear systems:

```math
\dot{x}=f(x)
```

For nonautonomous systems:

```math
\dot{x}=f(x,t)
```

The notes emphasize that linear state-space tools are not enough for general robot dynamics because the equations depend on `q`, `\dot{q}`, and nonlinear trigonometric terms.

## Page 12 - Examples of Nonlinear Systems

### Damped pendulum

The pendulum equation is:

```math
\ddot{y}+b\dot{y}+\sin(y)=0
```

Choose:

```math
x_1=y,\qquad x_2=\dot{y}
```

Then:

```math
\dot{x}_1=x_2
```

```math
\dot{x}_2=-b x_2-\sin(x_1)
```

This is nonlinear due to `\sin(x_1)`.

### Van der Pol oscillator

The notes also mention a nonlinear oscillator of Van der Pol type, where the damping term depends on the state. It illustrates that nonlinear systems can have behavior not captured by linear systems.

## Page 13 - First-Order Linear System Driven by Nonlinear Terms

The notes give an example of a nonlinear-time or nonlinear-state system:

```math
\dot{x}=A(t)x+G(x)u
```

or a similar form in which the linear part and nonlinear input coupling are separated.

This is useful because many nonlinear systems can be written as:

```math
\dot{x}=f(x)+g(x)u
```

Such systems are called control-affine. Robot dynamics often become control-affine once the state is chosen as `x=[q^T,\dot{q}^T]^T`.

## Page 14 - Beginning of Stability Theory

The lecturer transitions to stability theory with the main question:

What happens to a system after a small change in initial condition or after a small disturbance?

Stability asks whether the system remains near an equilibrium or desired behavior.

The note introduces:

- Norms and distances.
- Positive definite functions.
- Equilibrium points.
- Lyapunov-style reasoning.

## Page 15 - Common Norms

For a vector:

```math
x=\begin{bmatrix}x_1&x_2&\cdots&x_n\end{bmatrix}^T
```

common norms are:

```math
\|x\|_1=\sum_{i=1}^n |x_i|
```

```math
\|x\|_2=\left(\sum_{i=1}^n x_i^2\right)^{1/2}
```

```math
\|x\|_p=\left(\sum_{i=1}^n |x_i|^p\right)^{1/p}
```

```math
\|x\|_\infty=\max_i |x_i|
```

Example:

```math
x=\begin{bmatrix}1\\-2\\3\end{bmatrix}
```

Then:

```math
\|x\|_1=6,\qquad \|x\|_2=\sqrt{14},\qquad \|x\|_\infty=3
```

## Page 16 - Equivalence of Norms

In finite-dimensional vector spaces, all norms are equivalent. This means there exist positive constants `k_1` and `k_2` such that:

```math
k_1\|x\|_a \le \|x\|_b \le k_2\|x\|_a
```

This is important because proving boundedness in one norm implies boundedness in another norm.

### Induced matrix norm

For a matrix `A`, an induced matrix norm is:

```math
\|A\|=\max_{x\neq 0}\frac{\|Ax\|}{\|x\|}
```

It satisfies:

```math
\|Ax\|\le \|A\|\,\|x\|
```

## Page 17 - Matrix Norm Examples

Common matrix norms:

```math
\|A\|_\infty=\max_i\sum_j |a_{ij}|
```

maximum absolute row sum.

```math
\|A\|_1=\max_j\sum_i |a_{ij}|
```

maximum absolute column sum.

```math
\|A\|_2=\sqrt{\lambda_{\max}(A^TA)}
```

For symmetric matrices:

```math
\|A\|_2=\max_i |\lambda_i(A)|
```

The page includes an example matrix and computes row/column norm quantities.

## Page 18 - Function Norms

For a function `f(t)`, uniform continuity and boundedness are discussed.

A function `f(t)` is uniformly continuous if for every `\epsilon>0`, there exists a `\delta>0` such that:

```math
|t_2-t_1|<\delta \Rightarrow |f(t_2)-f(t_1)|<\epsilon
```

This condition is important later for Barbalat's lemma.

## Page 19 - Function Spaces and Examples

For vector-valued functions:

```math
f(t)=\begin{bmatrix}f_1(t)\\f_2(t)\\\vdots\\f_n(t)\end{bmatrix}
```

an `L_p` norm can be written as:

```math
\|f\|_p=\left(\int_{t_0}^{t_1}\|f(t)\|^pdt\right)^{1/p}
```

For `p=\infty`:

```math
\|f\|_\infty=\max_{t\in[t_0,t_1]}\|f(t)\|
```

Examples in the notes compare constant, exponential, and sinusoidal functions.

## Page 20 - System Norms and BIBO Stability

The notes define a system operator `H` that maps input `u(t)` to output `y(t)`:

```math
y=Hu
```

The induced gain is:

```math
\|H\|=\sup_{u\neq 0}\frac{\|Hu\|}{\|u\|}
```

Bounded-input bounded-output stability means:

```math
\|u\|<\infty \Rightarrow \|y\|<\infty
```

For LTI systems, BIBO stability relates to the impulse response being absolutely integrable.

## Page 21 - Input-Output Stability for LTI Systems

For a linear system:

```math
y(t)=\int_{-\infty}^{t}h(t-\tau)u(\tau)d\tau
```

If the impulse response is integrable, the system is BIBO stable:

```math
\int_0^\infty |h(\tau)|d\tau<\infty
```

The page shows examples of stable and unstable impulse responses, including decaying exponentials and nondecaying/unstable modes.

## Page 22 - Matrix Properties

Definitions:

### Positive definite matrix

```math
x^TAx>0,\qquad \forall x\neq 0
```

### Positive semidefinite matrix

```math
x^TAx\ge 0,\qquad \forall x
```

### Negative definite matrix

```math
x^TAx<0,\qquad \forall x\neq 0
```

### Skew-symmetric matrix

```math
A^T=-A
```

For skew-symmetric `A`:

```math
x^TAx=0
```

This skew-symmetric property is crucial for robot dynamics.

## Page 23 - Eigenvalue Conditions and Matrix Tests

For a real symmetric matrix `A`, the following are equivalent:

- `A` is positive definite.
- All eigenvalues of `A` are positive.
- `x^TAx>0` for all nonzero `x`.

For positive semidefinite matrices, all eigenvalues are nonnegative.

The page also mentions tests such as principal minors and matrix inequalities.

## Page 24 - Matrix Example

An example matrix is checked for positive definiteness by evaluating eigenvalues and/or principal minors.

For a `2 x 2` symmetric matrix:

```math
A=
\begin{bmatrix}
a&b\\
b&d
\end{bmatrix}
```

positive definiteness requires:

```math
a>0,\qquad ad-b^2>0
```

The example reinforces how to check if a matrix can be used as a Lyapunov weight matrix.

## Relation to the Project

Lecture 1 gives the mathematical setup for the Matlab project:

- The robot simulation state will be `x=[q^T,\dot{q}^T]^T`.
- Computed-torque control turns robot tracking into double-integrator-like error dynamics.
- Norms will be used to measure tracking error.
- Positive definite gain matrices are needed for controller stability.
- Skew-symmetric matrix properties will later simplify Lyapunov proofs.

