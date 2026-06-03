# Session 02 - Stability and Lyapunov Theory

Sources:
- Lecture note: `Lex/Lecture02-03.pdf`, first stability block
- Reference: Ref.1, PDF pages 70-112 according to `Table of Contents.pdf`

## Stability Concepts

The lecture introduces stability around an equilibrium point `x_e`.

Stability means that solutions starting close to `x_e` remain close:

```math
\|x(t_0)-x_e\|<\delta \Rightarrow \|x(t)-x_e\|<\epsilon,\quad t\ge t_0
```

Asymptotic stability adds convergence:

```math
\lim_{t\to\infty} x(t)=x_e
```

Exponential stability gives a rate:

```math
\|x(t)-x_e\| \le k e^{-\alpha(t-t_0)} \|x(t_0)-x_e\|
```

The notes distinguish local/global and uniform/nonuniform versions:

- Local stability: only for initial conditions in a neighborhood.
- Global stability: for all initial conditions.
- Uniform stability: bounds do not depend on the initial time.
- Uniform ultimate boundedness: trajectories eventually enter and stay in a bounded set.

## Lyapunov Direct Method

Instead of solving the differential equation, choose an energy-like function:

```math
V(x)>0,\qquad V(0)=0
```

Then inspect:

```math
\dot{V}(x)=\frac{\partial V}{\partial x} f(x)
```

Typical conclusions:

- `V` positive definite and `\dot{V}` negative semidefinite gives stability.
- `V` positive definite and `\dot{V}` negative definite gives asymptotic stability.
- Quadratic bounds on `V` and `\dot{V}` can give exponential stability.

## Class K Functions and Decrescence

Class `K` functions are increasing functions used to bound Lyapunov functions:

```math
\alpha_1(\|x\|) \le V(x,t) \le \alpha_2(\|x\|)
```

These bounds let us translate energy decrease into state convergence.

## Barbalat's Lemma

Barbalat's lemma is repeatedly used later in robot adaptive/robust control:

If a signal has a finite integral and its derivative is bounded or uniformly continuous, then the signal converges to zero.

In controller proofs, the usual pattern is:

1. Show `V(t)` is bounded and nonincreasing.
2. Show a tracking-error signal is square integrable.
3. Show the derivative of that signal is bounded.
4. Conclude the signal goes to zero.

## Passivity

Passivity is introduced as an input-output energy property. Robot dynamics have useful passivity-like structure, especially with the inertia matrix and Coriolis matrix. This becomes important for passivity-based adaptive control.

## Why This Session Matters for the Project

Every serious robot controller in the project needs a stability explanation:

- Computed torque can produce linear error dynamics, then stability follows from linear second-order systems.
- PD plus gravity uses a Lyapunov function involving kinetic energy and potential-like error energy.
- Robust, sliding, RISE, and adaptive controllers use Lyapunov inequalities to prove boundedness or convergence.

