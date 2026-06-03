# Session 10 - Flexible/Elastic Joints and Wheeled Mobile Robots

Sources:
- Lecture note: `Lex/lecture10.pdf`
- Reference: Ref.2, PDF pages 179 onward for elastic joints
- Reference location for wheeled mobile robots was marked `/?`; this note follows the lecture notes and Ref.2 mobile-robot chapter area

## Flexible or Elastic Joint Manipulators

Rigid-joint robot models assume motor and link coordinates are the same. Elastic joint models separate:

- Link coordinates `q`
- Motor coordinates or actuator-side coordinates `\theta`

A typical elastic joint relation includes spring torque:

```math
K(\theta-q)
```

The dynamics become higher order because motor inertia and link inertia both matter.

## Control Challenge

Elastic joints create internal dynamics. A controller that stabilizes rigid-joint dynamics may not stabilize the elastic system unless the motor-link coupling is considered.

Typical objectives:

- Link-position regulation or tracking.
- Motor torque design.
- Damping of elastic oscillations.

## Wheeled Mobile Robots

Mobile robots often have nonholonomic constraints. These are velocity constraints that cannot be integrated into pure position constraints.

For a unicycle-like robot:

```math
\dot{x}=v\cos\theta,\qquad
\dot{y}=v\sin\theta,\qquad
\dot{\theta}=\omega
```

The robot cannot directly move sideways:

```math
-\sin\theta\,\dot{x}+\cos\theta\,\dot{y}=0
```

This is the nonholonomic constraint.

## Tracking Error Coordinates

For mobile robots, the tracking error is often transformed into the robot body frame. This makes the control law depend on longitudinal, lateral, and heading errors.

The lecture notes work through regulation/tracking designs using transformed errors and Lyapunov-style arguments.

## Why This Session Matters for the Project

This session is probably less central to the 3-DOF planar manipulator project, but it is important for the final exam survey. It broadens the idea that robot control depends strongly on the mechanical model and constraints.

