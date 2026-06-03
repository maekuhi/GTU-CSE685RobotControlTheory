# Reference Check Log

Purpose: record how the handwritten lecture notes were checked against the two course references.

References named in `Table of Contents.pdf`:

- Ref.1: `Boox/Robot Manipulator Control Theory and Practice.pdf`
- Ref.2: `Boox/Carlos-Canudas-de-Wit-Bruno-Siciliano-Georges-Bastin-Theory-of-Robot-Control.pdf`

## Working Rule

The handwritten lecture notes are treated as the primary source because they reflect what was taught in class. The reference books are used to:

- confirm definitions and standard controller forms,
- clarify notation when handwriting is ambiguous,
- fill small conceptual gaps,
- detect likely typos or inconsistencies.

When a reference appears inconsistent with the lecture notes or with standard robot-control identities, the lecture-note version is retained and the point is marked as a caution.

## Initial Checks

### Lecture 01

Checked against Ref.1 control-theory pages listed in the table of contents. The state-space, transfer-function, norm, matrix-property, and BIBO-stability concepts match the reference structure. The full note keeps lecture notation and examples.

### Lecture 02

Checked against Ref.1 stability/Lyapunov material. The stability definitions, Lyapunov derivative form, Barbalat/LaSalle usage, passivity idea, and LTI Lyapunov equation are consistent with the listed reference pages.

### Lecture 03

Checked against Ref.2 kinematics/dynamics material and Ref.1 Lagrange/robot-equation properties. The Lagrange dynamics and standard robot equation are consistent with the references. Robot dynamic properties are recorded in the standard form used later for control:

```math
M(q)=M^T(q)>0
```

```math
\dot{M}(q)-2C(q,\dot{q}) \text{ is skew-symmetric}
```

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)+F(\dot{q})=\tau
```

### Caution About Ref.1

The user noted that Ref.1 contains some mistakes. Therefore, future checks will not copy formulas mechanically. Each robot-control formula will be checked against:

1. the handwritten lecture sequence,
2. the standard manipulator equation,
3. dimensional consistency,
4. Lyapunov cancellation requirements,
5. the corresponding reference section if it is reliable for that concept.

## Completed Lecture File Structure

The full markdown notes are intentionally one file per lecture:

- `Lecture01_Full_Control_Theory_Preliminaries.md`
- `Lecture02_Full_Stability_and_Lyapunov.md`
- `Lecture03_Full_Robot_Kinematics_and_Dynamics.md`
- `Lecture04_Full_Feedback_Linearization_and_Computed_Torque.md`
- `Lecture05_Full_Filtered_Tracking_Error.md`
- `Lecture06_Full_Robust_Control_VSC_RISE.md`
- `Lecture07_Full_Adaptive_Computed_Torque_Inertia.md`
- `Lecture08_Full_Passivity_PE_Composite_Adaptive.md`
- `Lecture09_Full_Task_Space_Control.md`
- `Lecture10_Full_Flexible_Joints_and_Mobile_Robots.md`
- `Lecture11_Full_Unified_Regulation_Tracking.md`

The Lecture 06 supplementary RISE material is merged into the Lecture 06 file, per user instruction.
