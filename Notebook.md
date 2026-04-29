# Introduction to Control
## Basic definitions
* Most robots used in industrial applications are simple positioning / handling devices.
* Therefore it is necessary for a robot to be able to “control” its movements and the forces it applies to its environment.
* In order to properly “control”, requires the ==mathematical model== and some sort of intelligence to act on the model.
	* The mathematical model of the robot can be obtained from physical laws governing its movement
	* Intelligence, on the other hand requires sensory capabilities and means for acting/reacting the sensed variables
* The actions and reactions of the robotic device are the result of control design

## Linear state-variable systems
* Most physical systems (like robots) are described by differential (or difference) equations.
* These equations (usually obtained via physical laws) provide the starting point for the analysis and control of systems.
* For sake of presentation we limit ourselves to systems described by ordinary differential equations which will lead to a finite-dimensional state space ( Partial differential equations, leading to infinite-dimensional systems, are used to study flexible structures, like flexible-link robotic arms)
### Linear systems
A continuous-time system is said to be linear if it obeys the principle of **superposition**.

---
#### Superposition:
Additivity AND Homogeneity
For any constants a₁ and a₂:

a₁·x₁(t) + a₂·x₂(t) → a₁·y₁(t) + a₂·y₂(t)

---
A linear SISO, continuous-time, time invariant system is described by linear, scalar, constant-coefficient ordinary differential equations such as:

![eq](./images/linsys.png)

where:  
$a_i, b_i \; (i = 1, \dots, n)$ are scalar constants
$u(t)$ is the input
$y(t)$ is output
### State-space realization
- General form
	$\dot{x}(t) = A x(t) + B u(t)$
	
	$y(t) = C x(t) + D u(t)$
	
		- x(t): state vector (internal system variables)
		- u(t): input
		- y(t): output
		- A: system matrix
		- B: input matrix
		- C: output matrix
		- D: feedthrough matrix

### Transfer function
...!!
### Nonlinear state-variable systems
...!!
### Vector spaces, norms and inner product
...!!
kkk



