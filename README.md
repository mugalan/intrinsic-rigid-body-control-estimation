# Intrinsic Almost Global rigid-body PID Control

This is a compilation of a set of interactive notes and supplementary material for teaching and learning Mechanics.

**D. H. S. Maithripala, PhD.**  
smaithri@eng.pdn.ac.lk  
https://eng.pdn.ac.lk/ME/People/FacultyProfiles.php?id=6  
https://orcid.org/0000-0002-8200-2696


---

## 🌀 Geometric PID Controller on (\mathbb{R}^3 \times SO(3))

This repository implements a **geometric PID controller** for fully actuated rigid body systems evolving on the nonlinear configuration space (\mathbb{R}^3 \times SO(3)). The controller operates in the **spatial momentum space**, enabling coordinate-free tracking of position and orientation trajectories with provable convergence properties.

### ✒️ Controller Description

The control law is derived by lifting classical PID control to the Lie group (\mathbb{R}^3 \times SO(3)), ensuring compatibility with the group structure of rigid body motion. The key components include:

* **Right-invariant configuration error**: Defined as ((o_e, R_e) = (o_r - o, R_r R^T)), capturing position and orientation discrepancies between the current and reference trajectories.

* **Momentum-based error dynamics**: The controller is expressed in terms of linear and angular momentum errors ((p_e, \pi_e)), which evolve linearly under the influence of control inputs.

* **Geometric integral terms**: Integral errors are defined on the configuration space itself, preserving geometric consistency without resorting to local coordinates or quaternions.

* **Quadratic-like error function**: A scalar Lyapunov candidate function is constructed using a trace-based term on (SO(3)), whose gradient defines the proportional action in a group-consistent way.

### 🚀 Significance

This controller achieves **global coordinate-free tracking** of desired rigid body trajectories, avoiding singularities and ambiguities associated with parameterizations like Euler angles or quaternions.

By working directly in the **momentum space** and respecting the geometric structure of the configuration space, the design naturally accommodates:

* **Feedforward compensation** of desired momentum trajectories
* **Proportional-derivative-integral feedback** in a globally consistent way
* **Stability and convergence** guarantees under standard Lyapunov analysis on manifolds

### 🌍 Almost-Global Convergence

Due to the topological properties of the rotation group (SO(3)), **global asymptotic stabilization** is impossible using continuous state-feedback. However, this controller achieves **almost-global convergence**, meaning:

A key feature of this controller is that it leverages the **linearity of the rigid body momentum equations**. By formulating the dynamics in terms of spatial linear and angular momentum, the control design reduces to applying **standard PID structure** on a linear system — despite the nonlinear configuration space. This dramatically simplifies the controller implementation while preserving geometric correctness.


* The desired configuration is **asymptotically stable** from almost all initial conditions.
* The only exceptions are a measure-zero set of initial attitudes corresponding to 180° rotations around principal axes — these are **unstable saddle points** of the error function.

This is the best possible result achievable with smooth feedback on (SO(3)), making the controller **theoretically optimal** under the constraints of continuous control.

---


## 🔖 License

MIT © DHS Maithripala
