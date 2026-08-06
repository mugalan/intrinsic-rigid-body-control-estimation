# Intrinsic PID on Lie Groups

## Overview
--------
This notebook (Intrinsic_PID_on_Lie_Groups.ipynb) presents a concise, implementation-focused introduction to intrinsic PID control for mechanical systems evolving on Lie groups, with specialisation to rigid-body motion on SE(3). It is prepared by D. H. S. Maithripala and accompanies theoretical work on geometric PID control and almost-global tracking on Lie groups. The notebook covers the mathematical preliminaries, derives an intrinsic (coordinate-free) PID controller that operates in momentum space, and verifies the controller with numerical simulations (animated rigid-body examples).

## Highlights
----------
- Presents Lie-group preliminaries and momentum-space formulations needed for coordinate-free control.
- Derives an intrinsic AGAS (almost-globally asymptotically stable) PID controller defined on Lie groups.
- Specializes the general controller to rigid bodies on $SE(3)$ (translational + rotational motion).
- Implements feedforward + PID control with correct Lie-group error definitions (right/left invariant errors).
- Provides two simulation examples for verification:
  - Example 1: Trajectory tracking for a fully-actuated cube pivoted at a vertex.
  - Example 2: Vertical stabilization for an under-actuated cube (actuation projected onto a subspace).
- Uses a lightweight simulation helper package (git repo) for dynamics and animation.

## Mathematical background (brief)
-------------------------------
- Configuration space: general Lie group $G$ with Lie algebra $\mathfrak{g}$ and dual $\mathfrak{g}^\star$; for rigid bodies, $G = SE(3)$.
- Inertia: body inertia operator $\mathbb{I} : \mathfrak{g}  \to \mathfrak{g}^\star$ and $g$-dependent inertia $I_g = {Ad}_{g^{-1}}^\star \mathbb{I} {Ad}_g$.
- Momentum and angular velocity:
  - $\pi \in \mathfrak{g}^\star$ (spatial momentum), $\Omega \in \mathfrak{g}$ (body angular velocity), $\omega = \mathrm{Ad}_g \Omega$ (spatial angular velocity).
  - $\pi$ and $\omega$ related by $\pi = I_{g^{-1}} \omega$.
- Tracking error: right-invariant error $e = g_r g^{-1}$; associated velocity error $\omega_e = \omega_r − \mathrm{Ad}_e \omega$.
- Momentum error $\pi_e$ constructed to live in the momentum space and to be consistent with the kinetic energy metric.

## Intrinsic PID controller (concept)
----------------------------------

- Introduces a polar Morse error function $f_e: G \to \mathbb{R}$ with unique minimum at the identity; $d f_e$ defines the proportional term ($\pi_e$).

- Defines an integral momentum-like state $\pi_I$ with dynamics
  $$\dot{\pi}_I = \pi_e.$$

- The intrinsic PID control law in momentum space (feedforward + feedback) takes the form
  $$f^u = \text{feedforward terms} - k_p \,\pi_e - k_d \,\dot{\pi}_e - k_I \,\pi_I,$$
  (presented carefully in the notebook with $\mathrm{Ad}$ / $\mathrm{Ad}^*$ transformations so the law is coordinate-free).

- Controlled error dynamics reduce to a standard-looking PID system in the Lie-group setting:

  $$\dot{e} = \omega_e \cdot e$$,
  
  $$\dot{\pi}_I = \pi_e$$,
  
  $$\ddot{\pi}_e = -k_p\,\pi_e - k_d\,\dot{\pi}_e - k_I\,\pi_I$$,
  
  (interpreted in the appropriate Lie-algebra / dual spaces).


## Rigid body specialization (SE(3))
---------------------------------
LaTeX-formatted version:

- Rigid-body states: $o\in\mathbb{R}^3$ (position), $R\in\mathrm{SO}(3)$ (orientation), linear momentum $p$, angular momentum $\pi$.

- Equations of motion (spatial representation):

  $$\dot{o}=\frac{1}{M}p$$,
  
  $$\dot{R}=\widehat{\omega}\,R$$,
  
  $$\dot{p}=f^{e}+f^{u}$$,
  
  $$\dot{\pi}=\tau^{e}+\tau^{u}$$,
  
  where $\omega=(\mathbb{I}_R)^{-1}\pi$ and $\mathbb{I}_R=R\mathbb{I}R^{T}$ (and $\widehat{\omega}$ is the skew matrix of $\omega$).

- Configuration error (right-invariant form):

  $$(o_e,R_e)=(o_r-o,R_r R^{T}).$$

- Rotational error vector $e_R$ defined from the error function

  $$f(o_e,R_e)=\tfrac{1}{2}\Big(o_e^{T}o_e+\mathrm{trace}\big(K(I-R_e)\big)\Big),$$
  
  with $e_R$ obtained from the matrix derivative (e.g. $\widehat{e}_R=\frac{1}{2}(R_e K-KR_e^T)$).

- Feedforward + PID controllers:
  - Translational:

    $$f^u=M\ddot{o}_r$$
    
    $$f^u=M\ddot{o}_r + k_{P_o}e_o + k_{D_o}p_e + k_{I_o}e_{I_o}.$$

  - Rotational:

    $$\tau^{u}=\big(R\dot{\Pi}_r+\omega\times R_e^{T}\pi_r\big)-\tau^{e}+k_{P_R}\,e_R + k_{D_R}\,\pi_e + k_{I_R}\,e_{I_R}.$$

- Under-actuated case: control is projected onto the actuation subspace using a projection matrix; the example uses

  $$P=-\widehat{e}_3^{2}$$
  
  (projecting the torque control onto the allowed actuation axes).

## Implementation & simulation (practical)
--------------------------------------
- The notebook uses numpy, scipy, sympy and plotly for computations and visualization.
- A simulation helper package (rigid-body-sim) is installed from GitHub in the notebook via:
  pip install --quiet "git+https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view.git#egg=rigid-body-sim"
- The simulation API (sims.RigidBodySim) provides:
  - set_external_force_model(fn)
  - set_actuator(fn)
  - simulating_a_cube(dt, T, cubeDimensions, parameters, ICs)
  - animated_cube_flat_shading(simulation_data, title)
- Key functions implemented in the notebook:
  - externalForceModel(...) — defines gravity/other external forces and torques (heavy-top style).
  - actuator(...) — bridges controller outputs to simulation actuators.
  - referenceConfig(...) — returns reference R_r, π_r, ˙π_r for feedforward.
  - controller(...) — computes tau_u and f_u (rotational and translational control), and error terms.
  - controller_dynamics(...) — integrator for controller internal state (integral term).
- Simulation cells demonstrate:
  - Example 1: fully-actuated cube simulation using dt = 0.01, T = 50s, animations produced.
  - Example 2: under-actuated cube using dt = 0.2, T = 20s, animations produced after projecting the control torques.

## Examples included
-----------------
- Example #1: Trajectory tracking of a fully actuated cube pivoted at a vertex. Demonstrates tracking and animated visualization of the cube following a prescribed rotational reference. Uses a full torque actuator model.
- Example #2: Vertical stabilization of an under-actuated cube pivoted at a vertex. Demonstrates projecting the control action onto allowed actuation axes (underactuation) to achieve stabilization.

## How to run
----------
1. Install Python 3.8+ and standard scientific packages (numpy, scipy, sympy, plotly).
2. From the notebook, install the helper package:
   pip install --quiet "git+https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view.git#egg=rigid-body-sim"
3. Open Intrinsic_PID_on_Lie_Groups.ipynb in Jupyter / JupyterLab or on Colab (the notebook includes a Colab badge link).
4. Execute cells top-to-bottom. Simulations produce interactive Plotly animations and figures.

## Key references
--------------
- D. H. S. Maithripala, Jordan M. Berg, "An intrinsic PID controller for mechanical systems on Lie groups", Automatica, 2015.
- Rama Seshan Chandrasekaran, Ravi N. Banavar, Arun D. Mahindrakar, D. H. S. Maithripala, "Geometric PID controller for stabilization of nonholonomic mechanical systems on Lie groups", Automatica, 2024.
- D. H. S. Maithripala, J. M. Berg and W. P. Dayawansa, "Almost-global tracking of simple mechanical systems on a general class of Lie Groups," IEEE TAC.

## Notes & suggestions
-------------------
- The notebook emphasizes coordinate-free controller design in momentum space; this makes the controller naturally compatible with different coordinate representations (e.g., quaternions, rotation matrices).
- The error function for rotation includes a diagonal weighting K; you may tune K and PID gains to test performance and robustness.
- Example controllers include small feedforward terms and projection for under-actuation; the notebook is a good starting point for adapting controllers to other rigid-body geometries and actuation constraints.

## License / attribution
---------------------
- Keep the original attribution to the notebook author (D. H. S. Maithripala) and cite the referenced papers when using the algorithms or results in publications.

