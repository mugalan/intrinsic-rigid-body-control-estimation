# Intrinsic Rigid-Body Control and Estimation on Lie Groups

This repository develops geometric methods for the **control and estimation of rigid-body systems directly on Lie groups**, with particular emphasis on $SO(3)$ and $SE(3)$.

The material is organized around two complementary themes:

1. **Intrinsic control** — coordinate-free PID control for mechanical systems evolving on Lie groups.
2. **Intrinsic estimation** — discrete Kalman filtering formulated using invariant errors and Lie-group geometry.

Both parts begin with general Lie-group formulations and progressively specialize the theory to rigid-body applications, accompanied by executable Python simulations and numerical examples.

---

## 1. Intrinsic PID Control on Lie Groups

The control component develops an intrinsic PID framework for mechanical systems whose configuration evolves on a Lie group.

It includes:

- Lie-group and momentum-space formulations of mechanical systems;
- coordinate-free proportional, derivative, and integral error constructions;
- almost-global asymptotic tracking;
- feedforward and PID control in momentum space;
- specialization to rigid-body dynamics on $SE(3)$;
- fully actuated rigid-body trajectory tracking;
- under-actuated stabilization using projected control; and
- numerical simulations and interactive rigid-body animations.

The notebook connects the general geometric PID theory to implementable rigid-body controllers without introducing local attitude coordinates.

**[Read the Intrinsic PID documentation](./rigid-body-control/README.md)**

---

## 2. Intrinsic Discrete Kalman Filtering on Lie Groups

The estimation component develops discrete-time Kalman filtering directly on Lie groups, beginning with the ordinary linear-Gaussian Kalman filter and extending its probabilistic structure to nonlinear configuration spaces.

A central viewpoint is to treat the predicted group estimate as a stochastic random variable,

$$
\widetilde g_k^-
=
\widetilde g_{k-1}
\exp\left(
\Delta T(\zeta_{k-1}+w_{k-1})
\right),
$$

in direct analogy with Euclidean Kalman prediction.

Using compatible invariant errors, the deterministic input shared by the true and estimated systems cancels from the first-order error propagation. This produces the particularly simple transition

$$
A_{k-1}=I,
$$

with the Lie-group geometry and process uncertainty entering through

$$
G_{k-1}
=
-\Delta T\,
\operatorname{Ad}_{\widetilde g_{k-1}}
\Phi(-\Delta T\zeta_{k-1})^{-1}.
$$

The notebook includes:

- the discrete Kalman filter on $\mathbb R^n$;
- invariant errors and discrete pre-observers;
- derivation of the intrinsic discrete EKF;
- continuous-time intrinsic EKF comparison;
- attitude estimation on $SO(3)$;
- gyroscope-bias estimation and IMU fusion;
- rigid-body estimation and landmark-aided localization on $SE(3)$; and
- IMU--GNSS navigation with gyroscope and accelerometer biases.

Executable examples connect the general theory to practical rigid-body estimation and sensor fusion.

**[Read the Intrinsic DEKF documentation](./intrinsic-DEKF/README.md)**

---

## Repository perspective

The two components address complementary sides of the same geometric problem:

$$
\boxed{
\text{Lie-group mechanics}
\quad\longrightarrow\quad
\begin{cases}
\text{intrinsic control},\\
\text{intrinsic estimation}.
\end{cases}
}
$$

Rather than introducing local coordinates for rotations and rigid-body poses, both developments work directly with the underlying geometric structure. The objective is to provide a coherent route from Lie-group mechanics and invariant errors to implementable algorithms for rigid-body **control, state estimation, and sensor fusion**.

## Implementation

The notebooks use Python with NumPy, SciPy, SymPy, and Plotly. Rigid-body simulation and visualization utilities are provided by the companion package:

[`classical-mechanics-from-a-geometric-point-of-view`](https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view)

Each subdirectory contains its own README with mathematical details, examples, installation instructions, and notebook-specific documentation.

## References

The notebooks build on work in geometric mechanics, intrinsic PID control, invariant observers, and invariant Kalman filtering. Detailed references are provided in the corresponding notebooks and subproject READMEs.

## License

This repository is licensed under the [MIT License](./LICENSE).
