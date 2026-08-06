# Intrinsic Rigid Body Control & Estimation
Comprehensive notebook and supporting code demonstrating Kalman filters on Euclidean spaces and intrinsic Extended Kalman Filters (EKF) on Lie groups, with simulations for constant-velocity models and intrinsic discrete/continuous-time observers.

This repository collects teaching and research-oriented code (notebooks and helper modules) for intrinsic filtering and observer design on manifolds, with emphasis on:
- Standard Kalman filtering on R^n (scalar, 1-D, and 2-D examples).
- Pre-observers and time-invariant error dynamics on Lie groups.
- Intrinsic Extended Kalman Filter (IEKF) derivation for systems evolving on Lie groups.
- Simulation pipelines and visualization of filtering performance and measurement likelihoods.

[Notebook:](./intrinsic-DEKF/RigidBodyIntinsicEKF_DHSM.ipynb)

## Contents
- Short description of the mathematical models used (linear-Gaussian systems, CV models).
- Worked 1-D and 2-D constant-velocity examples with simulation, filtering, and visualization.
- Theory of pre-observers on Lie groups (left-/right-invariant outputs) and how to build filters with autonomous error dynamics.
- Continuous-time intrinsic EKF derivation and discussion about log-error dynamics and linearization.
- Simulation code using a small helper package exposed as `sims` (installed in the notebook).

## Key learning goals
- Understand the discrete-time Kalman filter in R^n and its error dynamics.
- See practical simulations for scalar and 2D CV models (process/measurement noise modeling).
- Learn how to construct observers on Lie groups that make the estimation error autonomous (left/right invariance).
- Learn the intrinsic EKF derivation and how to apply it to systems with Lie-group structure.
- Reproduce visualizations and experiment with parameters (dt, process noise, measurement noise).

## Quick start (Colab)
1. Open the notebook directly in Google Colab:
   - https://colab.research.google.com/github/mugalan/intrinsic-rigid-body-control-estimation/blob/main/intrinsic-DEKF/RigidBodyIntinsicEKF_DHSM.ipynb
2. Run the notebook cells. The notebook installs an auxiliary package via:
   ```
   !pip install --quiet "git+https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view.git#egg=rigid-body-sim"
   ```
   This provides `sims` used for simulation, plotting, and helper classes.

## Quick start (local)
1. Clone the repo:
   ```
   git clone https://github.com/mugalan/intrinsic-rigid-body-control-estimation.git
   cd intrinsic-rigid-body-control-estimation
   ```
2. Create and activate a virtual environment (recommended):
   ```
   python -m venv .venv
   source .venv/bin/activate      # Linux / macOS
   .venv\Scripts\activate         # Windows (PowerShell/CMD)
   ```
3. Install core Python dependencies:
   ```
   pip install numpy scipy pandas sympy plotly ipython jupyterlab
   pip install "git+https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view.git#egg=rigid-body-sim"
   ```
   If you prefer pinned versions, create a requirements.txt and pin as needed.
4. Launch JupyterLab or Jupyter Notebook and open:
   ```
   intrinsic-DEKF/RigidBodyIntinsicEKF_DHSM.ipynb
   jupyter lab
   ```
5. Run the notebook cells interactively.

Dependencies
- Python 3.8+ (tested with Python 3.9+)
- numpy
- scipy
- pandas
- sympy
- plotly (for visualization)
- IPython (display helpers)
- A helper package installed from GitHub:
  - git+https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view.git#egg=rigid-body-sim
    - provides the `sims` module used to construct LinearGaussianSystemSyms objects and plotting utilities.

## Notebook structure and section summary
- Imports and setup
  - Load scientific and plotting libraries; install and import `sims` helper package.
- The Kalman Filter on R^n
  - Formal statement of the linear-Gaussian model, prediction and update equations, and error dynamics.
- 1-D Example
  - Scalar constant-velocity system, closed-form Kalman filter recursion, and an animation function to visualize measurement likelihoods (Gaussian measurement distributions).
  - Helper: `make_cv1d` to build a scalar CV model with process/measurement noise.
- Simulation 2D-example
  - Two-dimensional CV model (states: p_x, p_y, v_x, v_y).
  - Construction of discrete-time matrices (A, G) and process covariance Q for random acceleration modeling.
  - Helper: `make_cv2d` which returns a `sims.LinearGaussianSystemSyms` instance configured for 2D CV experiments.
  - Scripts for simulating measurements and running standard KF with plotting utilities.
- Discrete-Time Pre-Observers on Lie Groups with Time-Invariant Error Dynamics
  - Theory of left- and right-invariant outputs and observer construction with innovations designed to yield autonomous error dynamics.
- Intrinsic Extended Kalman Filter on Lie Groups
  - Continuous-time intrinsic EKF derivation: state and measurement models on Lie groups, definition of log-error, linearization, Riccati equation, and conditions for convergence (uniform observability).
  - Discussion of exact log-error dynamics and higher-order residual terms from the Lie algebra exponential maps.

## Examples provided in the notebook
- Simulated scalar KF: `sys.animate_measurement_gaussians_scalar(...)`
- Simulated 2D CV filtering pipeline:
  - `sys = make_cv2d(...)`
  - `X2, Y2 = sys.simulate(T=300)`
  - `M, Yhat = sys.filter_with_kf_and_plot(...)` — runs KF and plots estimates vs measurements.
- The notebook contains runnable code cells to reproduce simulations and figures. Tweak dt, q, r, and P0 to observe filter behavior.

## Reproducibility and tips
- Use the notebook-provided RNG `seed` argument for reproducible simulations.
- If running headless (no browser), replace interactive Plotly display with saving HTML files:
  - Many helper functions accept a `save_html_path` argument to write interactive plots to disk.
- If you encounter missing `sims` module errors, ensure the pip install command ran successfully, or install locally using:
  ```
  pip install -e path/to/local/classical-mechanics-from-a-geometric-point-of-view
  ```
  or reinstall using the git+https URL.



Citation and references
- The repository includes concise derivations and references to standard multivariate Gaussian conditioning and EKF theory. For implementation details and further reading:
  - Standard textbooks on estimation and Kalman filtering (e.g., Simon, "Optimal State Estimation"; Maybeck, "Stochastic Models and Estimation").
  - Research literature on invariant filtering and intrinsic EKFs on Lie groups (e.g., Barrau & Bonnabel, 2017 papers on invariant filters).
  - See the in-notebook references and the `classical-mechanics-from-a-geometric-point-of-view` repository for geometric mechanics background.

Contributing
- Pull requests are welcome. Please:
  - Add reproducible notebooks or scripts, including seeds and explicit parameters.
  - Keep notebooks reasonably sized; move long derivations to markdown or separate .md files when possible.
  - Add tests or small scripts to reproduce key figures.

License
- MIT LICENSE

Contact
- Repository owner: mugalan@gmail.com
- For questions related to the notebooks or reproducing results, open an issue in this repository with the notebook path and error details.

