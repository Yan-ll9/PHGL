# Port-Hamiltonian Control with Observability-Constrained Learning for Stratospheric Airships

This repository provides a **demonstration codebase** accompanying the paper:

> **Port-Hamiltonian Control with Observability-Constrained Learning for Stratospheric Airships in Low-Density Environments**
> Lilin Yan, Pengshuo Li, Hongwei Zhao
> *IEEE Transactions on Control Systems Technology*, 2026 (submitted)

## Overview

Stratospheric airships (18–22 km altitude) operate in low-density environments where the air density is only ~7% of sea level. This dramatically reduces the added-mass-to-hull-mass ratio (from ~180% at sea level to ~13%), creating **observability degradation** challenges for parameter estimation. Combined with 70 K day-night temperature swings and 10–25 m/s time-varying winds, this poses a difficult control problem.

This work proposes:

- **Port-Hamiltonian (PH) dynamics modeling** with attitude-dependent added mass
- **IDA-PBC-inspired energy-shaping control** (potential shaping + damping injection)
- **Observability-Constrained EKF (OC-EKF)** that projects parameter updates onto the observable subspace
- **Sparse Gaussian Process wind inference** for real-time disturbance compensation
- **Symplectic integration** (Störmer–Verlet) for long-horizon numerical stability

## Repository Structure

```
├── Phgl_part1.m      # Main script: physics init, PHGL & ABC controllers,
│                      # Monte Carlo simulation, statistical analysis, visualization
├── Phgl_part2.m      # Extended comparison: MRAC, MPC baselines,
│                      # ablation study, failure mode analysis, full diagnostics
└── README.md
```

## Quick Start

### Requirements

- **MATLAB** R2020b or later (Statistics and Machine Learning Toolbox recommended for `ttest2`)

### Running

1. Run `Phgl_part1.m` first. This executes 30 Monte Carlo runs comparing PHGL vs. ABC (adaptive backstepping) and saves results to `./results_v3/`.

2. Run `Phgl_part2.m` after Part 1 completes. This loads the Part 1 results and adds MRAC and MPC comparisons, ablation study, and failure mode analysis.

```matlab
% In MATLAB
run('Phgl_part1.m')   % ~10-30 min depending on hardware
run('Phgl_part2.m')   % ~15-40 min
```

### Outputs

Results are saved to `./results_v3/`, including:

- `MC_Results_Complete.mat` — Monte Carlo raw data
- `Fig1_*.png/fig` — Box plots (RMSE & control effort)
- `Fig2_*.png/fig` — Trajectory comparisons
- `Fig3_*.png/fig` — Error time series
- `Fig4_*.png/fig` — Ablation study bar chart
- `Fig5_*.png/fig` — Failure mode analysis
- `Fig6_*.png/fig` — PHGL diagnostic plots (parameter convergence, observability, GP wind, control input)

## Important Note

**This is a simplified demonstration**, not a reproduction of the full results reported in the paper. Key differences include:

- **Reduced Monte Carlo runs** (30 vs. 100 in the paper) for faster execution
- **Simplified GP implementation** (direct kernel computation instead of optimized GPyTorch/CUDA pipeline)
- **No hardware-in-loop (HIL) component** — the paper includes Jetson Xavier NX real-time validation
- **Ablation study** uses a challenging scenario design to expose module contributions at a smaller code scale; absolute RMSE values differ from the paper's Table III
- **SAC (deep RL) baseline** from the paper is not included; MRAC is used as an additional baseline instead

The paper's industrial-scale results were obtained with a more comprehensive simulation environment, larger-scale experiments, and hardware integration not included here.

## Method Summary

| Module | Description |
|---|---|
| PH Dynamics | Planar motion model with attitude-dependent added mass `M(ψ) = M_RB + R(ψ)M_A R(ψ)ᵀ` |
| IDA-PBC Control | Energy-shaping: `u = −Kₑeq − KdM⁻¹ep + feedforward − d̂_wind + f_comp` |
| OC-EKF | SVD of observability Gramian → project updates onto observable subspace when `α_obs > α_PE` |
| Sparse GP | Spatio-temporal SE kernel, sliding window, wind residual inference |
| Symplectic Integrator | Störmer–Verlet splitting, energy drift < 3.2% over 24 h |

## Citation

If you find this code useful, please cite:

```bibtex
@article{yan2026porthamiltonian,
  title={Port-{Hamiltonian} Control with Observability-Constrained Learning for Stratospheric Airships in Low-Density Environments},
  author={Yan, Lilin and Li, Pengshuo and Zhao, Hongwei},
  journal={IEEE Transactions on Control Systems Technology},
  year={2026},
  note={submitted}
}
```

## License

This project is released under the [MIT License](LICENSE).

## Contact

- Pengshuo Li — pengshuoli55@gmail.com
- Lilin Yan — yanlilin@mail.nwpu.edu.cn
- Hongwei Zhao (corresponding author) — hongvi_zhao@126.com
