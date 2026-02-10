# PHGL: Port-Hamiltonian Control with Observability-Constrained Learning for Stratospheric Airships

Demo code for the paper *"Port-Hamiltonian Control with Observability-Constrained Learning for Stratospheric Airships in Low-Density Environments"* (under review).

## Overview

An adaptive control framework for stratospheric airship trajectory tracking, integrating Port-Hamiltonian energy-shaping control, observability-constrained parameter estimation, and sparse Gaussian process wind inference.

## Requirements

- MATLAB R2020b or later

## Usage

```matlab
run('Phgl_part1.m')   % Monte Carlo: PHGL vs ABC baseline
run('Phgl_part2.m')   % Extended comparison: MRAC, MPC, ablation, failure modes
```

Run Part 1 first. Part 2 loads Part 1 results automatically. Outputs (figures and data) are saved to `./results_v3/`.

## Note

This is a **simplified demonstration** for algorithm verification, not a full reproduction of the paper's results. Details will be updated upon publication.



## License

[MIT License](LICENSE)
