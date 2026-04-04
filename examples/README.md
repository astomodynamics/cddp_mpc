# Examples

This directory contains helper scripts for PX4 simulation, validation, and tuning with `cddp_mpc`.

## Quick Start Scripts

### 1. Test PX4 Connection

```bash
python examples/test_px4_connection.py
```

### 2. List PX4 Topics

```bash
python examples/list_px4_topics.py
```

### 3. Validate Lift-Off, Hover, and Optional Landing

```bash
python examples/validate_takeoff_hover.py \
  --target-z -3.0 \
  --settle-tolerance 0.3 \
  --hold-duration-sec 20 \
  --timeout-sec 90 \
  --min-climb-m 0.6 \
  --climb-timeout-sec 25 \
  --max-downward-speed-mps 2.5
```

To require landing and disarm near the start altitude:

```bash
python examples/validate_takeoff_hover.py \
  --target-z -3.0 \
  --settle-tolerance 0.3 \
  --hold-duration-sec 20 \
  --require-landing \
  --landing-tolerance-m 0.3 \
  --timeout-sec 120
```

### 4. Calibrate Hover Thrust

```bash
python examples/calibrate_hover_mass.py \
  --duration-sec 25 \
  --min-samples 30 \
  --vz-threshold-mps 0.2 \
  --max-thrust-n 20.0
```

Use the printed `recommended_hover_thrust_n` value in `config/mpc_sitl.yaml` or `config/mpc_hardware.yaml`.
