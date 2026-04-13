# MINCO Terminal Mapping Notes

Current state-to-state and tracking tasks use fixed-boundary MINCO:

```text
x = [tau, xi]
head_state, tail_state = fixed task-layer boundary data
```

The optimizer therefore only optimizes segment times and inner points. This is
still the correct default for the existing tracking pipeline: the task/runtime
layer chooses a tracking anchor or terminal reference, freezes it as the MINCO
tail boundary, and then solves a fixed-boundary position trajectory.

Perching in the paper sense needs a different terminal model:

```text
tail_state = F(T, Xi(T), surface_frame(T), nu, tau_f, ...)
```

The MINCO kernel now exposes:

```text
dJ / d head_state
dJ / d tail_state
```

through the adjoint solve. This repository now has a lightweight
`PerchingTerminalMapping` that implements a constant-velocity moving-platform
model with fixed surface frame over one planning cycle:

```text
tail_pos(T) = Xi(T) + l * z_s
tail_vel(T) = Xi_dot + nu_x * x_s + nu_y * y_s - v_plus * z_s
tail_acc(T) = (tau_m + tau_r sin(tau_f)) * z_s + g
tail_jerk(T) = 0    (when the backend uses S=4 / minimum snap)
```

The generic terminal-mapping interface applies:

```text
d tail_state / d nu
d tail_state / d tau_f
d tail_state / d T
```

and any target-prediction / surface-frame derivatives. In particular, the
fixed-tail time gradient must be augmented by:

```text
(d tail_state / d T)^T * (dJ / d tail_state)
```

before the time map converts physical time gradients back to unconstrained
`tau` gradients.

`FixedTerminalMapping` is still a no-op mapping for the current state-to-state,
tracking, and corridor/ESDF paths. Perching now uses the same generic MINCO
optimizer path with an enabled terminal mapping instead of freezing the moving
platform's terminal state once at the task layer.
