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

through the adjoint solve. A future perching task should implement a
`TerminalMappingBase` subclass that applies:

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

`FixedTerminalMapping` is a no-op mapping for the current state-to-state,
tracking, and corridor/ESDF paths.
