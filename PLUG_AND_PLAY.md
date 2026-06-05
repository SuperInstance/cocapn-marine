# PLUG_AND_PLAY — Cocapn Marine

> **Marine sensor integration for CoCapn — NMEA 0183 parsing, PID autopilot, bathymetric recording, and deadband monitoring.**

## What Is This?

A Rust library for marine systems integration. Parses NMEA 0183 sentences from GPS, depth sounders, and heading sensors; implements a PID autopilot; records bathymetric data; and monitors sensor deadbands for alerting.

## Why Should You Care?

- **Read any NMEA sensor** — GPS, depth, heading, speed, wind, temperature
- **PID autopilot** — Configurable gains, anti-windup, on-course detection
- **Bathymetric recording** — Log depth profiles with timestamps and coordinates
- **Deadband monitoring** — Detect when heading deviates beyond tolerance

## Quick Start

```toml
[dependencies]
cocapn-marine = "0.1"
```

```rust
use cocapn_marine::{Autopilot, SensorReading};

let mut pilot = Autopilot::new(1.5, 0.3, 0.1, 30.0, 5.0);
pilot.set_target_heading(270.0);
let output = pilot.update(265.0, 0.1); // heading=265, dt=0.1s
println!("Rudder: {}°", output.rudder_command);
```

## ✨ Key Features

- NMEA 0183 parser (GPGGA, GPRMC, HDG, DPT, VHW)
- PID autopilot with configurable Kp, Ki, Kd gains
- `SensorReading` struct with quality indicators
- Bathymetric data logging (depth, position, time)
- Heading deadband monitoring

## Next Steps

| Guide | What It Covers |
|-------|----------------|
| [`GETTING_STARTED.md`](./GETTING_STARTED.md) | Add to project, first example |
| [`ARCHITECTURE.md`](./ARCHITECTURE.md) | Module design and data flow |
| [`API_REFERENCE.md`](./API_REFERENCE.md) | All public types |
| [`LOW_LEVEL.md`](./LOW_LEVEL.md) | Internal structure, testing |

## Status

**v0.1.0 — Active development.** Core NMEA parsing and PID autopilot are stable.
