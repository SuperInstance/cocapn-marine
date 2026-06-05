# Architecture — Cocapn Marine

> *Marine sensor integration library: NMEA parsing, PID autopilot, bathymetric logging, deadband monitoring.*

## Design Goals

1. **Real-time capable** — Zero allocations on hot paths, no heap in sensor parsing
2. **No-std friendly** — Core components don't need std (except chrono for timestamps)
3. **Composable** — Each module is independent; use what you need

## High-Level Overview

```
NMEA Stream ──▶ nmea::parse_sentence()
                     │
                     ▼
              ┌──────────────┐
              │ SensorReading │──▶ sensor::SensorReading
              │ NmeaMessage   │──▶ deadband::monitor()
              └──────────────┘
                     │
                     ▼
              ┌──────────────┐
              │  Autopilot   │──▶ update(heading, dt) → rudder_command
              │  (PID loop)  │──▶ set_target(), is_on_course()
              └──────────────┘
                     │
                     ▼
              ┌──────────────┐
              │  BathyLog    │──▶ record(lat, lon, depth, heading)
              │              │──▶ max_depth(), profile()
              └──────────────┘
```

## Core Components

### `nmea` — NMEA 0183 Parser
Parses standard NMEA sentences: GPGGA, GPRMC, HDG, DPT, VHW, MWV, MTW. Returns typed `NmeaMessage` variants.

### `sensor` — Sensor Abstraction
`SensorReading<T>` with quality indicators, sensor IDs, type-safe readings.

### `autopilot` — PID Autopilot
Configurable PID controller with anti-windup, max correction limits, on-course deadband detection.

### `bathy` — Bathymetric Logger
Records depth-position-time triplets, provides statistics (max depth, count, profile).

### `deadband` — Heading Deadband Monitor
Detects when heading deviates beyond configured tolerance for alerting.

## Data Flow

```
NMEA sentence → parser → SensorReading → autopilot.update(heading, dt) → rudder
                                      ↘ deadband.check(heading, target) → alert
                                      ↘ bathy.record(lat, lon, depth, heading) → log
```

## Key Design Decisions

### Safe Rust Only (`#![deny(unsafe_code)]`)
No unsafe code. Marine safety-critical domain.

### PID with Anti-Windup
Integral term is clamped to prevent windup after sustained error. Essential for real-world autopilots.

## Dependencies

| Dependency | Why |
|-----------|-----|
| `thiserror` | Error types |
| `log` | Runtime logging |
| `chrono` | Timestamps for sensor readings |
| `serde` (optional) | Serialization |

## Extension Points

- **New NMEA sentence types** — Add variants to `NmeaMessage` enum
- **Custom sensors** — Implement the sensor trait for proprietary hardware

## See Also

- [GETTING_STARTED.md](./GETTING_STARTED.md) — Quick start
- [API_REFERENCE.md](./API_REFERENCE.md) — Full API
- [LOW_LEVEL.md](./LOW_LEVEL.md) — Internal details
