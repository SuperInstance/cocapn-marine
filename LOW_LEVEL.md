# LOW LEVEL — Cocapn Marine

> *For contributors extending the marine sensor integration.*

## Internal Architecture

```
cocapn-marine/
├── src/
│   ├── lib.rs         # Module declarations + re-exports
│   ├── nmea.rs        # NMEA 0183 sentence parser
│   ├── sensor.rs      # SensorReading, SensorType, ReadingQuality
│   ├── autopilot.rs   # PID autopilot implementation
│   ├── bathy.rs       # Bathymetric data logging
│   └── deadband.rs    # Heading deadband monitor
└── Cargo.toml
```

### Module Map

| Module | Responsibility | Key Types |
|--------|---------------|-----------|
| `nmea` | Parse NMEA sentences | `NmeaMessage` enum, 7 sentence types |
| `sensor` | Typed sensor data | `SensorReading<T>`, `SensorType` |
| `autopilot` | PID control loop | `Autopilot`, `AutopilotOutput` |
| `bathy` | Depth-position logging | `BathyLog`, `DepthSample` |
| `deadband` | Deviation detection | `HeadingDeadband` |

## PID Controller Implementation

```rust
// Standard PID with anti-windup
error = target - current                    // normalize to [-180, 180)
integral += error * dt                      // accumulate
integral = clamp(integral, -integral_limit, integral_limit)  // anti-windup
derivative = (error - last_error) / dt
output = kp * error + ki * integral + kd * derivative
output = clamp(output, -max_correction, max_correction)
```

## Testing

```bash
cargo test
```

Tests cover: PID convergence, anti-windup behavior, heading normalization, NMEA checksum validation, bathymetric edge cases.

## Safety

`#![deny(unsafe_code)]` — no unsafe code permitted. Marine safety-critical domain.

## Debugging

- Enable logging: `RUST_LOG=cocapn_marine=debug`
- NMEA parser logs unknown sentence types at debug level
- Autopilot logs PID terms (error, P, I, D, output) at trace level

## Future Work

- AIS message parsing
- Serial port abstraction
- Kalman filter for sensor fusion
- Autopilot tuning autotuner
