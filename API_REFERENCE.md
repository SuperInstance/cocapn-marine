# API Reference — Cocapn Marine

> *Public API surface. MSRV: Rust edition 2021.*

---

## `Autopilot`

```rust
pub struct Autopilot { /* private fields */ }
```

PID-based autopilot with anti-windup and deadband detection.

**Constructor:**
| Method | Signature | Description |
|--------|-----------|-------------|
| `new` | `(kp, ki, kd, max_correction, tolerance) -> Self` | Create with PID gains and limits |

**Methods:**
| Method | Signature | Description |
|--------|-----------|-------------|
| `set_target_heading` | `(&mut self, heading: f64)` | Set target heading (0–360°) |
| `update` | `(&mut self, heading: f64, dt: f64) -> AutopilotOutput` | Compute rudder command |
| `target_heading` | `(&self) -> f64` | Current target |
| `reset` | `(&mut self)` | Clear integral term |

**Output:**
```rust
pub struct AutopilotOutput {
    pub rudder_command: f64,  // +starboard, -port (degrees)
    pub heading_error: f64,   // signed error [-180, 180)
    pub is_on_course: bool,   // within tolerance
}
```

---

## `SensorReading`

```rust
pub struct SensorReading<T> {
    pub sensor_id: String,
    pub sensor_type: SensorType,
    pub value: T,
    pub quality: ReadingQuality,
    pub timestamp: DateTime<Utc>,
}
```

**SensorType:** `Gps | Depth | Heading | Speed | Wind | Temperature | Custom(String)`

**ReadingQuality:** `Excellent | Good | Fair | Poor | Invalid`

---

## `nmea::parse_sentence`

```rust
pub fn parse_sentence(sentence: &str) -> Option<NmeaMessage>
```

Parses NMEA 0183 sentences. Supports GPGGA, GPRMC, HDG, DPT, VHW, MWV, MTW.

**NmeaMessage variants:**
| Variant | Description |
|---------|-------------|
| `GGA(GgaData)` | GPS fix data |
| `RMC(RmcData)` | Recommended minimum navigation |
| `HDG(HdgData)` | Heading |
| `DPT(DptData)` | Depth |
| `VHW(VhwData)` | Water speed and heading |
| `MWV(MwvData)` | Wind speed and angle |
| `MTW(MtwData)` | Water temperature |

---

## `bathy::BathyLog`

```rust
pub struct BathyLog(Vec<DepthSample>);
pub struct DepthSample { pub lat, pub lon, pub depth, pub heading, pub timestamp }
```

**Methods:**
- `new()` — Create empty log
- `record(lat, lon, depth, heading)` — Add sample
- `count()` → `usize` — Sample count
- `max_depth()` → `f64` — Maximum recorded depth
- `profile()` → `Vec<DepthSample>` — All samples

---

## `deadband::HeadingDeadband`

```rust
pub struct HeadingDeadband { tolerance: f64 }
```

**Methods:**
- `new(tolerance)` — Create with heading tolerance
- `check(heading, target)` → `bool` — Returns true if deviated beyond tolerance
- `set_tolerance(tolerance)` — Update tolerance

## Feature Gates

| Feature | What It Enables | Default? |
|---------|----------------|----------|
| `serde` | Serialization for sensor readings | No |

## Minimum Supported Rust Version (MSRV)

Rust edition 2021 (any stable toolchain).
