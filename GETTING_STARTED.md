# GETTING STARTED — Cocapn Marine

> *Estimated time: 5 minutes*

## Prerequisites

- Rust edition 2021+
- A serial or network NMEA 0183 data source (or use simulated data)

## Installation

```toml
[dependencies]
cocapn-marine = "0.1"
```

## Your First 5 Minutes

### 1. Create an Autopilot

```rust
use cocapn_marine::Autopilot;

// Kp=1.5, Ki=0.3, Kd=0.1, max_correction=30°, tolerance=5°
let mut pilot = Autopilot::new(1.5, 0.3, 0.1, 30.0, 5.0);
pilot.set_target_heading(270.0);

// Simulate sensor updates
loop {
    let current_heading = read_compass(); // your function
    let output = pilot.update(current_heading, 0.1);
    set_rudder(output.rudder_command); // your function
    std::thread::sleep(std::time::Duration::from_millis(100));
}
```

### 2. Parse NMEA Sentences

```rust
use cocapn_marine::nmea::parse_sentence;

let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
match parse_sentence(sentence) {
    Some(msg) => println!("Parsed: {:?}", msg),
    None => println!("Unknown sentence type"),
}
```

### 3. Record Bathymetry

```rust
use cocapn_marine::bathy::BathyLog;

let mut log = BathyLog::new();
log.record(48.5, -122.6, 15.0, 270.0);
println!("Samples: {}", log.count());
println!("Max depth: {}m", log.max_depth());
```

## Next Steps

- [ARCHITECTURE.md](./ARCHITECTURE.md) — Module design
- [API_REFERENCE.md](./API_REFERENCE.md) — Full API
- [LOW_LEVEL.md](./LOW_LEVEL.md) — Internal structure
