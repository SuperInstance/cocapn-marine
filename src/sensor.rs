use crate::nmea::NmeaMessage;
use chrono::{DateTime, Utc};
use thiserror::Error;

/// Sensor ID constants for well-known CoCapn sensors (for convenience, not exhaustive).
pub mod sensor_ids {
    pub const GPS: &str = "gps";
    pub const DEPTH: &str = "depth";
    pub const HEADING: &str = "heading";
    pub const SPEED: &str = "speed";
    pub const WIND: &str = "wind";
    pub const TEMPERATURE: &str = "temperature";
}

/// Types of marine sensors supported by CoCapn.
#[derive(Debug, Clone, PartialEq)]
pub enum SensorType {
    Gps,
    Depth,
    Heading,
    Speed,
    Wind,
    Temperature,
    Custom(String),
}

impl std::fmt::Display for SensorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SensorType::Gps => write!(f, "GPS"),
            SensorType::Depth => write!(f, "Depth"),
            SensorType::Heading => write!(f, "Heading"),
            SensorType::Speed => write!(f, "Speed"),
            SensorType::Wind => write!(f, "Wind"),
            SensorType::Temperature => write!(f, "Temperature"),
            SensorType::Custom(s) => write!(f, "Custom({})", s),
        }
    }
}

/// Quality indicator for a sensor reading.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReadingQuality {
    Excellent,
    Good,
    Fair,
    Poor,
    Invalid,
}

impl ReadingQuality {
    /// Convert from NMEA fix quality value.
    pub fn from_fix_quality(q: u8) -> Self {
        match q {
            4 => ReadingQuality::Excellent,
            1 | 2 => ReadingQuality::Good,
            3 => ReadingQuality::Fair,
            5 | 6 => ReadingQuality::Poor,
            _ => ReadingQuality::Invalid,
        }
    }

    /// Returns a numeric score where higher is better.
    pub fn score(&self) -> u8 {
        match self {
            ReadingQuality::Excellent => 5,
            ReadingQuality::Good => 4,
            ReadingQuality::Fair => 3,
            ReadingQuality::Poor => 2,
            ReadingQuality::Invalid => 1,
        }
    }
}

impl From<u8> for ReadingQuality {
    fn from(q: u8) -> Self {
        ReadingQuality::from_fix_quality(q)
    }
}

/// A single sensor reading with metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorReading {
    pub sensor_id: String,
    pub sensor_type: SensorType,
    pub value: f64,
    pub unit: String,
    pub timestamp: DateTime<Utc>,
    pub quality: ReadingQuality,
}

impl SensorReading {
    pub fn new(
        sensor_id: impl Into<String>,
        sensor_type: SensorType,
        value: f64,
        unit: impl Into<String>,
        quality: ReadingQuality,
    ) -> Self {
        SensorReading {
            sensor_id: sensor_id.into(),
            sensor_type,
            value,
            unit: unit.into(),
            timestamp: Utc::now(),
            quality,
        }
    }
}

/// Errors produced by sensor operations.
#[derive(Error, Debug)]
pub enum SensorError {
    #[error("sensor {0} not available")]
    NotAvailable(String),
    #[error("sensor {0} read failure: {1}")]
    ReadFailure(String, String),
    #[error("sensor {0} returned invalid data")]
    InvalidData(String),
    #[error("communication error: {0}")]
    CommsError(String),
}

impl From<crate::nmea::NmeaError> for SensorError {
    fn from(_e: crate::nmea::NmeaError) -> Self {
        SensorError::InvalidData("NMEA parse error".into())
    }
}

/// The [`Sensor`] trait — implemented by all CoCapn marine sensors.
pub trait Sensor: Send + Sync {
    /// Unique identifier for this sensor (e.g. "gps-01").
    fn sensor_id(&self) -> &str;

    /// Type of sensor.
    fn sensor_type(&self) -> SensorType;

    /// Perform a reading and return the measurement.
    fn read(&mut self) -> Result<SensorReading, SensorError>;

    /// Return the last reading without performing a new one.
    fn last_reading(&self) -> Option<&SensorReading>;
}

/// A simple GPS sensor that stores parsed GGA/RMC data.
#[derive(Debug)]
pub struct GpsSensor {
    id: String,
    last: Option<SensorReading>,
}

impl GpsSensor {
    pub fn new(id: impl Into<String>) -> Self {
        GpsSensor {
            id: id.into(),
            last: None,
        }
    }

    /// Ingest a parsed NMEA message and produce a reading.
    pub fn ingest(&mut self, msg: &NmeaMessage) -> Result<SensorReading, SensorError> {
        match msg {
            NmeaMessage::Gga(g) => {
                let quality = ReadingQuality::from_fix_quality(g.fix_quality);
                let reading = SensorReading {
                    sensor_id: self.id.clone(),
                    sensor_type: SensorType::Gps,
                    value: g.altitude,
                    unit: "m".into(),
                    timestamp: Utc::now(),
                    quality,
                };
                self.last = Some(reading.clone());
                Ok(reading)
            }
            _ => Err(SensorError::InvalidData(
                "expected GGA message for GPS sensor".into(),
            )),
        }
    }
}

impl Sensor for GpsSensor {
    fn sensor_id(&self) -> &str {
        &self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Gps
    }

    fn read(&mut self) -> Result<SensorReading, SensorError> {
        self.last
            .clone()
            .ok_or_else(|| SensorError::NotAvailable(self.id.clone()))
    }

    fn last_reading(&self) -> Option<&SensorReading> {
        self.last.as_ref()
    }
}

/// A simple depth sensor backed by DPT messages.
#[derive(Debug)]
pub struct DepthSensor {
    id: String,
    last: Option<SensorReading>,
}

impl DepthSensor {
    pub fn new(id: impl Into<String>) -> Self {
        DepthSensor {
            id: id.into(),
            last: None,
        }
    }

    pub fn ingest(&mut self, msg: &NmeaMessage) -> Result<SensorReading, SensorError> {
        match msg {
            NmeaMessage::Dpt(d) => {
                let reading = SensorReading {
                    sensor_id: self.id.clone(),
                    sensor_type: SensorType::Depth,
                    value: d.depth_meters,
                    unit: "m".into(),
                    timestamp: Utc::now(),
                    quality: ReadingQuality::Good,
                };
                self.last = Some(reading.clone());
                Ok(reading)
            }
            _ => Err(SensorError::InvalidData(
                "expected DPT message for depth sensor".into(),
            )),
        }
    }
}

impl Sensor for DepthSensor {
    fn sensor_id(&self) -> &str {
        &self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Depth
    }

    fn read(&mut self) -> Result<SensorReading, SensorError> {
        self.last
            .clone()
            .ok_or_else(|| SensorError::NotAvailable(self.id.clone()))
    }

    fn last_reading(&self) -> Option<&SensorReading> {
        self.last.as_ref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nmea::parse_nmea;

    fn compute_nmea_checksum(body: &str) -> u8 {
        let mut cs: u8 = 0;
        for b in body.bytes() {
            cs ^= b;
        }
        cs
    }

    fn make_nmea(body: &str) -> String {
        format!("${}*{:02X}", body, compute_nmea_checksum(body))
    }

    #[test]
    fn test_gps_sensor_ingest() {
        let sent = make_nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        let msg = parse_nmea(&sent).unwrap();
        let mut sensor = GpsSensor::new("gps-01");
        let reading = sensor.ingest(&msg).unwrap();
        assert_eq!(reading.sensor_id, "gps-01");
        assert_eq!(reading.sensor_type, SensorType::Gps);
        assert!((reading.value - 545.4).abs() < 0.01);
        assert_eq!(reading.unit, "m");
        assert_eq!(reading.quality, ReadingQuality::Good);
    }

    #[test]
    fn test_gps_sensor_wrong_message_type() {
        let sent = make_nmea("SDDPT,12.3,0.0");
        let msg = parse_nmea(&sent).unwrap();
        let mut sensor = GpsSensor::new("gps-01");
        assert!(sensor.ingest(&msg).is_err());
    }

    #[test]
    fn test_depth_sensor_ingest() {
        let sent = make_nmea("SDDPT,12.3,0.0");
        let msg = parse_nmea(&sent).unwrap();
        let mut sensor = DepthSensor::new("depth-01");
        let reading = sensor.ingest(&msg).unwrap();
        assert_eq!(reading.sensor_id, "depth-01");
        assert_eq!(reading.sensor_type, SensorType::Depth);
        assert!((reading.value - 12.3).abs() < 0.01);
        assert_eq!(reading.unit, "m");
    }

    #[test]
    fn test_sensor_trait_read_fails_without_data() {
        let mut sensor = GpsSensor::new("gps-02");
        let result = sensor.read();
        assert!(result.is_err());
        match result.unwrap_err() {
            SensorError::NotAvailable(id) => assert_eq!(id, "gps-02"),
            e => panic!("Expected NotAvailable, got {:?}", e),
        }
    }

    #[test]
    fn test_sensor_trait_last_reading() {
        let sent = make_nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
        let msg = parse_nmea(&sent).unwrap();
        let mut sensor = GpsSensor::new("gps-03");
        assert!(sensor.last_reading().is_none());
        sensor.ingest(&msg).unwrap();
        assert!(sensor.last_reading().is_some());
        assert_eq!(sensor.last_reading().unwrap().sensor_id, "gps-03");
    }

    #[test]
    fn test_reading_quality_from_fix() {
        assert_eq!(ReadingQuality::from_fix_quality(0), ReadingQuality::Invalid);
        assert_eq!(ReadingQuality::from_fix_quality(1), ReadingQuality::Good);
        assert_eq!(ReadingQuality::from_fix_quality(4), ReadingQuality::Excellent);
        assert_eq!(ReadingQuality::from_fix_quality(5), ReadingQuality::Poor);
    }

    #[test]
    fn test_reading_quality_score_ordering() {
        let excellent = ReadingQuality::Excellent.score();
        let good = ReadingQuality::Good.score();
        let fair = ReadingQuality::Fair.score();
        let poor = ReadingQuality::Poor.score();
        let invalid = ReadingQuality::Invalid.score();
        assert!(excellent > good);
        assert!(good > fair);
        assert!(fair > poor);
        assert!(poor > invalid);
    }

    #[test]
    fn test_sensor_display() {
        assert_eq!(SensorType::Gps.to_string(), "GPS");
        assert_eq!(SensorType::Custom("foo".into()).to_string(), "Custom(foo)");
    }
}
