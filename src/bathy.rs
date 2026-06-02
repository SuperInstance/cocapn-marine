use crate::sensor::ReadingQuality;
use chrono::{DateTime, Utc};
use thiserror::Error;

/// A single bathymetric depth measurement.
#[derive(Debug, Clone, PartialEq)]
pub struct BathyRecord {
    pub lat: f64,
    pub lon: f64,
    pub depth: f64,
    pub timestamp: DateTime<Utc>,
    pub quality: ReadingQuality,
}

/// In-memory store of bathymetric soundings with spatial query methods.
#[derive(Debug, Clone)]
pub struct BathyDatabase {
    pub records: Vec<BathyRecord>,
}

/// Errors from bathymetric operations.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum BathyError {
    #[error("CSV parse error on line {line}: {message}")]
    CsvParseError { line: usize, message: String },
    #[error("no records in database")]
    NoRecords,
}

impl BathyDatabase {
    /// Create an empty bathymetric database.
    pub fn new() -> Self {
        BathyDatabase { records: Vec::new() }
    }

    /// Add a single record to the database.
    pub fn record(&mut self, record: BathyRecord) {
        self.records.push(record);
    }

    /// Return records forming a contour line at the given depth.
    ///
    /// Selects all records whose depth is within `tolerance` of `depth`.
    /// Returns (lat, lon) pairs.
    pub fn contour_line(&self, depth: f64, tolerance: f64) -> Vec<(f64, f64)> {
        self.records
            .iter()
            .filter(|r| (r.depth - depth).abs() <= tolerance)
            .map(|r| (r.lat, r.lon))
            .collect()
    }

    /// Estimate depth at a given (lat, lon) using nearest-neighbour interpolation.
    ///
    /// Returns `None` if the database is empty.
    pub fn depth_at(&self, lat: f64, lon: f64) -> Option<f64> {
        if self.records.is_empty() {
            return None;
        }
        let mut best_idx = 0usize;
        let mut best_dist = f64::MAX;
        for (i, r) in self.records.iter().enumerate() {
            let dlat = r.lat - lat;
            let dlon = r.lon - lon;
            let dist = dlat * dlat + dlon * dlon;
            if dist < best_dist {
                best_dist = dist;
                best_idx = i;
            }
        }
        Some(self.records[best_idx].depth)
    }

    /// Export all records as a GeoJSON FeatureCollection string.
    pub fn export_geojson(&self) -> String {
        let mut features = Vec::new();
        for r in &self.records {
            let depth = r.depth;
            let lat = r.lat;
            let lon = r.lon;
            let ts = r.timestamp.to_rfc3339();
            let quality = match r.quality {
                ReadingQuality::Excellent => "excellent",
                ReadingQuality::Good => "good",
                ReadingQuality::Fair => "fair",
                ReadingQuality::Poor => "poor",
                ReadingQuality::Invalid => "invalid",
            };
            features.push(format!(
                r#"{{"type":"Feature","geometry":{{"type":"Point","coordinates":[{lon},{lat}]}},"properties":{{"depth":{depth},"timestamp":"{ts}","quality":"{quality}"}}}}"#,
                lon = lon,
                lat = lat,
                depth = depth,
                ts = ts,
                quality = quality,
            ));
        }
        format!(
            r#"{{"type":"FeatureCollection","features":[{}]}}"#,
            features.join(",")
        )
    }

    /// Import records from a CSV string.
    ///
    /// Expected columns: lat,lon,depth[,timestamp[,quality]]
    /// Timestamp is RFC 3339 formatted. Quality defaults to Good.
    pub fn import_from_csv(&mut self, csv: &str) -> Result<usize, BathyError> {
        let mut count = 0usize;
        for (line_idx, line) in csv.lines().enumerate() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            let parts: Vec<&str> = trimmed.split(',').collect();
            if parts.len() < 3 {
                return Err(BathyError::CsvParseError {
                    line: line_idx + 1,
                    message: format!("expected at least 3 columns, got {}", parts.len()),
                });
            }
            let lat: f64 = parts[0]
                .parse()
                .map_err(|e| BathyError::CsvParseError {
                    line: line_idx + 1,
                    message: format!("invalid lat: {}", e),
                })?;
            let lon: f64 = parts[1]
                .parse()
                .map_err(|e| BathyError::CsvParseError {
                    line: line_idx + 1,
                    message: format!("invalid lon: {}", e),
                })?;
            let depth: f64 = parts[2]
                .parse()
                .map_err(|e| BathyError::CsvParseError {
                    line: line_idx + 1,
                    message: format!("invalid depth: {}", e),
                })?;

            let timestamp = if parts.len() > 3 && !parts[3].is_empty() {
                parts[3]
                    .parse::<DateTime<Utc>>()
                    .map_err(|e| BathyError::CsvParseError {
                        line: line_idx + 1,
                        message: format!("invalid timestamp: {}", e),
                    })?
            } else {
                Utc::now()
            };

            let quality = if parts.len() > 4 && !parts[4].is_empty() {
                match parts[4].to_lowercase().trim() {
                    "excellent" => ReadingQuality::Excellent,
                    "good" => ReadingQuality::Good,
                    "fair" => ReadingQuality::Fair,
                    "poor" => ReadingQuality::Poor,
                    _ => ReadingQuality::Invalid,
                }
            } else {
                ReadingQuality::Good
            };

            self.records.push(BathyRecord {
                lat,
                lon,
                depth,
                timestamp,
                quality,
            });
            count += 1;
        }
        Ok(count)
    }
}

impl Default for BathyDatabase {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;

    fn sample_records() -> Vec<BathyRecord> {
        vec![
            BathyRecord {
                lat: 48.0,
                lon: -122.0,
                depth: 10.0,
                timestamp: Utc.with_ymd_and_hms(2024, 6, 1, 12, 0, 0).unwrap(),
                quality: ReadingQuality::Excellent,
            },
            BathyRecord {
                lat: 48.001,
                lon: -122.001,
                depth: 15.0,
                timestamp: Utc.with_ymd_and_hms(2024, 6, 1, 12, 1, 0).unwrap(),
                quality: ReadingQuality::Good,
            },
            BathyRecord {
                lat: 48.002,
                lon: -122.002,
                depth: 10.5,
                timestamp: Utc.with_ymd_and_hms(2024, 6, 1, 12, 2, 0).unwrap(),
                quality: ReadingQuality::Fair,
            },
            BathyRecord {
                lat: 48.003,
                lon: -122.003,
                depth: 25.0,
                timestamp: Utc.with_ymd_and_hms(2024, 6, 1, 12, 3, 0).unwrap(),
                quality: ReadingQuality::Poor,
            },
        ]
    }

    #[test]
    fn test_record_and_count() {
        let mut db = BathyDatabase::new();
        assert_eq!(db.records.len(), 0);
        for r in sample_records() {
            db.record(r);
        }
        assert_eq!(db.records.len(), 4);
    }

    #[test]
    fn test_contour_line() {
        let mut db = BathyDatabase::new();
        for r in sample_records() {
            db.record(r);
        }
        // Depth 10.0 with tolerance 0.01 → only the first record
        let contour = db.contour_line(10.0, 0.01);
        assert_eq!(contour.len(), 1);
        assert!((contour[0].0 - 48.0).abs() < 1e-6);

        // Depth 10.0 with tolerance 0.6 → picks 10.0 and 10.5
        let contour = db.contour_line(10.0, 0.6);
        assert_eq!(contour.len(), 2);

        // Non-existent depth
        let contour = db.contour_line(999.0, 1.0);
        assert!(contour.is_empty());
    }

    #[test]
    fn test_depth_at_nearest_neighbour() {
        let mut db = BathyDatabase::new();
        for r in sample_records() {
            db.record(r);
        }
        // Exactly at first record
        let d = db.depth_at(48.0, -122.0).unwrap();
        assert!((d - 10.0).abs() < 1e-6);

        // Slightly perturbed — still nearest to first record
        let d = db.depth_at(48.0005, -122.0005).unwrap();
        assert!((d - 15.0).abs() < 1e-6);
    }

    #[test]
    fn test_depth_at_empty_db() {
        let db = BathyDatabase::new();
        assert_eq!(db.depth_at(0.0, 0.0), None);
    }

    #[test]
    fn test_export_geojson() {
        let mut db = BathyDatabase::new();
        for r in sample_records() {
            db.record(r);
        }
        let geojson = db.export_geojson();
        assert!(geojson.starts_with(r#"{"type":"FeatureCollection""#), "starts with FeatureCollection");
        assert!(geojson.contains("-122.0"), "contains lon -122.0: {}", geojson);
        assert!(geojson.contains("48.0"), "contains lat 48.0: {}", geojson);
        assert!(geojson.contains("\"depth\":10") || geojson.contains("\"depth\":10.0"), "contains depth 10.0 or 10");
        assert!(geojson.contains("\"quality\":\"excellent\""), "contains excellent quality");
    }

    #[test]
    fn test_export_geojson_empty() {
        let db = BathyDatabase::new();
        let geojson = db.export_geojson();
        assert_eq!(geojson, r#"{"type":"FeatureCollection","features":[]}"#);
    }

    #[test]
    fn test_import_from_csv_basic() {
        let mut db = BathyDatabase::new();
        let csv = "48.0,-122.0,10.0\n48.1,-122.1,15.0\n";
        let count = db.import_from_csv(csv).unwrap();
        assert_eq!(count, 2);
        assert_eq!(db.records.len(), 2);
        assert!((db.records[0].depth - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_import_from_csv_with_quality() {
        let mut db = BathyDatabase::new();
        let csv = "48.0,-122.0,10.0,2024-06-01T12:00:00Z,excellent\n48.1,-122.1,15.0,,fair\n";
        let count = db.import_from_csv(csv).unwrap();
        assert_eq!(count, 2);
        assert_eq!(db.records[0].quality, ReadingQuality::Excellent);
        assert_eq!(db.records[1].quality, ReadingQuality::Fair);
    }

    #[test]
    fn test_import_round_trip() {
        let mut db = BathyDatabase::new();
        let csv = "48.0,-122.0,10.0\nexport test,line,broken\n";
        let result = db.import_from_csv(csv);
        assert!(result.is_err());
        match result.unwrap_err() {
            BathyError::CsvParseError { line, .. } => assert_eq!(line, 2),
            e => panic!("Expected CsvParseError, got {:?}", e),
        }
    }

    #[test]
    fn test_import_skips_comments_and_blanks() {
        let mut db = BathyDatabase::new();
        let csv = "# header comment\n\n48.0,-122.0,10.0\n   \n48.1,-122.1,15.0\n";
        let count = db.import_from_csv(csv).unwrap();
        assert_eq!(count, 2);
    }

    #[test]
    fn test_default() {
        let db = BathyDatabase::default();
        assert!(db.records.is_empty());
    }
}
