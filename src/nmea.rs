use chrono::{NaiveDate, NaiveTime};
use thiserror::Error;

/// A raw NMEA 0183 sentence with its components.
#[derive(Debug, Clone, PartialEq)]
pub struct NmeaSentence {
    pub talker: String,
    pub sentence_type: String,
    pub fields: Vec<String>,
    pub checksum: u8,
}

/// Parsed NMEA 0183 messages supported by this crate.
#[derive(Debug, Clone, PartialEq)]
pub enum NmeaMessage {
    /// Global Positioning System Fix Data
    Gga(GgaData),
    /// Recommended Minimum Navigation Information
    Rmc(RmcData),
    /// Depth of water
    Dpt(DptData),
    /// Water speed and heading
    Vhw(VhwData),
    /// Heading
    Hdg(HdgData),
    /// Catch-all for parsed but unrecognised NMEA sentences
    Custom(NmeaSentence),
}

/// GGA — GPS Fix Data
#[derive(Debug, Clone, PartialEq)]
pub struct GgaData {
    pub time: NaiveTime,
    pub lat: f64,
    pub lon: f64,
    pub fix_quality: u8,
    pub satellites: u8,
    pub hdop: f64,
    pub altitude: f64,
}

/// RMC — Recommended Minimum Navigation Information
#[derive(Debug, Clone, PartialEq)]
pub struct RmcData {
    pub time: NaiveTime,
    pub date: NaiveDate,
    pub lat: f64,
    pub lon: f64,
    pub speed_knots: f64,
    pub course: f64,
}

/// DPT — Depth of Water
#[derive(Debug, Clone, PartialEq)]
pub struct DptData {
    pub depth_meters: f64,
    pub offset: f64,
}

/// VHW — Water speed and heading
#[derive(Debug, Clone, PartialEq)]
pub struct VhwData {
    pub heading_true: Option<f64>,
    pub heading_magnetic: Option<f64>,
    pub speed_knots: f64,
}

/// HDG — Heading
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, PartialEq)]
pub struct HdgData {
    pub heading: f64,
    pub deviation: Option<f64>,
    pub variation: Option<f64>,
}

/// Errors that can occur during NMEA parsing.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum NmeaError {
    #[error("sentence too short")]
    TooShort,
    #[error("missing leading '$'")]
    MissingLeadingDollar,
    #[error("missing trailing checksum (no '*')")]
    MissingChecksumMarker,
    #[error("invalid checksum: expected {expected:#04x}, got {actual:#04x}")]
    ChecksumMismatch { expected: u8, actual: u8 },
    #[error("invalid checksum hex digits")]
    InvalidChecksumHex,
    #[error("unknown sentence type: {0}")]
    UnknownSentenceType(String),
    #[error("missing fields for sentence type {0}")]
    MissingFields(String),
    #[error("failed to parse float: {0}")]
    FloatParse(String),
    #[error("failed to parse time: {0}")]
    TimeParse(String),
    #[error("failed to parse date: {0}")]
    DateParse(String),
    #[error("invalid coordinate: {0}")]
    InvalidCoordinate(String),
    #[error("invalid fix quality value: {0}")]
    InvalidFixQuality(String),
}

/// Compute the NMEA checksum (XOR of all bytes between '$' and '*').
fn compute_checksum(sentence: &str) -> u8 {
    let mut checksum: u8 = 0;
    for byte in sentence.bytes() {
        match byte {
            b'$' => {}
            b'*' => break,
            _ => checksum ^= byte,
        }
    }
    checksum
}

/// Parse a decimal degrees value from NMEA format.
///
/// NMEA latitude: DDMM.MMMM  →  DD + MM.MMMM / 60
/// NMEA longitude: DDDMM.MMMM → DDD + MM.MMMM / 60
pub fn parse_nmea_coordinate(value: &str, is_latitude: bool) -> Result<f64, NmeaError> {
    if value.is_empty() {
        return Err(NmeaError::InvalidCoordinate("empty coordinate".into()));
    }
    let dot_pos = value.find('.');
    let int_part = match dot_pos {
        Some(pos) => &value[..pos],
        None => value,
    };
    // Latitude: DDMM (2 deg digits), Longitude: DDDMM (3 deg digits)
    let deg_digits = if is_latitude { 2 } else { 3 };
    if int_part.len() < deg_digits {
        return Err(NmeaError::InvalidCoordinate(format!(
            "int part too short: '{}'",
            value
        )));
    }
    let deg_str = &int_part[..deg_digits];
    let min_str = &int_part[deg_digits..];
    let deg: f64 = deg_str
        .parse()
        .map_err(|_| NmeaError::FloatParse(deg_str.into()))?;
    let minutes: f64 = if min_str.is_empty() {
        if let Some(pos) = dot_pos {
            let frac = &value[pos..];
            frac
                .parse()
                .map_err(|_| NmeaError::FloatParse(frac.into()))?
        } else {
            0.0
        }
    } else {
        let full_min = if let Some(pos) = dot_pos {
            let frac = &value[pos..];
            format!("{}{}", min_str, frac)
        } else {
            min_str.to_string()
        };
        full_min
            .parse()
            .map_err(|_| NmeaError::FloatParse(full_min.clone()))? 
    };
    Ok(deg + minutes / 60.0)
}

/// Parse a full NMEA 0183 sentence string.
///
/// Expects format like: `$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,...*47`
pub fn parse_nmea(sentence: &str) -> Result<NmeaMessage, NmeaError> {
    let trimmed = sentence.trim();
    if trimmed.len() < 6 {
        return Err(NmeaError::TooShort);
    }
    if !trimmed.starts_with('$') {
        return Err(NmeaError::MissingLeadingDollar);
    }

    // Find and verify checksum
    let star = trimmed.rfind('*').ok_or(NmeaError::MissingChecksumMarker)?;
    let body = &trimmed[1..star]; // skip '$', stop before '*'
    let checksum_hex = &trimmed[star + 1..];

    let expected_checksum = compute_checksum(trimmed);
    let actual_checksum = u8::from_str_radix(checksum_hex, 16)
        .map_err(|_| NmeaError::InvalidChecksumHex)?;
    if expected_checksum != actual_checksum {
        return Err(NmeaError::ChecksumMismatch {
            expected: expected_checksum,
            actual: actual_checksum,
        });
    }

    // Split body into fields
    let parts: Vec<&str> = body.split(',').collect();
    if parts.is_empty() || parts[0].len() < 3 {
        return Err(NmeaError::TooShort);
    }

    let talker = parts[0][..2].to_string();
    let sentence_type = parts[0][2..].to_string();
    let fields: Vec<String> = parts[1..].iter().map(|s| s.to_string()).collect();

    let raw = NmeaSentence {
        talker: talker.clone(),
        sentence_type: sentence_type.clone(),
        fields: fields.clone(),
        checksum: actual_checksum,
    };

    match sentence_type.as_str() {
        "GGA" => parse_gga(fields, raw),
        "RMC" => parse_rmc(fields, raw),
        "DPT" => parse_dpt(fields, raw),
        "VHW" => parse_vhw(fields, raw),
        "HDG" => parse_hdg(fields, raw),
        _ => Ok(NmeaMessage::Custom(raw)),
    }
}

fn parse_gga(fields: Vec<String>, _raw: NmeaSentence) -> Result<NmeaMessage, NmeaError> {
    if fields.len() < 9 {
        return Err(NmeaError::MissingFields("GGA".into()));
    }
    // 0: time (hhmmss.ss), 1: lat, 2: N/S, 3: lon, 4: E/W, 5: fix quality, 6: satellites, 7: HDOP, 8: altitude
    let time_str = &fields[0];
    let time = if time_str.is_empty() {
        NaiveTime::from_hms_opt(0, 0, 0).unwrap()
    } else {
        parse_nmea_time(time_str)?
    };

    let lat_sign = if fields[2] == "S" { -1.0 } else { 1.0 };
    let lon_sign = if fields[4] == "W" { -1.0 } else { 1.0 };
    let lat = parse_nmea_coordinate(&fields[1], true)? * lat_sign;
    let lon = parse_nmea_coordinate(&fields[3], false)? * lon_sign;

    let fix_quality: u8 = fields[5]
        .parse()
        .map_err(|_| NmeaError::InvalidFixQuality(fields[5].clone()))?;
    let satellites: u8 = fields[6]
        .parse()
        .map_err(|_| NmeaError::FloatParse(fields[6].clone()))?;
    let hdop: f64 = parse_field_float(&fields[7])?;
    let altitude: f64 = parse_field_float(&fields[8])?;

    Ok(NmeaMessage::Gga(GgaData {
        time,
        lat,
        lon,
        fix_quality,
        satellites,
        hdop,
        altitude,
    }))
}

fn parse_rmc(fields: Vec<String>, _raw: NmeaSentence) -> Result<NmeaMessage, NmeaError> {
    if fields.len() < 9 {
        return Err(NmeaError::MissingFields("RMC".into()));
    }
    // 0: time, 1: status (A=active), 2: lat, 3: N/S, 4: lon, 5: E/W, 6: speed, 7: course, 8: date
    let time_str = &fields[0];
    let time = if time_str.is_empty() {
        NaiveTime::from_hms_opt(0, 0, 0).unwrap()
    } else {
        parse_nmea_time(time_str)?
    };

    let lat_sign = if fields[3] == "S" { -1.0 } else { 1.0 };
    let lon_sign = if fields[5] == "W" { -1.0 } else { 1.0 };
    let lat = parse_nmea_coordinate(&fields[2], true)? * lat_sign;
    let lon = parse_nmea_coordinate(&fields[4], false)? * lon_sign;

    let speed_knots = parse_field_float(&fields[6])?;
    let course = parse_field_float(&fields[7])?;

    let date = parse_nmea_date(&fields[8])?;

    Ok(NmeaMessage::Rmc(RmcData {
        time,
        date,
        lat,
        lon,
        speed_knots,
        course,
    }))
}

fn parse_dpt(fields: Vec<String>, _raw: NmeaSentence) -> Result<NmeaMessage, NmeaError> {
    if fields.is_empty() {
        return Err(NmeaError::MissingFields("DPT".into()));
    }
    let depth_meters = parse_field_float(&fields[0])?;
    let offset = if fields.len() > 1 && !fields[1].is_empty() {
        parse_field_float(&fields[1])?
    } else {
        0.0
    };
    Ok(NmeaMessage::Dpt(DptData {
        depth_meters,
        offset,
    }))
}

fn parse_vhw(fields: Vec<String>, _raw: NmeaSentence) -> Result<NmeaMessage, NmeaError> {
    if fields.len() < 5 {
        return Err(NmeaError::MissingFields("VHW".into()));
    }
    // 0: heading true, 1: "T", 2: heading magnetic, 3: "M", 4: speed knots, 5: "N"
    let heading_true = if fields[0].is_empty() {
        None
    } else {
        Some(parse_field_float(&fields[0])?)
    };
    let heading_magnetic = if fields[2].is_empty() {
        None
    } else {
        Some(parse_field_float(&fields[2])?)
    };
    let speed_knots = parse_field_float(&fields[4])?;
    Ok(NmeaMessage::Vhw(VhwData {
        heading_true,
        heading_magnetic,
        speed_knots,
    }))
}

fn parse_hdg(fields: Vec<String>, _raw: NmeaSentence) -> Result<NmeaMessage, NmeaError> {
    if fields.is_empty() || fields[0].is_empty() {
        return Err(NmeaError::MissingFields("HDG".into()));
    }
    // 0: heading, 1: deviation, 2: dev dir (E/W), 3: variation, 4: var dir (E/W)
    let heading = parse_field_float(&fields[0])?;
    let deviation = if fields.len() > 1 && !fields[1].is_empty() {
        let dev = parse_field_float(&fields[1])?;
        if fields.len() > 2 && fields[2] == "W" {
            Some(-dev)
        } else {
            Some(dev)
        }
    } else {
        None
    };
    let variation = if fields.len() > 3 && !fields[3].is_empty() {
        let var = parse_field_float(&fields[3])?;
        if fields.len() > 4 && fields[4] == "W" {
            Some(-var)
        } else {
            Some(var)
        }
    } else {
        None
    };
    Ok(NmeaMessage::Hdg(HdgData {
        heading,
        deviation,
        variation,
    }))
}

fn parse_field_float(s: &str) -> Result<f64, NmeaError> {
    if s.is_empty() {
        return Ok(0.0);
    }
    s.parse()
        .map_err(|_| NmeaError::FloatParse(s.to_string()))
}

fn parse_nmea_time(s: &str) -> Result<NaiveTime, NmeaError> {
    // Format: hhmmss.ss
    let s = if s.len() > 6 { &s[..6] } else { s };
    let h: u32 = s[..2]
        .parse()
        .map_err(|_| NmeaError::TimeParse(s.into()))?;
    let m: u32 = s[2..4]
        .parse()
        .map_err(|_| NmeaError::TimeParse(s.into()))?;
    let sec: u32 = s[4..6]
        .parse()
        .map_err(|_| NmeaError::TimeParse(s.into()))?;
    NaiveTime::from_hms_opt(h, m, sec)
        .ok_or_else(|| NmeaError::TimeParse(s.to_string()))
}

fn parse_nmea_date(s: &str) -> Result<NaiveDate, NmeaError> {
    // Format: ddmmyy — NMEA 0183 convention: years 00-80 → 2000s, 81-99 → 1900s
    if s.len() < 6 {
        return Err(NmeaError::DateParse(s.into()));
    }
    let d: u32 = s[..2]
        .parse()
        .map_err(|_| NmeaError::DateParse(s.into()))?;
    let m: u32 = s[2..4]
        .parse()
        .map_err(|_| NmeaError::DateParse(s.into()))?;
    let yy: i32 = s[4..6]
        .parse::<i32>()
        .map_err(|_| NmeaError::DateParse(s.into()))?;
    let y = if yy > 80 { 1900 + yy } else { 2000 + yy };
    NaiveDate::from_ymd_opt(y, m, d)
        .ok_or_else(|| NmeaError::DateParse(s.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    // ----- Helper: compute checksum for a full NMEA string -----
    fn nmea_checksum(body_without_dollar_and_star: &str) -> u8 {
        let mut cs: u8 = 0;
        for b in body_without_dollar_and_star.bytes() {
            cs ^= b;
        }
        cs
    }

    fn make_sentence(body: &str) -> String {
        let cs = nmea_checksum(body);
        format!("${}*{:02X}", body, cs)
    }

    // ----- GGA tests -----
    #[test]
    fn test_gga_valid() {
        // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        let sent = make_sentence(&body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Gga(g) => {
                assert_eq!(g.time, NaiveTime::from_hms_opt(12, 35, 19).unwrap());
                assert!((g.lat - 48.1173).abs() < 0.001);
                assert!((g.lon - 11.516667).abs() < 0.001);
                assert_eq!(g.fix_quality, 1);
                assert_eq!(g.satellites, 8);
                assert!((g.hdop - 0.9).abs() < 0.01);
                assert!((g.altitude - 545.4).abs() < 0.01);
            }
            _ => panic!("Expected GGA"),
        }
    }

    #[test]
    fn test_gga_southern_hemisphere_western() {
        let body = "GPGGA,123519,3343.000,S,15130.000,W,1,05,1.2,100.0,M,,";
        let sent = make_sentence(&body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Gga(g) => {
                assert!(g.lat < 0.0);
                assert!(g.lon < 0.0);
                assert!((g.lat - (-33.716667)).abs() < 0.001);
                assert!((g.lon - (-151.5)).abs() < 0.001);
            }
            _ => panic!("Expected GGA"),
        }
    }

    // ----- RMC tests -----
    #[test]
    fn test_rmc_valid() {
        let body = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";
        let sent = make_sentence(&body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Rmc(r) => {
                assert_eq!(r.time, NaiveTime::from_hms_opt(12, 35, 19).unwrap());
                assert_eq!(r.date, NaiveDate::from_ymd_opt(1994, 3, 23).unwrap());
                assert!((r.lat - 48.1173).abs() < 0.001);
                assert!((r.lon - 11.516667).abs() < 0.001);
                assert!((r.speed_knots - 22.4).abs() < 0.01);
                assert!((r.course - 84.4).abs() < 0.01);
            }
            _ => panic!("Expected RMC"),
        }
    }

    #[test]
    fn test_rmc_with_empty_time() {
        // Some RMC sentences have empty time field
        let body = "GPRMC,,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";
        let sent = make_sentence(body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Rmc(r) => {
                assert_eq!(r.time, NaiveTime::from_hms_opt(0, 0, 0).unwrap());
                assert_eq!(r.date, NaiveDate::from_ymd_opt(1994, 3, 23).unwrap());
            }
            _ => panic!("Expected RMC"),
        }
    }

    // ----- DPT tests -----
    #[test]
    fn test_dpt_valid() {
        let body = "SDDPT,12.3,0.0";
        let sent = make_sentence(&body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Dpt(d) => {
                assert!((d.depth_meters - 12.3).abs() < 0.01);
                assert!((d.offset - 0.0).abs() < 0.01);
            }
            _ => panic!("Expected DPT"),
        }
    }

    #[test]
    fn test_dpt_no_offset() {
        let body = "SDDPT,5.0";
        let sent = make_sentence(body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Dpt(d) => {
                assert!((d.depth_meters - 5.0).abs() < 0.01);
                assert!((d.offset - 0.0).abs() < 0.01);
            }
            _ => panic!("Expected DPT"),
        }
    }

    // ----- VHW tests -----
    #[test]
    fn test_vhw_valid() {
        let body = "VWVHW,45.0,T,43.5,M,12.5,N";
        let sent = make_sentence(&body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Vhw(v) => {
                assert!((v.heading_true.unwrap() - 45.0).abs() < 0.01);
                assert!((v.heading_magnetic.unwrap() - 43.5).abs() < 0.01);
                assert!((v.speed_knots - 12.5).abs() < 0.01);
            }
            _ => panic!("Expected VHW"),
        }
    }

    #[test]
    fn test_vhw_partial() {
        // Missing true heading
        let body = "VWVHW,,T,43.5,M,12.5,N";
        let sent = make_sentence(body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Vhw(v) => {
                assert!(v.heading_true.is_none());
                assert!((v.heading_magnetic.unwrap() - 43.5).abs() < 0.01);
            }
            _ => panic!("Expected VHW"),
        }
    }

    // ----- HDG tests -----
    #[test]
    fn test_hdg_valid() {
        let body = "HCHDG,101.1,1.0,E,14.0,W";
        let sent = make_sentence(&body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Hdg(h) => {
                assert!((h.heading - 101.1).abs() < 0.01);
                assert!((h.deviation.unwrap() - 1.0).abs() < 0.01);
                assert!((h.variation.unwrap() - (-14.0)).abs() < 0.01);
            }
            _ => panic!("Expected HDG"),
        }
    }

    // ----- Checksum tests -----
    #[test]
    fn test_checksum_valid() {
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        let sent = make_sentence(&body);
        assert!(parse_nmea(&sent).is_ok());
    }

    #[test]
    fn test_checksum_invalid() {
        let sent = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
        let err = parse_nmea(sent).unwrap_err();
        match err {
            NmeaError::ChecksumMismatch { .. } => {}
            _ => panic!("Expected ChecksumMismatch, got {:?}", err),
        }
    }

    #[test]
    fn test_checksum_invalid_hex() {
        let sent = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*ZZ";
        let err = parse_nmea(sent).unwrap_err();
        match err {
            NmeaError::InvalidChecksumHex => {}
            _ => panic!("Expected InvalidChecksumHex, got {:?}", err),
        }
    }

    #[test]
    fn test_checksum_missing_marker() {
        let sent = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        let err = parse_nmea(sent).unwrap_err();
        match err {
            NmeaError::MissingChecksumMarker => {}
            _ => panic!("Expected MissingChecksumMarker, got {:?}", err),
        }
    }

    // ----- Edge-case parsing tests -----
    #[test]
    fn test_too_short() {
        assert_eq!(
            parse_nmea("$*00").unwrap_err(),
            NmeaError::TooShort
        );
    }

    #[test]
    fn test_missing_leading_dollar() {
        let err = parse_nmea("GPGGA,1*00").unwrap_err();
        assert_eq!(err, NmeaError::MissingLeadingDollar);
    }

    #[test]
    fn test_custom_sentence() {
        // $IIMWV,45.0,R,12.5,N,A* (wind — not in enum, should be Custom)
        let body = "IIMWV,45.0,R,12.5,N,A";
        let sent = make_sentence(body);
        let msg = parse_nmea(&sent).unwrap();
        match msg {
            NmeaMessage::Custom(c) => {
                assert_eq!(c.talker, "II");
                assert_eq!(c.sentence_type, "MWV");
                assert_eq!(c.fields.len(), 5);
            }
            _ => panic!("Expected Custom"),
        }
    }

    // ----- Coordinate parsing tests -----
    #[test]
    fn test_parse_coordinate_latitude() {
        // 4807.038,N → 48°07.038' = 48 + 7.038/60 = 48.1173
        let lat = parse_nmea_coordinate("4807.038", true).unwrap();
        assert!((lat - 48.1173).abs() < 0.001);
    }

    #[test]
    fn test_parse_coordinate_longitude() {
        // 01131.000,E → 11°31.000' = 11 + 31/60 = 11.516667
        let lon = parse_nmea_coordinate("01131.000", false).unwrap();
        assert!((lon - 11.516667).abs() < 0.001);
    }

    #[test]
    fn test_parse_coordinate_empty() {
        assert!(parse_nmea_coordinate("", true).is_err());
    }

    // ----- compute_checksum verification -----
    #[test]
    fn test_compute_checksum_known() {
        let sent = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        assert_eq!(compute_checksum(sent), 0x47);
    }

    // ----- Round-trip with make_sentence -----
    #[test]
    fn test_roundtrip_gga() {
        let body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
        let sent = make_sentence(body);
        let msg = parse_nmea(&sent).unwrap();
        assert!(matches!(msg, NmeaMessage::Gga(_)));
    }
}
