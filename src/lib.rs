#![no_std]
#![deny(missing_docs)]

//! A unified measurement representation for sensors on the Kiwi mainboard.
//! This library provides types and serialization/deserialization methods
//! for common sensor measurements such as accelerometer, gyroscope,
//! magnetometer, temperature, and barometric pressure.

#[cfg(feature = "defmt")]
use defmt::Format;

use heapless::String;

type CrcState = crc16::State<crc16::XMODEM>;

const ACCEL_CODE: u16 = 0xACC1;
const GYRO_CODE: u16 = 0x6e50;
const MAG_CODE: u16 = 0x9A61;
const TEMP_CODE: u16 = 0x7E70;
const BARO_CODE: u16 = 0xB480;
const HUMI_CODE: u16 = 0xF0AC;
const LUX_CODE: u16 = 0x1A2B;

/// Unified measurement enum
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CommonMeasurement {
    /// Accelerometer measurement in g
    Accel(f32, f32, f32),
    /// Gyroscope measurement in degrees per second
    Gyro(f32, f32, f32),
    /// Magnetometer measurement in milligauss
    Mag(f32, f32, f32),
    /// Temperature measurement in degrees Celsius
    Temp(heapless::String<8>, f32),
    /// Temperature, pressure and altitude measurement
    Baro(f32, f32, f32),
    /// Temperature (degree Celsius), humidity (%), AQI measurement
    Humi(f32, f32, f32),
    /// Ambient light measurement in lux
    Lux(heapless::String<8>, f32),
}

/// Size of the common measurement in bytes, as returned by [`into`](CommonMeasurement::into)
pub const COMMON_MEASUREMENT_SIZE: usize = 1 + 12 + 1; // type (2 bytes) + 12 bytes (3 * f32, 8 bytes for Temp label + f32 value)

#[allow(clippy::from_over_into)]
impl Into<[u8; COMMON_MEASUREMENT_SIZE]> for CommonMeasurement {
    #[inline]
    fn into(self) -> [u8; COMMON_MEASUREMENT_SIZE] {
        self.into_bytes()
    }
}

impl CommonMeasurement {
    /// Represents the measurement as a byte array.
    #[inline]
    fn into_bytes(self) -> [u8; COMMON_MEASUREMENT_SIZE] {
        match self {
            CommonMeasurement::Accel(x, y, z) => encode_xyz(ACCEL_CODE, x, y, z),
            CommonMeasurement::Gyro(x, y, z) => encode_xyz(GYRO_CODE, x, y, z),
            CommonMeasurement::Mag(x, y, z) => encode_xyz(MAG_CODE, x, y, z),
            CommonMeasurement::Temp(label, value) => encode_label_value(TEMP_CODE, &label, value),
            CommonMeasurement::Baro(temp, pres, alt) => encode_xyz(BARO_CODE, temp, pres, alt),
            CommonMeasurement::Humi(temp, humi, aqi) => encode_xyz(HUMI_CODE, temp, humi, aqi),
            CommonMeasurement::Lux(label, value) => encode_label_value(LUX_CODE, &label, value),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
/// Errors that can occur when parsing a CommonMeasurement
pub enum CommonMeasurementError {
    /// Invalid length of input byte slice
    Length,
    /// Invalid UTF-8 string for temperature label
    String,
    /// Invalid measurement type
    Type,
    /// CRC mismatch error
    CrcMismatch,
}

impl TryFrom<&[u8]> for CommonMeasurement {
    type Error = CommonMeasurementError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != COMMON_MEASUREMENT_SIZE {
            return Err(CommonMeasurementError::Length);
        }
        let measurement = match u16::from_le_bytes(value[0..2].try_into().unwrap()) {
            ACCEL_CODE => {
                let (x, y, z) = parse_xyz(value);
                CommonMeasurement::Accel(x, y, z)
            }
            GYRO_CODE => {
                let (x, y, z) = parse_xyz(value);
                CommonMeasurement::Gyro(x, y, z)
            }
            MAG_CODE => {
                let (x, y, z) = parse_xyz(value);
                CommonMeasurement::Mag(x, y, z)
            }
            TEMP_CODE => {
                let (label, value) = parse_label_value(value);
                CommonMeasurement::Temp(label, value)
            }
            BARO_CODE => {
                let (t, p, a) = parse_xyz(value);
                CommonMeasurement::Baro(t, p, a)
            }
            HUMI_CODE => {
                let (t, h, a) = parse_xyz(value);
                CommonMeasurement::Humi(t, h, a)
            }
            LUX_CODE => {
                let (label, value) = parse_label_value(value);
                CommonMeasurement::Lux(label, value)
            }
            _ => return Err(CommonMeasurementError::Type),
        };
        Ok(measurement)
    }
}

/// Size of a single measurement in bytes, as returned by [`SingleMeasurement::into`]
pub const SINGLE_MEASUREMENT_SIZE: usize = 24;

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
/// A single measurement with a timestamp
pub struct SingleMeasurement {
    /// Measurement data
    pub measurement: CommonMeasurement,
    /// Timestamp in microseconds
    /// Note: This timestamp is relative to an arbitrary epoch, e.g., system start time.
    pub timestamp: u64,
}

#[allow(clippy::from_over_into)]
impl Into<[u8; SINGLE_MEASUREMENT_SIZE]> for SingleMeasurement {
    #[inline]
    fn into(self) -> [u8; SINGLE_MEASUREMENT_SIZE] {
        self.into_bytes()
    }
}

impl SingleMeasurement {
    /// Represents the single measurement as a byte array.
    /// This is a form of binary serialization that includes a CRC for integrity checking.
    #[inline]
    fn into_bytes(self) -> [u8; SINGLE_MEASUREMENT_SIZE] {
        // Byte map:
        // [2 byte TYPE] [12 bytes measurement data] [8 bytes timestamp] [2 bytes CRC]
        let mut bytes = [0u8; SINGLE_MEASUREMENT_SIZE];
        bytes[0..COMMON_MEASUREMENT_SIZE].copy_from_slice(&self.measurement.into_bytes());
        bytes[COMMON_MEASUREMENT_SIZE..COMMON_MEASUREMENT_SIZE + 8]
            .copy_from_slice(&self.timestamp.to_le_bytes());
        let crc = CrcState::calculate(&bytes[0..SINGLE_MEASUREMENT_SIZE - 2]);
        bytes[SINGLE_MEASUREMENT_SIZE - 2..].copy_from_slice(&crc.to_le_bytes());
        bytes
    }
}

impl TryFrom<&[u8]> for SingleMeasurement {
    type Error = CommonMeasurementError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != SINGLE_MEASUREMENT_SIZE {
            return Err(CommonMeasurementError::Length);
        }
        let received_crc = u16::from_le_bytes(
            value[SINGLE_MEASUREMENT_SIZE - 2..SINGLE_MEASUREMENT_SIZE]
                .try_into()
                .unwrap(),
        );
        let computed_crc = CrcState::calculate(&value[0..SINGLE_MEASUREMENT_SIZE - 2]);
        if received_crc != computed_crc {
            return Err(CommonMeasurementError::CrcMismatch);
        }
        let measurement = CommonMeasurement::try_from(&value[0..COMMON_MEASUREMENT_SIZE])?;
        let timestamp = u64::from_le_bytes(
            value[COMMON_MEASUREMENT_SIZE..COMMON_MEASUREMENT_SIZE + 8]
                .try_into()
                .unwrap(),
        );
        Ok(SingleMeasurement {
            measurement,
            timestamp,
        })
    }
}

fn parse_xyz(value: &[u8]) -> (f32, f32, f32) {
    let x = f32::from_le_bytes(value[2..6].try_into().unwrap_or([0u8; 4]));
    let y = f32::from_le_bytes(value[6..10].try_into().unwrap_or([0u8; 4]));
    let z = f32::from_le_bytes(
        value[10..COMMON_MEASUREMENT_SIZE]
            .try_into()
            .unwrap_or([0u8; 4]),
    );
    (x, y, z)
}

fn parse_label_value(value: &[u8]) -> (String<8>, f32) {
    let label_end = value[2..10].iter().position(|&b| b == 0).unwrap_or(8);
    let label: String<8> =
        String::try_from(core::str::from_utf8(&value[2..2 + label_end]).unwrap_or("Unknown"))
            .unwrap_or_else(|_| String::new());
    let val = f32::from_le_bytes(
        value[10..COMMON_MEASUREMENT_SIZE]
            .try_into()
            .unwrap_or([0u8; 4]),
    );
    (label, val)
}

fn encode_xyz(code: u16, x: f32, y: f32, z: f32) -> [u8; COMMON_MEASUREMENT_SIZE] {
    let mut bytes = [0u8; COMMON_MEASUREMENT_SIZE];
    bytes[0..2].copy_from_slice(&code.to_le_bytes());
    bytes[2..6].copy_from_slice(&x.to_le_bytes());
    bytes[6..10].copy_from_slice(&y.to_le_bytes());
    bytes[10..COMMON_MEASUREMENT_SIZE].copy_from_slice(&z.to_le_bytes());
    bytes
}

fn encode_label_value(code: u16, label: &str, value: f32) -> [u8; COMMON_MEASUREMENT_SIZE] {
    let mut bytes = [0u8; COMMON_MEASUREMENT_SIZE];
    bytes[0..2].copy_from_slice(&code.to_le_bytes());
    let label_bytes = label.as_bytes();
    let len = label_bytes.len().min(8);
    bytes[2..2 + len].copy_from_slice(&label_bytes[..len]);
    bytes[10..COMMON_MEASUREMENT_SIZE].copy_from_slice(&value.to_le_bytes());
    bytes
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    #[test]
    fn test_common_measurement_serialization() {
        let original = CommonMeasurement::Temp(heapless::String::try_from("CPU").unwrap(), 36.5);
        let bytes: [u8; _] = original.clone().into();
        let deserialized = CommonMeasurement::try_from(&bytes[..]).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_single_measurement_serialization() {
        let original = SingleMeasurement {
            measurement: CommonMeasurement::Accel(0.1, 0.2, 0.3),
            timestamp: 123456789,
        };
        let bytes: [u8; _] = original.clone().into();
        let deserialized = SingleMeasurement::try_from(&bytes[..]).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_single_measurement_crc_mismatch() {
        let original = SingleMeasurement {
            measurement: CommonMeasurement::Gyro(1.0, 2.0, 3.0),
            timestamp: 987654321,
        };
        let mut bytes: [u8; _] = original.into();
        // Corrupt a byte to simulate CRC mismatch
        bytes[5] ^= 0xFF;
        let result = SingleMeasurement::try_from(&bytes[..]);
        assert!(matches!(result, Err(CommonMeasurementError::CrcMismatch)));
    }

    #[test]
    fn test_common_measurement_humi() {
        let original = CommonMeasurement::Humi(25.0, 60.0, 42.0);
        let bytes: [u8; _] = original.clone().into();
        let deserialized = CommonMeasurement::try_from(&bytes[..]).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    fn test_common_measurement_lux() {
        let original = CommonMeasurement::Lux(heapless::String::try_from("Room").unwrap(), 350.0);
        let bytes: [u8; _] = original.clone().into();
        let deserialized = CommonMeasurement::try_from(&bytes[..]).unwrap();
        assert_eq!(original, deserialized);
    }

    #[test]
    #[cfg(feature = "serde")]
    fn test_single_measurement_serde_baro() {
        let original = SingleMeasurement {
            measurement: CommonMeasurement::Baro(25.0, 1013.25, 150.0),
            timestamp: 555555555,
        };
        let serialized = serde_json::to_string(&original).unwrap();
        let deserialized: SingleMeasurement = serde_json::from_str(&serialized).unwrap();
        std::println!("Serialized: {}", serialized);
        assert_eq!(original, deserialized);
    }

    #[test]
    #[cfg(feature = "serde")]
    fn test_single_measurement_serde_temp() {
        let original = SingleMeasurement {
            measurement: CommonMeasurement::Temp(heapless::String::try_from("CPU").unwrap(), 36.5),
            timestamp: 123456789,
        };
        let serialized = serde_json::to_string(&original).unwrap();
        let deserialized: SingleMeasurement = serde_json::from_str(&serialized).unwrap();
        std::println!("Serialized: {}", serialized);
        assert_eq!(original, deserialized);
    }
}
