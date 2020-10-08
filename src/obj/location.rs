extern crate ordered_float;

use self::ordered_float::*;
use std::fmt;

/// A struct for storing a position in world coordinates
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct Location {
    /// Latitude in radians
    lat: OrderedFloat<f64>,
    /// Longitude in radians
    lon: OrderedFloat<f64>,
    /// Altitude in meters
    pub alt: OrderedFloat<f32>,
}

impl fmt::Display for Location {
    /// Formats the location for display using the given formatter
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "({:.5}, {:.5}, {:.5})",
            self.lat_degree(),
            self.lon_degree(),
            self.alt()
        )
    }
}

impl Location {
    /// Create location from coordinates in degrees
    pub fn from_degrees(lat: f64, lon: f64, alt: f32) -> Self {
        Self {
            lat: lat.to_radians().into(),
            lon: lon.to_radians().into(),
            alt: alt.into(),
        }
    }
    /// Create location from coordinates in radians
    pub fn from_radians(lat: f64, lon: f64, alt: f32) -> Self {
        Self {
            lat: lat.into(),
            lon: lon.into(),
            alt: alt.into(),
        }
    }

    /// Get latitude in radians
    pub fn lat(&self) -> f64 {
        self.lat.into()
    }
    /// Get longitude in radians
    pub fn lon(&self) -> f64 {
        self.lon.into()
    }
    /// Get altitude in radians
    pub fn alt(&self) -> f32 {
        self.alt.into()
    }
    /// Get latitude in degrees
    pub fn lat_degree(&self) -> f64 {
        Into::<f64>::into(self.lat) * 180f64 / ::std::f64::consts::PI
    }
    /// Get longitude in degrees
    pub fn lon_degree(&self) -> f64 {
        Into::<f64>::into(self.lon) * 180f64 / ::std::f64::consts::PI
    }
}
