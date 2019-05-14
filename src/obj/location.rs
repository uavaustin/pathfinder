extern crate ordered_float;

use self::ordered_float::*;
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct Location {
    lat: OrderedFloat<f64>,     //In radians
    lon: OrderedFloat<f64>,     //In radians
    pub alt: OrderedFloat<f32>, //In meters
}

impl fmt::Display for Location {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({:.5}, {:.5})", self.lat_degree(), self.lon_degree())
    }
}

impl Location {
    // Create location from coordinates in degrees
    pub fn from_degrees(lat: f64, lon: f64, alt: f32) -> Self {
        Self {
            lat: lat.to_radians().into(),
            lon: lon.to_radians().into(),
            alt: alt.into(),
        }
    }
    // Create location from coordinates in radians
    pub fn from_radians(lat: f64, lon: f64, alt: f32) -> Self {
        Self {
            lat: lat.into(),
            lon: lon.into(),
            alt: alt.into(),
        }
    }

    pub fn lat(&self) -> f64 {
        self.lat.into()
    }
    pub fn lon(&self) -> f64 {
        self.lon.into()
    }
    pub fn alt(&self) -> f32 {
        self.alt.into()
    }
    pub fn lat_degree(&self) -> f64 {
        Into::<f64>::into(self.lat) * 180f64 / ::std::f64::consts::PI
    }
    pub fn lon_degree(&self) -> f64 {
        Into::<f64>::into(self.lon) * 180f64 / ::std::f64::consts::PI
    }
}
