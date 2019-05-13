use super::*;

use graph::Point;
use std::fmt;

impl fmt::Display for Location {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({:.5}, {:.5})", self.lat_degree(), self.lon_degree())
    }
}

impl From<(&Point, &Location)> for Location {
    // Convert point with respect to origin to location
    fn from((point, origin): (&Point, &Location)) -> Self {
        let lat = point.y as f64 / RADIUS + origin.lat();
        let lon = ((point.x as f64 / RADIUS / 2f64).sin() / lat.cos()).asin() * 2f64 + origin.lon();
        Self::from_radians(lat, lon, point.z)
    }
}

impl Location {
    // Create location from coordinates in degrees
    pub fn from_degrees(lat: f64, lon: f64, alt: f32) -> Self {
        const FACTOR: f64 = ::std::f64::consts::PI / 180f64;
        Location {
            lat: lat.to_radians().into(),
            lon: lon.to_radians().into(),
            alt: alt.into(),
        }
    }
    // Create location from coordinates in radians
    pub fn from_radians(lat: f64, lon: f64, alt: f32) -> Self {
        Location {
            lat: lat.into(),
            lon: lon.into(),
            alt: alt.into(),
        }
    }
    // Create location using x-y distance from origin
    pub fn from_meters(x: f32, y: f32, alt: f32, origin: &Location) -> Self {
        (&Point::new(x, y, alt), origin).into()
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
