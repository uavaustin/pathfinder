use super::*;

impl Obstacle {
    pub fn new(location: Location, radius: f32, height: f32) -> Self {
        Obstacle {
            location: location,
            radius: radius,
            height: height,
        }
    }
    pub fn from_degrees(lat: f64, lon: f64, radius: f32, height: f32) -> Self {
        Obstacle::new(Location::from_degrees(lat, lon, height), radius, height)
    }

    pub fn from_radians(lat: f64, lon: f64, radius: f32, height: f32) -> Self {
        Obstacle::new(Location::from_radians(lat, lon, height), radius, height)
    }
}
