use super::*;

impl Point {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Point { x: x, y: y, z: z }
    }

    // Creates a point from a location and reference point
    pub fn from_location(location: &Location, origin: &Location) -> Self {
        Point::new(
            (2f64
                * RADIUS
                * (location.lat().cos() * ((location.lon() - origin.lon()) / 2f64).sin()).asin())
                as f32,
            (RADIUS * (location.lat() - origin.lat())) as f32,
            location.alt(),
        )
    }

    // Convert point with respect to origin to location
    pub fn to_location(&self, origin: &Location) -> Location {
        let lat = self.y as f64 / RADIUS + origin.lat();
        let lon = ((self.x as f64 / RADIUS / 2f64).sin() / lat.cos()).asin() * 2f64 + origin.lon();
        Location::from_radians(lat, lon, self.z)
    }

    pub fn distance(&self, other: &Point) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt() as f32
    }
}
