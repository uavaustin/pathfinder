use super::*;

impl Waypoint {
    pub fn new(index: u32, location: Location, radius: f32) -> Self {
        Waypoint {
            index: index,
            location: location,
            radius: radius,
        }
    }

    pub fn from_degrees(index: u32, lat: f64, lon: f64, alt: f32, radius: f32) -> Self {
        Waypoint::new(index, Location::from_degrees(lat, lon, alt), radius)
    }

    pub fn from_radians(index: u32, lat: f64, lon: f64, alt: f32, radius: f32) -> Self {
        Waypoint::new(index, Location::from_radians(lat, lon, alt), radius)
    }

    pub fn extend(&self, mut location: Location, alt: f32) -> Self {
        location.alt = alt.into();
        Waypoint::new(self.index, location, self.radius)
    }
}
