use super::Location;

/// A struct for storing an obstacle's data
///
/// Obstacles are cylinders of a specified radius that reach from the
/// ground to a specified height.
#[derive(Debug, Clone)]
pub struct Obstacle {
    /// The position of the 'Obstacle' as a ['Location']
    pub location: Location,
    /// The radius of the 'Obstacle' in meters
    pub radius: f32,
    /// The height of the 'Obstacle' in meters
    pub height: f32,
}

// #TODO: swap radius and height to be consistent with waypoint
// #TODO: remove height because redundant? can directly use location z
// #TODO: update documentation to match other changes
impl Obstacle {
    /// Create an 'Obstacle' from a ['Location'], a radius, and a height
    pub fn new(location: Location, radius: f32, height: f32) -> Self {
        Self {
            location,
            radius,
            height,
        }
    }
    /// Create an 'Obstacle' from a longitude (degrees), a latitude (degrees),
    /// a radius, and a height
    pub fn from_degrees(lon: f64, lat: f64, radius: f32, height: f32) -> Self {
        Self::new(Location::from_degrees(lon, lat, height), radius, height)
    }
    /// Create an 'Obstacle' from a longitude (radians), a latitude (radians),
    /// a radius, and a height
    pub fn from_radians(lon: f64, lat: f64, radius: f32, height: f32) -> Self {
        Self::new(Location::from_radians(lon, lat, height), radius, height)
    }
}
