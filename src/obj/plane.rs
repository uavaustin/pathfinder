use super::Location;

// #TODO: standardize location name
// #TODO: fully implement builder pattern for greater flexibility
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub location: Location,
    pub yaw: f32,         // In degrees, -1 if not provided
    pub pitch: f32,       // In degrees, -1 if not provided
    pub roll: f32,        // In degrees, -1 if not provided
    pub airspeed: f32,    // In meters per second, -1 if not provided
    pub groundspeed: f32, // In meters per second, -1 if not provided
    pub wind_dir: f32,    // In degrees, -1 if not provided
}

impl Plane {
    pub fn new(location: Location) -> Self {
        Self {
            location: location,
            yaw: -1f32,
            pitch: -1f32,
            roll: -1f32,
            airspeed: -1f32,
            groundspeed: -1f32,
            wind_dir: -1f32,
        }
    }

    pub fn from_degrees(lon: f64, lat: f64, alt: f32) -> Self {
        Self::new(Location::from_degrees(lon, lat, alt))
    }

    pub fn from_radians(lon: f64, lat: f64, alt: f32) -> Self {
        Self::new(Location::from_radians(lon, lat, alt))
    }

    pub fn yaw(mut self, yaw: f32) -> Self {
        if yaw >= 0f32 && yaw < 360f32 {
            self.yaw = yaw;
        }
        self
    }
}
