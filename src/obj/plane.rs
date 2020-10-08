use super::Location;

/// A struct for storing the plane's telemetry
///
/// #TODO: explain what fields mean (i.e. yaw, pitch, etc.) and how the physical plane compares to the virtual
// #TODO: standardize location name
// #TODO: fully implement builder pattern for greater flexibility
// #TODO: find out what handedness yaw, pitch, etc. are in
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    /// Position as a ['Location']
    pub location: Location,
    /// Rotation on the upward axis in degrees, -1 if not provided
    pub yaw: f32,
    /// Rotation on the wing axis in degrees, -1 if not provided
    pub pitch: f32,
    /// Rotation on the forward axis in degrees, -1 if not provided
    pub roll: f32,
    /// Airspeed in meters per second, -1 if not provided
    // #TODO: find out what this is and document
    pub airspeed: f32,
    /// Speed of the 'Plane' relative to the ground in meters per second, -1 if not provided
    pub groundspeed: f32,
    /// Direction of the wind as a rotation in degrees around the upward axis, -1 if not provided
    pub wind_dir: f32,
}

impl Plane {
    /// Create a new 'Plane' from a 'Location'
    pub fn new(location: Location) -> Self {
        Self {
            location,
            yaw: -1f32,
            pitch: -1f32,
            roll: -1f32,
            airspeed: -1f32,
            groundspeed: -1f32,
            wind_dir: -1f32,
        }
    }

    /// Create a 'Plane' from a longitude (degrees), a latitude (degrees), and an altitude
    pub fn from_degrees(lon: f64, lat: f64, alt: f32) -> Self {
        Self::new(Location::from_degrees(lon, lat, alt))
    }

    /// Create a 'Plane' from a longitude (radians), a latitude (radians), and an altitude
    pub fn from_radians(lon: f64, lat: f64, alt: f32) -> Self {
        Self::new(Location::from_radians(lon, lat, alt))
    }

    /// Modify this 'Plane''s yaw (in degrees) and return it
    ///
    /// This function handles over-rotation (i.e. rotations below 0 and at or above 360).
    pub fn yaw(mut self, yaw: f32) -> Self {
        if yaw >= 0f32 && yaw < 360f32 {
            self.yaw = yaw;
        }
        self
    }
}
