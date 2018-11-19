use super::*;

impl Plane {
    pub fn new(location: Location) -> Self {
        Plane {
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
        Plane::new(Location::from_degrees(lon, lat, alt))
    }

    pub fn from_radians(lon: f64, lat: f64, alt: f32) -> Self {
        Plane::new(Location::from_radians(lon, lat, alt))
    }

    pub fn yaw(mut self, yaw: f32) -> Self {
        if yaw >= 0f32 && yaw < 360f32 {
            self.yaw = yaw;
        }
        self
    }
}
