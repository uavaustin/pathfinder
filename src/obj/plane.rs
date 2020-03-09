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
            location,
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

impl PartialEq for Plane {
    fn eq(&self, other: &Self) -> bool {
        self.location.eq(&other.location)
            && (
                self.yaw,
                self.pitch,
                self.roll,
                self.airspeed,
                self.groundspeed,
                self.wind_dir,
            ) == (
                other.yaw,
                other.pitch,
                other.roll,
                other.airspeed,
                other.groundspeed,
                other.wind_dir,
            )
    }

    fn ne(&self, other: &Self) -> bool {
        self.location.ne(&other.location)
            || (
                self.yaw,
                self.pitch,
                self.roll,
                self.airspeed,
                self.groundspeed,
                self.wind_dir,
            ) != (
                other.yaw,
                other.pitch,
                other.roll,
                other.airspeed,
                other.groundspeed,
                other.wind_dir,
            )
    }
}

impl Eq for Plane {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eq() {
        let plane1 = Plane::from_degrees(1f64, 2f64, 3f32);
        let plane2 = Plane::from(plane1.clone());
        let plane3 = Plane::from_degrees(1f64, 2f64, 3f32);
        assert!(plane1.eq(&plane2) && plane2.eq(&plane1));
        assert!(plane2.eq(&plane3) && plane3.eq(&plane2));
        assert!(plane1.eq(&plane3) && plane3.eq(&plane1));
    }

    #[test]
    fn test_ne() {
        let plane1 = Plane::from_degrees(1f64, 2f64, 3f32);
        let plane2 = Plane::from_degrees(3f64, 2f64, 1f32);
        assert!(plane1.ne(&plane2) && plane2.ne(&plane1));
    }
}
