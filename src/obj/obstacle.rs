use super::Location;

#[derive(Debug, Clone)]
pub struct Obstacle {
    pub location: Location,
    pub radius: f32, // In meters
    pub height: f32, // In meters
}

// #TODO: swap radius and height to be consistent with waypoint
// #TODO: remove height because redundant? can directly use location z
impl Obstacle {
    pub fn new(location: Location, radius: f32, height: f32) -> Self {
        Self {
            location,
            radius,
            height,
        }
    }
    pub fn from_degrees(lon: f64, lat: f64, radius: f32, height: f32) -> Self {
        Self::new(Location::from_degrees(lon, lat, height), radius, height)
    }

    pub fn from_radians(lon: f64, lat: f64, radius: f32, height: f32) -> Self {
        Self::new(Location::from_radians(lon, lat, height), radius, height)
    }
}

impl PartialEq for Obstacle {
    fn eq(&self, other: &Self) -> bool {
        self.location.eq(&other.location)
            && (self.radius, self.height) == (other.radius, other.height)
    }

    fn ne(&self, other: &Self) -> bool {
        self.location.ne(&other.location)
            || (self.radius, self.height) != (other.radius, other.height)
    }
}

impl Eq for Obstacle {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eq() {
        let obstacle1 = Obstacle::from_degrees(1f64, 2f64, 10f32, 20f32);
        let obstacle2 = Obstacle::from(obstacle1.clone());
        let obstacle3 = Obstacle::from_degrees(1f64, 2f64, 10f32, 20f32);
        assert!(obstacle1.eq(&obstacle2) && obstacle2.eq(&obstacle1));
        assert!(obstacle2.eq(&obstacle3) && obstacle3.eq(&obstacle2));
        assert!(obstacle1.eq(&obstacle3) && obstacle3.eq(&obstacle1));
    }

    #[test]
    fn test_ne() {
        let obstacle1 = Obstacle::from_degrees(1f64, 2f64, 10f32, 20f32);
        let obstacle2 = Obstacle::from_degrees(2f64, 1f64, 10f32, 20f32);
        assert!(obstacle1.ne(&obstacle2) && obstacle2.ne(&obstacle1));
    }
}
