use super::Location;

#[derive(Clone, Debug)]
pub struct Waypoint<T> {
    pub location: Location,
    pub radius: f32, // In meters
    pub data: Option<T>,
}

impl<T: Copy> Waypoint<T> {
    pub fn get_data(&self) -> Option<T> {
        self.data
    }
}

impl<T> Waypoint<T> {
    pub fn new(location: Location, radius: f32) -> Self {
        Self {
            location,
            radius,
            data: None,
        }
    }

    pub fn new_with_data(location: Location, radius: f32, data: T) -> Self {
        Waypoint {
            data: Some(data),
            ..Self::new(location, radius)
        }
    }

    pub fn from_degrees(lon: f64, lat: f64, alt: f32, radius: f32) -> Self {
        Self::new(Location::from_degrees(lon, lat, alt), radius)
    }

    pub fn from_radians(lon: f64, lat: f64, alt: f32, radius: f32) -> Self {
        Self::new(Location::from_radians(lon, lat, alt), radius)
    }

    pub fn extend(&self, mut location: Location, alt: f32) -> Self {
        location.alt = alt.into();
        Self::new(location, self.radius)
    }

    pub fn add_data<U>(self, data: U) -> Waypoint<U> {
        Waypoint::<U>::new_with_data(self.location, self.radius, data)
    }

    pub fn set_data(&mut self, data: T) -> &mut Self {
        self.data = Some(data);
        self
    }
}

impl PartialEq for Waypoint<()> {
    fn eq(&self, other: &Self) -> bool {
        // data is being ignored for now because nothing uses it currently TODO: include data
        self.location.eq(&other.location) && self.radius == other.radius
    }

    fn ne(&self, other: &Self) -> bool {
        // data is being ignored for now because nothing uses it currently TODO: include data
        self.location.ne(&other.location) || self.radius != other.radius
    }
}

impl Eq for Waypoint<()> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eq() {
        let waypoint1 = Waypoint::from_degrees(1f64, 2f64, 10f32, 20f32);
        let waypoint2 = Waypoint::from(waypoint1.clone());
        let waypoint3 = Waypoint::from_degrees(1f64, 2f64, 10f32, 20f32);
        assert!(waypoint1.eq(&waypoint2) && waypoint2.eq(&waypoint1));
        assert!(waypoint2.eq(&waypoint3) && waypoint3.eq(&waypoint2));
        assert!(waypoint1.eq(&waypoint3) && waypoint3.eq(&waypoint1));
    }

    #[test]
    fn test_ne() {
        let waypoint1 = Waypoint::from_degrees(1f64, 2f64, 10f32, 20f32);
        let waypoint2 = Waypoint::from_degrees(2f64, 1f64, 10f32, 20f32);
        assert!(waypoint1.ne(&waypoint2) && waypoint2.ne(&waypoint1));
    }
}
