use super::*;

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
            location: location,
            radius: radius,
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
