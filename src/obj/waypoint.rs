use super::*;

#[derive(Clone, Debug)]
pub struct Waypoint<T> {
    pub index: u32,
    pub location: Location,
    pub radius: f32, // In meters
    pub data: Option<T>,
}

impl<T> Waypoint<T> {
    pub fn new(index: u32, location: Location, radius: f32) -> Self {
        Self {
            index: index,
            location: location,
            radius: radius,
            data: None,
        }
    }

    pub fn new_with_data(index: u32, location: Location, radius: f32, data: T) -> Self {
        Waypoint {
            data: Some(data),
            ..Self::new(index, location, radius)
        }
    }

    pub fn from_degrees(index: u32, lon: f64, lat: f64, alt: f32, radius: f32) -> Self {
        Self::new(index, Location::from_degrees(lon, lat, alt), radius)
    }

    pub fn from_radians(index: u32, lon: f64, lat: f64, alt: f32, radius: f32) -> Self {
        Self::new(index, Location::from_radians(lon, lat, alt), radius)
    }

    pub fn extend(&self, mut location: Location, alt: f32) -> Self {
        location.alt = alt.into();
        Self::new(self.index, location, self.radius)
    }

    pub fn add_data<U>(self, data: U) -> Waypoint<U> {
        Waypoint::<U>::new_with_data(self.index, self.location, self.radius, data)
    }

    pub fn set_data(&mut self, data: T) -> &mut Self {
        self.data = Some(data);
        self
    }
}

impl<T: Copy> Waypoint<T> {
    pub fn get_data(&self) -> Option<T> {
        self.data
    }
}
