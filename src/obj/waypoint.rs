use super::Location;

/// A struct for storing points on the path
///
/// 'Waypoint's are the positions that make up the plane's path and are connected by straight lines.
/// They are similar to ['Obstacle']s in that they are considered as a cylinder from the ground up
/// by the pathfinder, but when the final path is generated their altitude is used.
#[derive(Clone, Debug)]
pub struct Waypoint<T> {
    /// Position as a ['Location']
    pub location: Location,
    /// Radius of cylinder in meters
    pub radius: f32,
    /// An optional field containing extra data about the 'Waypoint'
    ///
    /// This is generally left as 'None'.
    // #TODO: Find out if data is ever used and/or is important
    pub data: Option<T>,
}

impl<T: Copy> Waypoint<T> {
    /// Get the 'data' if there is any, returns 'None' if not
    pub fn get_data(&self) -> Option<T> {
        self.data
    }
}

impl<T> Waypoint<T> {
    /// Create a 'Waypoint' from a ['Location'] and a radius
    pub fn new(location: Location, radius: f32) -> Self {
        Self {
            location,
            radius,
            data: None,
        }
    }

    /// Create a 'Waypoint' from a ['Location'], a radius, and data of some type
    pub fn new_with_data(location: Location, radius: f32, data: T) -> Self {
        Waypoint {
            data: Some(data),
            ..Self::new(location, radius)
        }
    }

    /// Create a 'Waypoint' from a longitude (degrees), a latitude (degrees), an altitude, and a radius
    pub fn from_degrees(lon: f64, lat: f64, alt: f32, radius: f32) -> Self {
        Self::new(Location::from_degrees(lon, lat, alt), radius)
    }

    /// Create a 'Waypoint' from a longitude (radians), a latitude (radians), an altitude, and a radius
    pub fn from_radians(lon: f64, lat: f64, alt: f32, radius: f32) -> Self {
        Self::new(Location::from_radians(lon, lat, alt), radius)
    }

    /// Create a new 'Waypoint' using this one's radius
    ///
    /// The altitude of the passed ['Location'] is replaced with the passed altitude.
    // #TODO: Explain why this is called extend and what purpose it has
    pub fn extend(&self, mut location: Location, alt: f32) -> Self {
        location.alt = alt.into();
        Self::new(location, self.radius)
    }

    /// Duplicate this 'Waypoint' but give the new one some passed data
    pub fn add_data<U>(self, data: U) -> Waypoint<U> {
        Waypoint::<U>::new_with_data(self.location, self.radius, data)
    }

    /// Change the data of this 'Waypoint' and return it
    pub fn set_data(&mut self, data: T) -> &mut Self {
        self.data = Some(data);
        self
    }
}
