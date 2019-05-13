use ordered_float::OrderedFloat;
use super::*;

mod location;
mod obstacle;
mod plane;
mod waypoint;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct Location {
    lat: OrderedFloat<f64>, //In radians
    lon: OrderedFloat<f64>, //In radians
    alt: OrderedFloat<f32>, //In meters
}

#[derive(Debug, Clone)]
pub struct Obstacle {
    pub location: Location,
    pub radius: f32, // In meters
    pub height: f32, // In meters
}

// #TODO: standarize location name
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

#[derive(Clone, Debug)]
pub struct Waypoint {
    pub index: u32,
    pub location: Location,
    pub radius: f32, // In meters
}
