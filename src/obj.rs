use super::{ordered_float::OrderedFloat, std};
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct Point {
    lat: OrderedFloat<f64>, //In radians
    lon: OrderedFloat<f64>, //In radians
    alt: OrderedFloat<f32>, //In meters
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({:.5}, {:.5})", self.lat_degree(), self.lon_degree())
    }
}

impl Point {
    pub fn from_degrees(lat: f64, lon: f64, alt: f32) -> Self {
        const FACTOR: f64 = std::f64::consts::PI / 180f64;
        Point {
            lat: lat.to_radians().into(),
            lon: lon.to_radians().into(),
            alt: alt.into(),
        }
    }
    pub fn from_radians(lat: f64, lon: f64, alt: f32) -> Self {
        Point {
            lat: lat.into(),
            lon: lon.into(),
            alt: alt.into(),
        }
    }
    pub fn lat(&self) -> f64 {
        self.lat.into()
    }
    pub fn lon(&self) -> f64 {
        self.lon.into()
    }
    pub fn alt(&self) -> f32 {
        self.alt.into()
    }
    pub fn lat_degree(&self) -> f64 {
        Into::<f64>::into(self.lat) * 180f64 / std::f64::consts::PI
    }
    pub fn lon_degree(&self) -> f64 {
        Into::<f64>::into(self.lon) * 180f64 / std::f64::consts::PI
    }
}

#[derive(Debug, Clone)]
pub struct Obstacle {
    pub coords: Point,
    pub radius: f32, // In meters
    pub height: f32, // In meters
}

impl Obstacle {
    pub fn from_degrees(lat: f64, lon: f64, radius: f32, height: f32) -> Self {
        Obstacle {
            coords: Point::from_degrees(lat, lon, 0f32),
            radius: radius,
            height: height,
        }
    }

    pub fn from_radians(lat: f64, lon: f64, radius: f32, height: f32) -> Self {
        Obstacle {
            coords: Point::from_radians(lat, lon, 0f32),
            radius: radius,
            height: height,
        }
    }
}

// #TODO: standarize location name
// #TODO: fully implement builder pattern for greater flexibility
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub location: Point,
    pub yaw: f32,         // In degrees, -1 if not provided
    pub pitch: f32,       // In degrees, -1 if not provided
    pub roll: f32,        // In degrees, -1 if not provided
    pub airspeed: f32,    // In meters per second, -1 if not provided
    pub groundspeed: f32, // In meters per second, -1 if not provided
    pub wind_dir: f32,    // In degrees, -1 if not provided
}

impl Plane {
    pub fn new(location: Point) -> Self {
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

    pub fn from_degrees(lat: f64, lon: f64, alt: f32) -> Self {
        Plane::new(Point::from_degrees(lat, lon, alt))
    }

    pub fn from_radians(lat: f64, lon: f64, alt: f32) -> Self {
        Plane::new(Point::from_radians(lat, lon, alt))
    }

    pub fn yaw(mut self, yaw: f32) -> Self {
        if yaw >= 0f32 && yaw < 360f32 {
            self.yaw = yaw;
        }
        self
    }
}

#[derive(Clone, Debug)]
pub struct Waypoint {
    pub index: u32,
    pub location: Point,
    pub radius: f32, // In meters
}

impl Waypoint {
    pub fn new(index: u32, location: Point, radius: f32) -> Self {
        Waypoint {
            index: index,
            location: location,
            radius: radius,
        }
    }

    pub fn from_degrees(index: u32, lat: f64, lon: f64, alt: f32, radius: f32) -> Self {
        Waypoint::new(index, Point::from_degrees(lat, lon, alt), radius)
    }

    pub fn from_radians(index: u32, lat: f64, lon: f64, alt: f32, radius: f32) -> Self {
        Waypoint::new(index, Point::from_radians(lat, lon, alt), radius)
    }

    pub fn extend(&self, mut location: Point, alt: f32) -> Self {
        location.alt = alt.into();
        Waypoint::new(self.index, location, self.radius)
    }
}
