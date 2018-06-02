use super::*;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    lat: f64,  //In radians
    lon: f64,  //In radians
    alt: f32,  //In meters
}

impl Point {
    pub fn from_degrees(lat:f64, lon:f64, alt: f32) -> Point {
        const FACTOR:f64 = std::f64::consts::PI/180f64;
        Point{lat: lat*FACTOR, lon: lon*FACTOR, alt: alt}
    }
    pub fn from_radians(lat:f64, lon:f64, alt: f32) -> Point{
        Point{lat: lat, lon: lon, alt: alt}
    }

    pub fn to_node(&self, path_finder: &PathFinder) -> Node {
        let origin = path_finder.origin;
        let x = 2f64*RADIUS*(self.lat.cos()*((self.lon-origin.lon)/2f64).sin()).asin();
        let y = RADIUS*(self.lat-origin.lat);
        Node::new((x/path_finder.grid_size as f64).floor() as i32,
            (y/path_finder.grid_size as f64).floor() as i32)
    }
    pub fn lat(&self) -> f64 {
        self.lat
    }
    pub fn lon(&self) -> f64 {
        self.lon
    }
    pub fn alt(&self) -> f32 {
        self.alt
    }
    pub fn lat_degree(&self) -> f64 {
        self.lat * 180f64 / std::f64::consts::PI
    }
    pub fn lon_degree(&self) -> f64 {
        self.lon * 180f64 / std::f64::consts::PI
    }
}

pub struct Obstacle {
	pub coords: Point,
	pub radius: f32,   // In meters
	pub height: f32,   // In meters
}

#[derive(Clone, Copy, Debug)]
pub struct Plane {
	pub location: Point,
	pub yaw: f32,  // In degrees
	pub pitch: f32,    // In degrees
	pub roll: f32, // In degrees
	pub airspeed: f32, // In meters per second
	pub groundspeed: f32,  // In meters per second
	pub wind_dir: f32, // In degrees
}

impl Plane {
    pub fn new(lat:f64, lon:f64, alt:f32) -> Plane {
        Plane {
            location: Point::from_degrees(lat, lon, alt),
            yaw: 0f32,
            pitch: 0f32,
            roll: 0f32,
            airspeed: 0f32,
            groundspeed: 0f32,
            wind_dir: 0f32,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Waypoint {
    pub index: u32,
	pub location: Point,
	pub radius: f32,   // In meters
}

impl Waypoint {
    pub fn new(index: u32, location: Point, radius: f32) -> Waypoint {
        Waypoint {
            index: index,
            location: location,
            radius: radius
        }
    }

    pub fn extend(&self, mut location: Point, alt: f32) -> Waypoint {
        location.alt = alt;
        Waypoint::new(
            self.index,
            location,
            self.radius
        )
    }
}
