#[derive(Clone)]
pub struct Point {
    pub lat: f32,  //In degrees
    pub lon: f32,  //In degrees
}

impl Point {
    pub fn to_rad(&self) -> Point{
        const FACTOR:f32 = ::std::f32::consts::PI/180.0;
        Point{lat: self.lat*FACTOR, lon: self.lon*FACTOR}
    }
}

pub struct Obstacle {
	pub coords: Point,
	pub radius: f32,   // In meters
	pub height: f32,   // In meters
}

pub struct Plane {
	pub coords: Point,
	pub alt: f32,  // In meters
	pub yaw: f32,  // In degrees
	pub pitch: f32,    // In degrees
	pub roll: f32, // In degrees
	pub airspeed: f32, // In meters per second
	pub groundspeed: f32,  // In meters per second
	pub wind_dir: f32, // In degrees
}

impl Plane {
    pub fn new(lat:f32, lon:f32, alt:f32) -> Plane {
        Plane {
            coords: Point{lat: lat, lon: lon},
            alt: alt,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            airspeed: 0.0,
            groundspeed: 0.0,
            wind_dir: 0.0,
        }
    }
}

#[derive(Clone)]
pub struct Waypoint {
    pub index: i32,
	pub coords: Point,
	pub alt: f32,  // In meters
	pub radius: f32,   // In meters
}
