#![allow(dead_code)]

pub struct Obstacle {
	pub lat: f32,  // In degrees
	pub lon: f32,  // In degrees
	pub radius: f32,   // In meters
	pub height: f32,   // In meters
}

pub struct Plane {
	pub lat: f32,  // In degrees
	pub lon: f32,  // In degrees
	pub alt: f32,  // In degrees
	pub yaw: f32,  // In degrees
	pub pitch: f32,    // In degrees
	pub roll: f32, // In degrees
	pub airspeed: f32, // In meters per second
	pub groundspeed: f32,  // In meters per second
	pub wind_dir: f32, // In degrees
}

impl Plane {
    pub fn new(lat:f32, lon:f32, alt:f32) -> Plane {
        Plane{
            lat: lat,
            lon: lon,
            alt: alt,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
            airspeed: 0.0,
            groundspeed: 0.0,
            wind_dir: 0.0
        }
    }
}

pub struct Waypoint {
    pub index: i32,
	pub lat: f32,  // In degrees
	pub lon: f32,  // In degrees
	pub alt: f32,  // In degrees
	pub radius: f32,   // In meters
}

pub struct PathFinder {
	pub delta_x: u32,   // In meters
	pub buffer: u32,   // In meters
	pub max_process_time: u32,   // In seconds
	pub plane: Plane,
	pub obstacle_list: Vec<Obstacle>,
	pub wp_list: Vec<Waypoint>,
}
