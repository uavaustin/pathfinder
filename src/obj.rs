#[derive(Clone)]
pub struct Point {
  lat: f32,  //In degrees
  lon: f32,  //In degrees
}

pub struct Obstacle {
	pub coords: Point,
	pub radius: f32,   // In meters
	pub height: f32,   // In meters
}

pub struct Plane {
	pub coords: Point,
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
	pub flyzone_points: Vec<Point>,
}
impl PathFinder
{
  //TODO
  fn adjust_path(self) -> Vec<Waypoint> {
    self.wp_list.clone()
  }
  
  fn set_x(&mut self, new_x: u32){
    self.delta_x = new_x;
  }

  fn set_waypoint_list(&mut self, list: Vec<Waypoint>) {
    self.wp_list = list;
  }
}

fn main()
{
  let mut point : Point = Point{lat: 0.0, lon: 0.0};
  let mut point2 : Point = Point{lat: 0.0, lon: 0.0};
  let mut point3 : Point = Point{lat: 0.0, lon: 0.0};
  let mut vec : Vec<Waypoint> = vec![Waypoint{index: 0, coords: point, alt: 0.0, radius: 0.0}];
  let mut obstacles : Vec<Obstacle> = vec![Obstacle{coords: point2, radius: 0.0, height: 0.0}];
  let mut flyzones : Vec<Point> = vec![point3];
  let mut plane : Plane = Plane::new(0.0,0.0,0.0);
  //let mut vec2 : Vec<Waypoint> = vec![Waypoint{}];
  let mut f : PathFinder = PathFinder{delta_x: 0, buffer: 0, max_process_time: 0, plane: plane,obstacle_list: obstacles, wp_list : vec, flyzone_points : flyzones};
  f.set_x(5);
}
