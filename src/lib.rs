#![allow(dead_code)]

mod obj;
use obj::*;

impl PathFinder{
    pub fn new() -> PathFinder {
        PathFinder {
            delta_x: 1,
            buffer: 1,
            max_process_time: 10,
            plane: Plane::new(0.0,0.0,0.0),
            obstacle_list: Vec::new(),
            wp_list: Vec::new(),
	    flyzone_points: Vec::new(),
        }
    }
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

#[cfg(test)]
mod tests {
	#[test]
	fn test() {
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
}
