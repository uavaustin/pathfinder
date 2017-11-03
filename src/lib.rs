#![allow(dead_code)]
#![allow(unused_variables)]

use std::collections::HashSet;

mod obj;
use obj::*;

const EQUATORIAL_RADIUS:f32 = 63781370.0;
const POLAR_RADIUS:f32 = 6356752.0;
const RADIUS:f32 = 6371000.0;

#[derive(Hash, Clone, Copy)]
pub struct Node {
    x: i32,
    y: i32,
    cost: u32,
}

impl PartialEq for Node {
    fn eq(&self, other: &Node) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Eq for Node {}

pub struct PathFinder {
	pub grid_size: u32,   // In meters
	pub buffer: u32,   // In meters
	pub max_process_time: u32,   // In seconds
	pub origin: Point,
	pub plane: Plane,
	pub obstacle_list: HashSet<Node>,
	pub wp_list: Vec<Waypoint>,
    offset: [(i32,i32); 8]
}

impl PathFinder{
	fn new(flyzone_points: Vec<Point>) -> PathFinder {
        assert!(flyzone_points.len() > 2);
		let origin = PathFinder::find_origin(&flyzone_points);
		let mut new_path_finder = PathFinder {
			grid_size: 1,
			buffer: 1,
			max_process_time: 10,
			origin: origin,
			plane: Plane::new(0.0,0.0,0.0),
			obstacle_list: HashSet::new(),
			wp_list: Vec::new(),
			offset: [(-1,-1),(-1,0),(-1,1),(1,-1),(1,0),(1,1),(0,-1),(0,1)]
		};
		new_path_finder.initialize(&flyzone_points);
		new_path_finder
	}

	fn find_origin(flyzone_points: &Vec<Point>) -> Point {
        let mut min_lat = flyzone_points[0].lat;
        let mut min_lon = flyzone_points[0].lon;
        let mut max_lon = flyzone_points[0].lon;
        let flyzone_points = &flyzone_points[1..];
        for point in flyzone_points {
            if point.lat < min_lat {
                min_lat = point.lat;
            }
            if point.lon < min_lon {
                min_lon = point.lon;
            }
            if point.lon > max_lon {
                max_lon = point.lon;
            }
        }
        let mut lon = min_lon;
        if max_lon - min_lon > 180.0 {
            lon = max_lon;
        }

        Point{lat: min_lat, lon: lon}.to_rad()
	}

    fn normalize(&self, point: &Point) -> Node {
        let x = 2.0*RADIUS*(point.lat.cos()*((point.lon-self.origin.lon)/2.0).sin()).asin();
        let y = 2.0*RADIUS*((point.lat-self.origin.lat)/2.0);
        Node{x: x.floor() as i32, y: y.floor() as i32, cost: 0}
    }

	fn initialize(&mut self, flyzone_points: &Vec<Point>) {
		// Initialize boundry obstacles
        for point in flyzone_points {
            let node = self.normalize(&(point.to_rad()));
            self.obstacle_list.insert(node);
        }
	}

	pub fn adjust_path(&mut self, plane: Plane) -> Vec<Waypoint> {
	    self.plane = plane;

	    self.wp_list.clone()
 	}

	pub fn set_grid_size(&mut self, grid_size: u32){
		self.grid_size = grid_size;
		let grid_size = grid_size as i32;
		self.offset = [
		    (-grid_size,-grid_size),(-grid_size,0),(-grid_size,grid_size),
		    (grid_size,-grid_size),(grid_size,0),(grid_size,grid_size),
		    (0,-grid_size),(0, grid_size)
		    ];
		}

  pub fn set_waypoint_list(&mut self, list: Vec<Waypoint>) {
		self.wp_list = list;
  }
	
  pub fn set_buffer( &mut self, new_buffer : u32 ) -> (){
    self.buffer = new_buffer;
  }
  pub fn set_process_time( &mut self, new_time : u32 ) -> (){
    self.max_process_time = new_time;
  }
  pub fn set_plane( &mut self, new_plane : Plane ) -> (){
    self.plane = new_plane;
  }
  pub fn set_obstacle_list( &mut self, new_obstacle_list : HashSet<Node> ) -> (){
    self.obstacle_list = new_obstacle_list;
  }
  pub fn get_buffer(self) -> u32{
    return self.buffer
  }
  pub fn get_process_time(self) -> u32{
    return self.max_process_time 
  }
  pub fn get_plane(self) -> Plane{
    return self.plane
  }
  pub fn get_obstacle_list(self) -> HashSet<Node>{
    return self.obstacle_list
  }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hav_test1() {
        println!("---------------");
        println!("test1");
        let path_finder1 = PathFinder::new(vec!(
            Point{lat:50.06638888888889,lon:-5.714722222222222},
            Point{lat:58.64388888888889,lon:-5.714722222222222},
            Point{lat:50.06638888888889,lon:-3.0700000000000003}
        ));
        for node in path_finder1.obstacle_list {
            println!("x: {} y: {}", node.x, node.y);
        }
    }
}
