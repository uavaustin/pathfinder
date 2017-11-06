#![allow(dead_code)]
#![allow(unused_variables)]

use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::BTreeSet;
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};

mod obj;
use self::obj::*;

const EQUATORIAL_RADIUS:f32 = 63781370.0;
const POLAR_RADIUS:f32 = 6356752.0;
const RADIUS:f32 = 6371000.0;

#[derive(Clone, Copy, Debug)]
struct Node {
    x: i32,
    y: i32,
    f_cost: f32
}

impl PartialEq for Node {
    fn eq(&self, other: &Node) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Node) -> Option<Ordering> {
        other.f_cost.partial_cmp(&self.f_cost)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Node) -> Ordering  {
        self.partial_cmp(other).unwrap()
    }
}

impl Eq for Node {
}

impl Node {
    fn new(x:i32, y:i32) -> Node {
        Node {
            x: x,
            y: y,
            f_cost: std::f32::MAX,
        }
    }
    fn to_point(&self, path_finder: &PathFinder) -> Point {
        let lat = self.y as f32 / RADIUS + path_finder.origin.lat;
        let lon = ((self.x as f32 / RADIUS / 2.0).powi(2).sin() /
         path_finder.origin.lat.cos() / lat.cos()).sqrt().asin() * 2.0 + path_finder.origin.lon;
        Point::from_radians(lat, lon)
    }
}

impl Point {
    fn to_node(&self, path_finder: &PathFinder) -> Node {
        let x = 2.0*RADIUS*(self.lat.cos()*((self.lon-path_finder.origin.lon)/2.0).sin()).asin();
        let y = 2.0*RADIUS*((self.lat-path_finder.origin.lat)/2.0);
        Node::new((x/path_finder.grid_size).floor() as i32, (y/path_finder.grid_size).floor() as i32)
    }
}

pub struct PathFinder {
	grid_size: f32,   // In meters
	buffer: u32,   // In meters
	max_process_time: u32,   // In seconds
	origin: Point,
	plane: Plane,
	wp_list: Vec<Waypoint>,
    obstacle_list: HashSet<Node>,
    offset: [(i32,i32,f32); 8]
}

impl PathFinder{
    fn new(grid_size: f32, flyzone_points: Vec<Point>) -> PathFinder {
        if flyzone_points.len() <= 2 {
            panic!("Require at least 3 points to construct fly zone.");
        }
		let origin = PathFinder::find_origin(&flyzone_points);
        let g_cost_diag = (2.0*grid_size.powi(2)).sqrt();
		let offset = [
		    (-1,-1,g_cost_diag),(-1,0,grid_size),
            (-1,1,g_cost_diag),(1,-1,g_cost_diag),
            (1,0,grid_size),(1,1,g_cost_diag),
            (0,-1,grid_size),(0,1,grid_size)
		];
		let mut new_path_finder = PathFinder {
            grid_size: grid_size,
			buffer: 1,
			max_process_time: 10,
			origin: origin,
			plane: Plane::new(0.0,0.0,0.0),
			obstacle_list: HashSet::new(),
			wp_list: Vec::new(),
			offset: offset
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

        Point::from_degrees(min_lat, lon)
	}

	fn initialize(&mut self, flyzone_points: &Vec<Point>) {
		// Initialize boundry obstacles
        for point in flyzone_points {
            let node = point.to_node(&self);
            self.obstacle_list.insert(node);
        }
	}

	pub fn adjust_path(&mut self, plane: Plane) -> Option<Vec<Waypoint>> {
	    self.plane = plane;
        let start = plane.coords.to_node(&self);
        let end = self.wp_list[0].location.to_node(&self);
        let mut g_cost_map: HashMap<Node, f32> = HashMap::new();
        let mut parent_map: HashMap<Node, Node> = HashMap::new();
        let mut open_list: BTreeSet<Node> = BTreeSet::new();
        let mut close_list: HashSet<Node> = HashSet::new();
        let f_cost = (((end.x-start.x).pow(2)+(end.y-start.y).pow(2)) as f64).sqrt() as u32 * 10;
        open_list.insert(start);
        let mut current_node;

        while !open_list.is_empty() {
            current_node = *open_list.iter().next().unwrap();
            if current_node == end {
                println!("Path found.");
                let mut path = Vec::new();
                while current_node != start {
                    let parent = *parent_map.get(&current_node).unwrap();
                    path.push(Waypoint::new(parent.to_point(&self)));
                    current_node = parent;
                }
                return Some(path);
            }
            open_list.remove(&current_node);
            close_list.insert(current_node);
            let current_cost = *g_cost_map.get(&current_node).unwrap();
            println!("current {:?}", current_node);
            for &(x_offset, y_offset, g_cost) in self.offset.into_iter() {
                let x = current_node.x + x_offset;
                let y = current_node.y + y_offset;
                let new = Node::new(x,y);
                if self.obstacle_list.contains(&new) {
                    continue;
                }
                if close_list.contains(&new) {
                    continue;
                }
                if !open_list.contains(&new) {
                    open_list.insert(new);
                }
                let mut current = open_list.take(&new).unwrap();
                let cost = current_cost + g_cost;
                if  cost >= *g_cost_map.get(&current).unwrap() {
                    open_list.insert(current);
                    continue;
                }
                g_cost_map.insert(current, cost);
                current.f_cost = cost +
                    (((end.x-x).pow(2)+(end.y-y).pow(2)) as f32).sqrt();
                parent_map.insert(current, current_node);
                open_list.insert(current);
            }
        }
        None
 	}

    pub fn set_waypoint_list(&mut self, list: Vec<Waypoint>) {
        self.wp_list = list;
    }

    pub fn set_buffer( &mut self, new_buffer : u32 ) -> (){
        self.buffer = new_buffer;
        // TODO: regenerate node map
    }
    pub fn set_process_time( &mut self, new_time : u32 ) -> (){
        self.max_process_time = new_time;
    }
    pub fn set_plane( &mut self, new_plane : Plane ) -> (){
        self.plane = new_plane;
    }
    pub fn set_obstacle_list( &mut self, obstacle_list : Vec<Obstacle> ) -> (){
        // Process obstacles
    }
    pub fn get_buffer(self) -> u32{
        self.buffer
    }
    pub fn get_process_time(self) -> u32{
        self.max_process_time
    }
    pub fn get_plane(self) -> Plane{
        self.plane
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hav_test1() {
        println!("---------------");
        println!("test1");
        let path_finder1 = PathFinder::new(1.0, vec!(
            Point::from_degrees(50.06638888888889,-5.714722222222222),
            Point::from_degrees(58.64388888888889,-5.714722222222222),
            Point::from_degrees(50.06638888888889,-3.0700000000000003)
        ));
        for node in path_finder1.obstacle_list {
            println!("x: {} y: {}", node.x, node.y);
        }
    }
}
