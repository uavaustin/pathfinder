#![allow(dead_code)]
#![allow(unused_variables)]

use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::BTreeSet;

mod obj;
mod node;
use node::*;
pub use obj::*;

const OFFSET:[(i32,i32,f32); 8] = [
    (-1,-1,1.4),(-1,0,1.0),
    (-1,1,1.4),(1,-1,1.4),
    (1,0,1.0),(1,1,1.4),
    (0,-1,1.0),(0,1,1.0)
];

const EQUATORIAL_RADIUS:f32 = 63781370.0;
const POLAR_RADIUS:f32 = 6356752.0;
const RADIUS:f32 = 6371000.0;

pub struct PathFinder {
	grid_size: f32,   // In meters
	buffer: u32,   // In meters
	max_process_time: u32,   // In seconds
	origin: Point,
	plane: Plane,
	wp_list: Vec<Waypoint>,
    obstacle_list: HashSet<Node>,
    start_node: Node,
    end_node: Node,
    parent_map: HashMap<Node, Node>,
    open_list: BTreeSet<Node>,
    close_list: HashSet<Node>
}

impl PathFinder {
    pub fn new(grid_size: f32, flyzone_points: Vec<Point>) -> PathFinder {
        if flyzone_points.len() <= 2 {
            panic!("Require at least 3 points to construct fly zone.");
        }
        let mut new_path_finder = PathFinder {
            grid_size: grid_size,
            buffer: 1,
            max_process_time: 10,
            origin: PathFinder::find_origin(&flyzone_points),
            plane: Plane::new(0.0,0.0,0.0),
            wp_list: Vec::new(),
            obstacle_list: HashSet::new(),
            start_node: Node::new(0,0),
            end_node: Node::new(0,0),
            parent_map: HashMap::new(),
            open_list: BTreeSet::new(),
            close_list: HashSet::new()
        };
        new_path_finder.initialize(&flyzone_points);
        new_path_finder
    }

	pub fn adjust_path(&mut self, plane: Plane) -> Option<Vec<Waypoint>> {
	    self.plane = plane;
        self.start_node = plane.coords.to_node(&self);
        self.end_node = self.wp_list[0].location.to_node(&self);
        self.reset();
        self.open_list.insert(self.start_node);

        let mut current_node;
        while !self.open_list.is_empty() {
            current_node = *self.open_list.iter().next().unwrap();
            if current_node == self.end_node {
                return Some(self.generate_path(current_node));
            }
            self.open_list.remove(&current_node);
            self.close_list.insert(current_node);

            // Regular a* node discovery
            self.discover_node(current_node);
            // Jump point search node discovery
            //self.find_successor(current_node, x_dir, y_dir);
        }
        None
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
        if max_lon - min_lon > 2.0*std::f32::consts::PI {
            lon = max_lon;
        }

        Point::from_radians(min_lat, lon)
    }

    fn initialize(&mut self, flyzone_points: &Vec<Point>) {
        // Initialize boundry obstacles
        for point in flyzone_points {
            let node = point.to_node(&self);
            self.obstacle_list.insert(node);
        }
    }

    fn reset(&mut self) {
        self.parent_map = HashMap::new();
        self.open_list = BTreeSet::new();
        self.close_list = HashSet::new();
    }

    fn generate_path(&mut self, current_node: Node) -> Vec<Waypoint>{
        let mut path = Vec::new();
        let mut current_node = current_node;
        while current_node != self.start_node {
            let parent = *self.parent_map.get(&current_node).unwrap();
            path.push(Waypoint::new(parent.to_point(&self)));
            current_node = parent;
        }
        return path;
    }

    fn discover_node(&mut self, current_node: Node) {
        for &(x_offset, y_offset, g_cost) in OFFSET.into_iter() {
            let mut new_node = Node::new(current_node.x + x_offset, current_node.y + y_offset);
            if self.obstacle_list.contains(&new_node) || self.close_list.contains(&new_node){
                continue;
            }
            new_node.g_cost = current_node.g_cost + g_cost;
            if let Some(node) = self.open_list.take(&new_node) {
                if new_node.g_cost >= node.g_cost {
                    self.open_list.insert(node);
                    continue;
                }
            }
            new_node.f_cost = new_node.g_cost + (((self.end_node.x-new_node.x).pow(2) +
                (self.end_node.y-new_node.y).pow(2)) as f32).sqrt();
            self.parent_map.insert(new_node, current_node);
            self.open_list.insert(new_node);
        }
    }

    pub fn set_waypoint_list(&mut self, list: Vec<Waypoint>) {
        self.wp_list = list;
    }

    pub fn set_buffer(&mut self, new_buffer: u32) {
        self.buffer = new_buffer;
        // TODO: regenerate node map
    }

    pub fn set_process_time(&mut self, new_time: u32) {
        self.max_process_time = new_time;
    }

    pub fn set_plane(&mut self, new_plane: Plane) {
        self.plane = new_plane;
    }

    pub fn set_obstacle_list(&mut self, obstacle_list: Vec<Obstacle>) {
        // Process obstacles
    }

    pub fn get_grid_size(self) -> f32 {
        self.grid_size
    }

    pub fn get_buffer(self) -> u32 {
        self.buffer
    }

	pub fn get_origin(self) -> Point {
		self.origin
	}

    pub fn get_process_time(self) -> u32 {
        self.max_process_time
    }

    pub fn get_plane(self) -> Plane {
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
        println!("Origin: {}, {}", path_finder1.origin.lat, path_finder1.origin.lon);
        for node in path_finder1.obstacle_list {
            println!("x: {} y: {}", node.x, node.y);
        }
    }
}
