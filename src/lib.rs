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
const DIR:[(i32, i32); 8] = [
    (-1,-1),(-1,0),(-1,1),(1,-1),
    (1,0),(1,1),(0,-1),(0,1)
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

    // Find best path using the a* algorithm
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
        }
        None
 	}

    // Find best path using the jump point search algorithm in combination with a*
    pub fn adjust_path_jump_point(&mut self, plane: Plane) -> Option<Vec<Waypoint>> {
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

            // Jump point search node discovery
            self.jump(current_node);
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
        let first_node : Node = flyzone_points[0].to_node(&self);
	let mut pre_node : Node = flyzone_points[flyzone_points.len() - 1].to_node(&self);
        let mut index = 0;

        for end_point in flyzone_points
        {
            let mut end_node : Node = end_point.to_node(&self);

            index += 1;

            let mut cur_x = pre_node.x;
            let mut cur_y = pre_node.y;

            println!("FIRST: {:?} {:?}", pre_node.x, pre_node.y);
            println!("SECOND: {:?} {:?}", end_node.x, end_node.y);

            if pre_node.x == end_node.x
            {
                while cur_y != end_node.y
                {
                    //AddPoint cur_x, cur_x
                    let to_add : Node = Node::new(cur_x, cur_y);
                    self.obstacle_list.insert(to_add);

                    if pre_node.y > end_node.y
                    {
                        cur_y -= 1;
                    }
                    else
		    {
                        cur_y += 1;
                    }
                }
            }
            else
	    {
                let top : f32 = (end_node.y - pre_node.y) as f32;
                let bot : f32 = (end_node.x - pre_node.x) as f32;
                let slope : f32 = top / bot;
                let mut cur_x_f32 : f32 = cur_x as f32;
                let mut cur_y_f32 : f32 = cur_y as f32;

                while ((cur_x >= end_node.x && pre_node.x > end_node.x) || (cur_x <= end_node.x && pre_node.x < end_node.x)) && !(cur_x == end_node.x && cur_y == end_node.y)
                {
                    //println!("{:?} {:?}", cur_x, cur_y);
                    if pre_node.x > end_node.x
                    {
                        cur_x_f32 = cur_x_f32 + (-1f32 * 0.1);
                        cur_y_f32 = cur_y_f32 + (-1f32 * 0.1 * slope);
                    }
                    else
		    {
                        cur_x_f32 = cur_x_f32 + 0.1;
                        cur_y_f32 = cur_y_f32 + (0.1f32 * slope);
                    }

                    if cur_x != cur_x_f32.floor() as i32 || cur_y != cur_y_f32.floor() as i32
                    {
                        cur_x = cur_x_f32.floor() as i32;
                        cur_y = cur_y_f32.floor() as i32;

                        let to_add : Node = Node::new(cur_x, cur_y);
                        self.obstacle_list.insert(to_add);

                        if pre_node.x > end_node.x
                        {
                            let add_buffer : Node = Node::new(cur_x + 1, cur_y);
                            self.obstacle_list.insert(add_buffer);
                        }
                        else
			{
                            let add_buffer : Node = Node::new(cur_x - 1, cur_y);
                            self.obstacle_list.insert(add_buffer);
                        }
                    }
                }
            }
	    pre_node = end_node;
        }
    }

    fn reset(&mut self) {
        self.parent_map = HashMap::new();
        self.open_list = BTreeSet::new();
        self.close_list = HashSet::new();
    }

    fn generate_path(&mut self, mut current_node: Node) -> Vec<Waypoint>{
        let mut path = Vec::new();
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
            if self.obstacle_list.contains(&new_node) || self.close_list.contains(&new_node) {
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

    fn jump(&mut self, current_node: Node) {
        if self.parent_map.contains_key(&current_node) {
            let parent_node = *self.parent_map.get(&current_node).unwrap();
            let mut x_dir = 1;
            let mut y_dir = 1;
            if current_node.x < parent_node.x {
                x_dir = 0;
            } else if current_node.x < parent_node.x {
                x_dir = -1;
            }
            if current_node.y < parent_node.y {
                y_dir = 0;
            } else if current_node.y < parent_node.y {
                y_dir = -1;
            }

            // Diagonal case
            if x_dir != 0 && y_dir != 0 {
                if self.obstacle_list.contains(&Node::new(current_node.x-x_dir, current_node.y)) {
                    self.jump_forward(current_node, -x_dir, y_dir);
                }
                if self.obstacle_list.contains(&Node::new(current_node.x, current_node.y-y_dir)) {
                    self.jump_forward(current_node, x_dir, -y_dir);
                }
            } else {
                self.jump_forward(current_node, x_dir, y_dir);
            }
        } else {
            // Case for root node, possibly overwrite with yaw in the future
            for &(x_dir, y_dir) in DIR.into_iter() {
                self.jump_forward(current_node, x_dir, y_dir);
            }
        }
    }

    // Discover nodes by jumping until forced neighbors are encountered
    fn jump_forward(&mut self, current_node: Node, x_dir: i32, y_dir: i32) {
        let mut new_node = current_node;
        let mut valid_candidate = false;
        new_node.advance(x_dir, y_dir);
        while new_node != self.end_node {
            // Diagonal exploration
            if x_dir != 0 && y_dir != 0 {
                if self.find_successor(new_node, x_dir, 0) ||
                  self.find_successor(new_node, 0, y_dir) {
                    valid_candidate = true;
                    break;
                }
                new_node.advance(x_dir, y_dir);
                if self.obstacle_list.contains(&new_node) || self.close_list.contains(&new_node) {
                    break;
                }
                // Forced neightbors handler
                if self.obstacle_list.contains(&Node::new(current_node.x-x_dir, current_node.y))
                  || self.obstacle_list.contains(&Node::new(current_node.x, current_node.y-y_dir)) {
                    valid_candidate = true;
                    break;
                }
            } else {
                if self.find_successor(new_node, x_dir, y_dir) {
                    valid_candidate = true;
                    break;
                }
            }
        }

        if valid_candidate {
            new_node.g_cost = (((new_node.x-current_node.x).pow(2) +
                (new_node.y-current_node.y).pow(2)) as f32).sqrt();
            new_node.f_cost = new_node.g_cost + (((self.end_node.x-new_node.x).pow(2) +
                (self.end_node.y-new_node.y).pow(2)) as f32).sqrt();
            self.open_list.insert(new_node);
            self.parent_map.insert(new_node, current_node);
        }
    }

    // WIP
    // Vertical and horizontal exploration
    // Return true if successor found, false otherwise
    fn find_successor(&mut self, mut current_node: Node, x_dir: i32, y_dir: i32) -> bool{
        while current_node != self.end_node {
            current_node.advance(x_dir, y_dir);
            if self.obstacle_list.contains(&current_node) {
                return false;
            }
            // Forced neighbors handler
            if self.obstacle_list.contains(&Node::new(current_node.x + y_dir, current_node.y + x_dir)) ||
            self.obstacle_list.contains(&Node::new(current_node.x - y_dir, current_node.y - x_dir)) {
              return true;
            }
        }

        true
    }
	
    /*
    Takes in the paramters of the max X and max Y you want to see, and displays obstacles
    and non-obstacle nodes. Obstacle nodes are labeled "X" and non-obstacle nodes are
    labeled as ".".
    */
    pub fn draw(&self, x_min : i32, x_max : i32, y_min : i32, y_max : i32)
    {
        for y in y_min..y_max
        {
            for x in x_min..x_max
            {
                let current : Node = Node::new(x, y_max - y);
                //print!("XY {:?} {:?}", x, y);
                //println!("", );
                if self.obstacle_list.contains(&current)
                {
                    print!("X");
                }
                else {
                    print!(".");
                }
            }
            println!("",);
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
	
    #[test]
    fn hav_test_draw() {
        println!("test_draw");
        let flight_zone = vec!(
         Point::from_degrees(30.32247, -97.6009),
         Point::from_degrees(30.32307, -97.6005),
         Point::from_degrees(30.32373, -97.6012),
         Point::from_degrees(30.32366, -97.6019),
         Point::from_degrees(30.32321, -97.6025),
     );
        let path_finder1 = PathFinder::new(1.0, flight_zone);
        path_finder1.draw(0, 200, 0, 200);
    }
}
