#![allow(dead_code)]
#![allow(unused_variables)]

use std::collections::HashSet;
use std::collections::BinaryHeap;
use std::collections::LinkedList;
use std::rc::Rc;

mod obj;
mod node;
mod debug;
use node::*;
pub use obj::*;
pub use debug::*;

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

const EQUATORIAL_RADIUS:f64 = 63781370.0;
const POLAR_RADIUS:f64 = 6356752.0;
const RADIUS:f64 = 6371000.0;

pub struct PathFinder {
    initialized: bool,
	grid_size: f32,   // In meters
	buffer: f32,   // In meters
	max_process_time: u32,   // In seconds
	origin: Point,
    current_wp: Waypoint,
	wp_list: LinkedList<Waypoint>,
    obstacle_list: HashSet<Node>,
    end_node: Node,
    open_heap: BinaryHeap<Rc<Node>>,
    open_set: HashSet<Rc<Node>>,
    close_list: HashSet<Rc<Node>>
}

impl PathFinder {
    pub fn new() -> PathFinder {
        PathFinder {
            initialized: false,
            grid_size: 1f32,
            buffer: 1f32,
            max_process_time: 10,
            origin: Point::from_degrees(0f64, 0f64),
            current_wp: Waypoint::new(0, Point::from_degrees(0f64, 0f64), 0f32, 0f32),
            wp_list: LinkedList::new(),
            obstacle_list: HashSet::new(),
            end_node: Node::new(0,0),
            open_heap: BinaryHeap::new(),
            open_set: HashSet::new(),
            close_list: HashSet::new()
        }
    }

    pub fn init(&mut self, grid_size: f32, flyzones: Vec<Vec<Point>>) {
        self.grid_size = grid_size;
        self.buffer = grid_size;
        self.origin = PathFinder::find_origin(&flyzones);
        self.generate_fly_zone(&flyzones);
        self.initialized = true;
    }

    // Initilization
    fn find_origin(flyzones: &Vec<Vec<Point>>) -> Point {
        const MAX_RADIAN: f64 = 2f64*std::f64::consts::PI;
        let mut min_lat = MAX_RADIAN;
        let mut min_lon = MAX_RADIAN;
        let mut max_lon = 0f64;
        let mut lon = min_lon;

        for i in 0..flyzones.len() {
            let flyzone_points = &flyzones[i];
            if flyzone_points.len() < 3 {
                panic!("Require at least 3 points to construct fly zone.");
            }

            for point in flyzone_points {
                if point.lat() < min_lat {
                    min_lat = point.lat();
                }
                if point.lon() < min_lon {
                    min_lon = point.lon();
                }
                if point.lon() > max_lon {
                    max_lon = point.lon();
                }
            }
            lon = min_lon;
            if max_lon - min_lon > MAX_RADIAN {
                lon = max_lon;
            }
        }

        Point::from_radians(min_lat, lon)
    }

    fn generate_fly_zone(&mut self, flyzones: &Vec<Vec<Point>>) {
        for i in 0..flyzones.len() {
            let flyzone_points = &flyzones[i];
            let first_node : Node = flyzone_points[0].to_node(&self);
            let mut pre_node : Node = flyzone_points[flyzone_points.len() - 1].to_node(&self);
            let mut end_node;
            let mut index = 0;

            for end_point in flyzone_points
            {
                // println!("{:.5}, {:.5}", end_point.lat_degree(), end_point.lon_degree());
                end_node = end_point.to_node(&self);
                let point = end_node.to_point(&self);
                // println!("{:.5}, {:.5}", point.lat_degree(), point.lon_degree());

                index += 1;

                let mut cur_x = pre_node.x;
                let mut cur_y = pre_node.y;

                // println!("FIRST: {:?} {:?}", pre_node.x, pre_node.y);
                // println!("SECOND: {:?} {:?}", end_node.x, end_node.y);

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
    }

    pub fn get_adjust_path(mut self, plane: Plane, mut wp_list: LinkedList<Waypoint>) -> LinkedList<Waypoint> {
        assert!(self.initialized);
        self.wp_list = LinkedList::new();
        let mut current_loc: Point;
        let mut next_wp: Waypoint;
        match wp_list.pop_front() {
            Some(wp) => next_wp = wp,
            None => return wp_list
        }
        self.current_wp = next_wp;   // First destination if first waypoint
        current_loc = self.current_wp.location;
        self.adjust_path(plane.location, current_loc);
        #[allow(while_true)]
        while true {
            match wp_list.pop_front() {
                Some(wp) => next_wp = wp,
                None => break
            }

            current_loc = self.current_wp.location;
            self.adjust_path(current_loc, next_wp.location);
            self.current_wp = next_wp;
        }
        self.wp_list
    }

    // Find best path using the a* algorithm
    // #TODO: handle altitude change
	fn adjust_path(&mut self, start: Point, end: Point) {
        self.open_heap = BinaryHeap::new();
        self.open_set = HashSet::new();
        self.close_list = HashSet::new();
        self.end_node = start.to_node(&self);
        let start_node = end.to_node(&self);    // Reverse because backtracking at the end
        let start_node = Rc::new(Node {
            x: start_node.x,
            y: start_node.y,
            g_cost: 0f32,
            f_cost: (((self.end_node.x-start_node.x).pow(2) +
                (self.end_node.y-start_node.y).pow(2)) as f32).sqrt(),
            parent: None
        });
        self.open_set.insert(Rc::clone(&start_node));
        self.open_heap.push(Rc::clone(&start_node));

        let mut current_node:Rc<Node>;
        #[allow(while_true)]
        while true {
            if let Some(node) = self.open_heap.pop() {
                current_node = node;
            } else {
                break;
            }
            // println!("f_cost: {}", current_node.f_cost);
            if *current_node == self.end_node {
                self.generate_path(Rc::clone(&current_node));
                return;
            }
            self.open_set.take(&current_node);
            self.close_list.insert(Rc::clone(&current_node));

            // Regular a* node discovery
            self.discover_node(Rc::clone(&current_node));
        }
        eprintln!("No path found!");
 	}

    fn discover_node(&mut self, current_node: Rc<Node>) {
        // println!("{} {}", current_node.x, current_node.y);
        for &(x_offset, y_offset, g_cost) in OFFSET.into_iter() {
            let mut new_node;
            new_node = Node::new(current_node.x + x_offset, current_node.y + y_offset);
            // println!("new {} {}", new_node.x, new_node.y);

            if self.obstacle_list.contains(&new_node) || self.close_list.contains(&new_node) {
                // println!("x: {}, y: {}", current_node.x, current_node.y);
                continue;
            }
            new_node.g_cost = current_node.g_cost + g_cost;

            if let Some(node) = self.open_set.take(&new_node) {
                // println!("exists {}", node.f_cost);
                if new_node.g_cost >= node.g_cost {
                    self.open_set.insert(node);
                    continue;
                }
            }
            // println!("len {} new_x {} new_y {} diff_x {} diff_ y {}", self.open_list.len(),
            //     new_node.x, new_node.y, self.end_node.x-new_node.x, self.end_node.y-new_node.y);
            new_node.f_cost = new_node.g_cost + (((self.end_node.x-new_node.x).pow(2) +
                (self.end_node.y-new_node.y).pow(2)) as f32).sqrt();
            new_node.parent = Some(Rc::clone(&current_node));
            let new_node = Rc::new(new_node);
            self.open_set.insert(Rc::clone(&new_node));
            self.open_heap.push(Rc::clone(&new_node));
        }
    }

// Jump Point Search
/*
    // Find best path using the jump point search algorithm in combination with a*
    pub fn adjust_path_jump_point(&mut self, plane: Plane) -> Option<Vec<Waypoint>> {
        self.plane = plane;
        self.open_heap = BinaryHeap::new();
        self.open_set = HashSet::new();
        self.close_list = HashSet::new();
        self.end_node = self.wp_list[0].location.to_node(&self);
        let start_node = plane.coords.to_node(&self);
        let start_node = Rc::new(Node {
            x: start_node.x,
            y: start_node.y,
            g_cost: 0f32,
            f_cost: (((self.end_node.x-start_node.x).pow(2) +
                (self.end_node.y-start_node.y).pow(2)) as f32).sqrt(),
            parent: None
        });
        self.open_set.insert(Rc::clone(&start_node));
        self.open_heap.push(Rc::clone(&start_node));

        let mut current_node:Rc<Node>;
        #[allow(while_true)]
        while true {
            if let Some(node) = self.open_heap.pop() {
                current_node = node;
            } else {
                break;
            }
            // println!("f_cost: {}", current_node.f_cost);
            if *current_node == self.end_node {
                return Some(self.generate_path(Rc::clone(&current_node)));
            }
            self.open_set.take(&current_node);
            self.close_list.insert(Rc::clone(&current_node));

            // Regular a* node discovery
            self.jump(Rc::clone(&current_node));
        }
        None
    }

    fn jump(&mut self, current_node: Rc<Node>) {
        if let Some(ref parent_node) = current_node.parent {
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
                    self.jump_forward(Rc::clone(&current_node), -x_dir, y_dir);
                }
                if self.obstacle_list.contains(&Node::new(current_node.x, current_node.y-y_dir)) {
                    self.jump_forward(Rc::clone(&current_node), x_dir, -y_dir);
                }
            } else {
                self.jump_forward(Rc::clone(&current_node), x_dir, y_dir);
            }
        } else {
            // Case for root node, possibly overwrite with yaw in the future
            for &(x_dir, y_dir) in DIR.into_iter() {
                self.jump_forward(Rc::clone(&current_node), x_dir, y_dir);
            }
        }
    }

    // Discover nodes by jumping until forced neighbors are encountered
    fn jump_forward(&mut self, current_node: Rc<Node>, x_dir: i32, y_dir: i32) {
        let mut new_node = Node::clone(&current_node);
        let mut valid_candidate = false;
        new_node.advance(x_dir, y_dir);
        while new_node != self.end_node {
            println!("x: {}, y: {}, x_dir: {}, y_dir: {}", new_node.x, new_node.y, x_dir, y_dir);
            // Diagonal exploration
            if x_dir != 0 && y_dir != 0 {
                if self.find_successor(&mut new_node, x_dir, 0) ||
                  self.find_successor(&mut new_node, 0, y_dir) {
                    valid_candidate = true;
                    break;
                }
                new_node.advance(x_dir, y_dir);
                if self.obstacle_list.contains(&new_node) || self.close_list.contains(&new_node) {
                    break;
                }
                // Forced neightbors handler
                if self.obstacle_list.contains(&Node::new(new_node.x-x_dir, new_node.y)) ||
                 self.obstacle_list.contains(&Node::new(new_node.x, new_node.y-y_dir)) {
                    valid_candidate = true;
                    break;
                }
            } else {
                if self.find_successor(&mut new_node, x_dir, y_dir) {
                    valid_candidate = true;
                    break;
                }
            }
        }

        if valid_candidate {
            println!("open_set");
            new_node.g_cost = (((new_node.x-current_node.x).pow(2) +
                (new_node.y-current_node.y).pow(2)) as f32).sqrt();
            new_node.f_cost = new_node.g_cost + (((self.end_node.x-new_node.x).pow(2) +
                (self.end_node.y-new_node.y).pow(2)) as f32).sqrt();

            new_node.parent = Some(Rc::clone(&current_node));
            let new_node = Rc::new(new_node);
            self.open_set.insert(Rc::clone(&new_node));
            self.open_heap.push(Rc::clone(&new_node));
        }
    }

    // WIP
    // Vertical and horizontal exploration
    // Return true if successor found, false otherwise
    fn find_successor(&mut self, current_node: &mut Node, x_dir: i32, y_dir: i32) -> bool{
        while *current_node != self.end_node {
            current_node.advance(x_dir, y_dir);
            // println!("x: {}, y: {}", current_node.x, current_node.y);
            if self.obstacle_list.contains(current_node) {
                // println!("x: {}, y: {}", current_node.x, current_node.y);
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
*/

    fn generate_path(&mut self, mut current_node: Rc<Node>) {
        let mut previous_node;
        let mut last_wp = current_node.clone();
        let mut x_dir = 0;
        let mut y_dir = 0;
        let mut new_x_dir;
        let mut new_y_dir;

        // Temp variable for debugging
        let mut waypoints = HashSet::new();
        let mut line = HashSet::new();

        let location = current_node.to_point(&self);
        current_node = match current_node.parent {
            Some(ref parent) => Rc::clone(&parent),
            None => Rc::clone(&current_node)
        };
        let location = current_node.to_point(&self);

        #[allow(while_true)]
        while true {
            previous_node = current_node;
            current_node = match previous_node.parent {
                Some(ref parent) => Rc::clone(&parent),
                None => break,
            };
            new_x_dir = current_node.x - previous_node.x;
            new_y_dir = current_node.y - previous_node.y;
            if x_dir != new_x_dir || y_dir != new_y_dir {
                x_dir = new_x_dir;
                y_dir = new_y_dir;

                if self.distance_between_nodes(&current_node, &last_wp)
                    < (self.current_wp.radius * 2f32) as i64 {
                    self.wp_list.pop_back();
                    let midpoint = Node::new(
                        (current_node.x + last_wp.x) / 2,
                        (current_node.y + last_wp.y) / 2
                    );
                    waypoints.remove(&last_wp);
                    let waypoint = self.current_wp.extend(midpoint.to_point(&self));
                    self.wp_list.push_back(waypoint);
                    last_wp = Rc::new(midpoint);
                    waypoints.insert(Rc::clone(&last_wp));
                } else {
                    last_wp = Rc::clone(&current_node);
                    waypoints.insert(Rc::clone(&current_node));
                    let waypoint = self.current_wp.extend(current_node.to_point(&self));
                    self.wp_list.push_back(waypoint);
                }
            } else {
                line.insert(Rc::clone(&current_node));
            }
        }
        // self.wp_list.pop_back();

        // Graphical display for debugging
        /*
        for y in 0 .. 50 {
            print!("|");
            for x in 0 .. 50 {
                if waypoints.contains(&Node::new(x, y)) {
                    print!("+");
                    continue;
                }
                if line.contains(&Node::new(x, y)) {
                    print!("*");
                    continue;
                }
                if Node::new(x, y) == self.end_node {
                    print!("E");
                    continue;
                }
                if self.obstacle_list.contains(&Node::new(x, y)) {
                    print!("X");
                } else {
                    print!(" ");
                }
            }
            println!("|");
        }
        println!();
        */
    }

    pub fn set_buffer(&mut self, new_buffer: f32) {
        self.buffer = new_buffer;
        // TODO: regenerate node map
    }

    pub fn set_process_time(&mut self, new_time: u32) {
        self.max_process_time = new_time;
    }

    fn distance_between_nodes(&self, first_node: &Node, second_node: &Node) -> i64 {
        let x_diff: f64 = (first_node.x - second_node.x).pow(2) as f64;
        let y_diff: f64 = (first_node.y - second_node.y).pow(2) as f64;
        ((x_diff + y_diff).sqrt() * self.grid_size as f64).ceil() as i64
    }

    // Likely useless
    // fn distance_between_points(&mut self, first_point: Point, second_point: Point) -> i64
    // {
    //     let first_node : Node = first_point.to_node(&self);
    //     let second_node : Node = second_point.to_node(&self);
    //     return distance_between_nodes(first_node, second_node)
    // }
    //
    // fn is_waypoint_inside_obstacle(&mut self, waypoint: Waypoint, obstacle_list: Vec<Obstacle>) -> bool
    // {
    //     for obstacle in &obstacle_list
    //     {
    //         if self.distance_between_points(waypoint.location, obstacle.coords) <= (waypoint.radius as i64)
    //         {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    pub fn set_obstacle_list(&mut self, obstacle_list: Vec<Obstacle>) {
        for obst in obstacle_list {
  			let radius = ((obst.radius + self.buffer)/(self.grid_size)) as i32;
            // println!("radius: {}",radius);
  			let n = obst.coords.to_node(&self);

  			for x in n.x - radius .. n.x + radius {
  				let dy = ((radius.pow(2) - (x - n.x).pow(2)) as f32).sqrt() as i32;
                self.obstacle_list.insert(Node::new(x, n.y + dy));
                self.obstacle_list.insert(Node::new(x, n.y + dy - 1));
                // self.obstacle_list.insert(Node::new(x, n.y + dy - 2));
                self.obstacle_list.insert(Node::new(x, n.y - dy));
                self.obstacle_list.insert(Node::new(x, n.y - dy - 1));
                // self.obstacle_list.insert(Node::new(x, n.y - dy - 2));
  			}
            for y in n.y - radius .. n.y + radius {
  				let dx = ((radius.pow(2) - (y - n.y).pow(2)) as f32).sqrt() as i32;
                self.obstacle_list.insert(Node::new(n.x + dx, y));
                self.obstacle_list.insert(Node::new(n.x + dx - 1, y));
                // self.obstacle_list.insert(Node::new(n.x + dx - 2, y));
                self.obstacle_list.insert(Node::new(n.x - dx, y));
                self.obstacle_list.insert(Node::new(n.x - dx - 1, y));
                // self.obstacle_list.insert(Node::new(n.x - dx - 2, y));
  			}
        }
    }

    pub fn get_grid_size(&self) -> f32 {
        self.grid_size
    }

    pub fn get_buffer(&self) -> f32 {
        self.buffer
    }

	pub fn get_origin(&self) -> Point {
		self.origin
	}

    pub fn get_process_time(&self) -> u32 {
        self.max_process_time
    }
}

/*
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn conversion_test() {
        let flight_zone = vec!(
        //  Point::from_degrees(30.32247, -97.6009),
        //  Point::from_degrees(30.32307, -97.6005),
        //  Point::from_degrees(30.32373, -97.6012),
        //  Point::from_degrees(30.32366, -97.6019),
        //  Point::from_degrees(30.32321, -97.6025),
        Point::from_degrees(30.32521, -97.60230),
        Point::from_degrees(30.32466, -97.59856),
        Point::from_degrees(30.32107, -97.60032),
        Point::from_degrees(30.32247, -97.60325),
        Point::from_degrees(30.32473, -97.60410)
        );
        let path_finder1 = PathFinder::new(1.0, flight_zone);
        let test_points = vec!(
         Point::from_degrees(30.32247, -97.6009),
         Point::from_degrees(30.32307, -97.6005),
         Point::from_degrees(30.32373, -97.6012),
         Point::from_degrees(30.32366, -97.6019),
         Point::from_degrees(30.32321, -97.6025),
         Point::from_degrees(30.32521, -97.60230),
         Point::from_degrees(30.32466, -97.59856),
         Point::from_degrees(30.32107, -97.60032),
         Point::from_degrees(30.32247, -97.60325),
         Point::from_degrees(30.32473, -97.60410)
        );
        for point in test_points {
            let node1 = point.to_node(&path_finder1);
            let point1 = node1.to_point(&path_finder1);
            // print!("{:.5}, {:.5} => ", point.lat_degree(), point.lon_degree());
            println!("{:.5}, {:.5}", point1.lat_degree(), point1.lon_degree());
            assert!(point.lat_degree()-point1.lat_degree() < 0.001);
            assert!(point.lon_degree()-point1.lon_degree() < 0.001);
        }
    }

    #[test]
    fn hav_test() {
        println!("---------------");
        println!("test1");
        let path_finder1 = PathFinder::new(1.0, vec!(
            Point::from_degrees(50.06638888888889,-5.714722222222222),
            Point::from_degrees(58.64388888888889,-5.714722222222222),
            Point::from_degrees(50.06638888888889,-3.0700000000000003)
        ));
        println!("Origin: {}, {}", path_finder1.origin.lat(), path_finder1.origin.lon());
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

    #[test]
    fn export_obstacle_list_to_file_test()
    {
        let flight_zone = vec!(
         Point::from_degrees(30.32247, -97.6009),
         Point::from_degrees(30.32307, -97.6005),
         Point::from_degrees(30.32373, -97.6012),
         Point::from_degrees(30.32366, -97.6019),
         Point::from_degrees(30.32321, -97.6025),
        );
        let mut path_finder1 = PathFinder::new(1.0, flight_zone);
        path_finder1.export_obstacle_list_to_file();
    }

    #[test]
    fn is_waypoint_inside_obstacle()
    {
        let flight_zone = vec!(
         Point::from_degrees(30.32247, -97.6009),
         Point::from_degrees(30.32307, -97.6005),
         Point::from_degrees(30.32373, -97.6012),
         Point::from_degrees(30.32366, -97.6019),
         Point::from_degrees(30.32321, -97.6025),
        );
        let obstacles = vec!(
            Obstacle{coords: Point::from_degrees(30.32374, -97.60232), radius: 120.0, height: 1.0}
        );
        let point : Point = Point::from_degrees(31.32374, -97.60232);
        let mut path_finder1 = PathFinder::new(1.0, flight_zone);
        let waypoint : Waypoint = Waypoint::new(point);
        println!("{}",path_finder1.is_waypoint_inside_obstacle(waypoint, obstacles));
    }
}
*/
