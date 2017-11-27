#![allow(dead_code)]
#![allow(unused_variables)]

use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::BTreeSet;
use std::error::Error;
use std::io::prelude::*;
use std::fs::File;
use std::path::Path;
use std::collections::BinaryHeap;
use std::rc::Rc;

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

const EQUATORIAL_RADIUS:f64 = 63781370.0;
const POLAR_RADIUS:f64 = 6356752.0;
const RADIUS:f64 = 6371000.0;

pub struct PathFinder {
	grid_size: f32,   // In meters
	buffer: f32,   // In meters
	max_process_time: u32,   // In seconds
	origin: Point,
	plane: Plane,
	wp_list: Vec<Waypoint>,
    obstacle_list: HashSet<Node>,
    end_node: Node,
    open_heap: BinaryHeap<Rc<Node>>,
    open_set: HashSet<Rc<Node>>,
    close_list: HashSet<Rc<Node>>
}

impl PathFinder {
    pub fn new(grid_size: f32, flyzone_points: Vec<Point>) -> PathFinder {
        if flyzone_points.len() <= 2 {
            panic!("Require at least 3 points to construct fly zone.");
        }
        let mut new_path_finder = PathFinder {
            grid_size: grid_size,
            buffer: 1.0,
            max_process_time: 10,
            origin: PathFinder::find_origin(&flyzone_points),
            plane: Plane::new(0.0,0.0,0.0),
            wp_list: Vec::new(),
            obstacle_list: HashSet::new(),
            end_node: Node::new(0,0),
            open_heap: BinaryHeap::new(),
            open_set: HashSet::new(),
            close_list: HashSet::new()
        };
        new_path_finder.initialize(&flyzone_points);
        new_path_finder
    }

    // Initilization
    fn find_origin(flyzone_points: &Vec<Point>) -> Point {
        let mut min_lat = flyzone_points[0].lat();
        let mut min_lon = flyzone_points[0].lon();
        let mut max_lon = flyzone_points[0].lon();
        let flyzone_points = &flyzone_points[1..];
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
        let mut lon = min_lon;
        if max_lon - min_lon > 2f64*std::f64::consts::PI {
            lon = max_lon;
        }

        Point::from_radians(min_lat, lon)
    }

    fn initialize(&mut self, flyzone_points: &Vec<Point>) {
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

    // Find best path using the a* algorithm
	pub fn adjust_path(&mut self, plane: Plane) -> Option<Vec<Waypoint>> {
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
            self.discover_node(Rc::clone(&current_node));
        }
        None
 	}

    fn discover_node(&mut self, current_node: Rc<Node>) {
        // println!("{} {}", current_node.x, current_node.y);
        for &(x_offset, y_offset, g_cost) in OFFSET.into_iter() {
            let mut new_node;
            new_node = Node::new(current_node.x + x_offset, current_node.y + y_offset);
            // println!("new {} {}", new_node.x, new_node.y);

            if self.obstacle_list.contains(&new_node) || self.close_list.contains(&new_node) {
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
            // println!("f_cost: {}", current_node.f_cost);
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
                if self.obstacle_list.contains(&Node::new(current_node.x-x_dir, current_node.y))
                  || self.obstacle_list.contains(&Node::new(current_node.x, current_node.y-y_dir)) {
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
            // println!("f_cost: {}", current_node.f_cost);
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

    fn generate_path(&self, mut current_node: Rc<Node>) -> Vec<Waypoint>{
        let mut path = Vec::new();
        let mut node;
        let mut x_dir = 0;
        let mut y_dir = 0;
        let mut new_x_dir;
        let mut new_y_dir;

        #[allow(while_true)]
        while true {
            match current_node.parent {
                Some(ref parent) => node = Rc::clone(&parent),
                None => break,
            }
            new_x_dir = node.x - current_node.x;
            new_y_dir = node.y - current_node.y;
            if x_dir != new_x_dir || y_dir != new_y_dir {
                x_dir = new_x_dir;
                y_dir = new_y_dir;
                // println!("{} {}", node.x, node.y);
                path.push(Waypoint::new(node.to_point(&self)));
            }
            current_node = node;
        }
        path
    }


    fn export_obstacle_list_to_file(&mut self)
    {
        let path = Path::new("obstacle_list_export.txt");
        let display = path.display();

        let mut file = match File::create(&path) {
            Err(why) => panic!("couldn't create {}: {}",
                               display,
                               why.description()),
            Ok(file) => file,
        };

        for Node in &self.obstacle_list {
            let comma = ",";
            let dash = "-";
            let node_rep = format!("{}{}{}{}", Node.x.to_string(), comma, Node.y.to_string(), dash);

            match file.write_all(node_rep.as_bytes()) {
                Err(why) => {
                    panic!("couldn't write to {}: {}", display,
                                                       why.description())
                },
                Ok(_) => println!("successfully wrote to {}", display),
            }
        }
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

    pub fn set_buffer(&mut self, new_buffer: f32) {
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
        for obst in obstacle_list {
      			let radius = (((obst.radius + self.buffer)/(self.grid_size)) as i32) + 1;
                //println!("radius: {}",radius);
      			let n = obst.coords.to_node(&self);
                // println!("center node: {:?}",n);
      			let top_left = Node::new(n.x - radius, n.y + radius);
      	// 		println!("top left node: {:?}",top_left);
      			let bottom_right = Node::new(n.x + radius, n.y - radius);
      	// 		println!("bottom right node: {:?}",bottom_right);
      			let rsquared = (radius)*(radius);
      	// 		println!("radius = {}", radius);
                // println!("{:?}",self.obstacle_list);
      			for x in top_left.x .. bottom_right.x + 1 {
                    let newx = x - n.x;
                    // println!("x: {}",x);
                    // println!("x^2: {}",x*x);
                    // println!("rsquared: {}",rsquared);
      				let temp = rsquared as f32 - (newx as f32)*(newx as f32);
                    // println!("rsquared - x^2: {}",temp);
                    let y  = temp.abs().sqrt() as i32;
                    let negy = -y;
                    if x == top_left.x || x == bottom_right.x {
                        for  difference in -radius/3 .. radius/3 + 1 {
                            self.obstacle_list.insert(Node::new(newx,y+difference));
                        }
                    }
                    else{
                        self.obstacle_list.insert(Node::new(newx,y));
                        self.obstacle_list.insert(Node::new(newx,y-1));
                        self.obstacle_list.insert(Node::new(newx,y+1));
                        self.obstacle_list.insert(Node::new(newx,negy));
                        self.obstacle_list.insert(Node::new(newx,negy-1));
                        self.obstacle_list.insert(Node::new(newx,negy+1));
                    }
                    //println!("y: {}",y);
      			}
                // println!("{:?}",self.obstacle_list);

                let size : i32 = radius*2+1;
                // Base 1d array
                let mut grid_raw = vec!['.'; (size*size) as usize];

                // Vector of 'width' elements slices
                let mut grid_base: Vec<_> = grid_raw.as_mut_slice().chunks_mut((size) as usize).collect();

                // Final 2d array
                let arr: &mut [&mut [_]] = grid_base.as_mut_slice();
                //println!("{:?}",arr);
                //let mut arr : [[char; 20];20] = [['.';20];20];
                // println!("n.y: {}",n.y);
                for node in &self.obstacle_list {
                //    println!("x: {}",(node.x - n.x) + size/2);
                //    println!("y: {}",(node.y - n.y) + size/2);
                   if (node.x) + size/2 < size && (node.x) + size/2 >= 0 && (node.y) + size/2  < size && (node.y) + size/2 >= 0 {
                        arr[(node.y + size/2) as usize][(node.x + size/2) as usize] = 'X';
                   }
                }
                arr[(size/2) as usize][(size/2) as usize] = 'O';
                for row in arr {
                    // println!("{:?}",row);
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

    pub fn get_plane(&self) -> Plane {
        self.plane
    }
}

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
}
