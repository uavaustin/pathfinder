#![allow(dead_code)]

#[cfg(feature = "debug")]
#[macro_use]
extern crate conrod;
extern crate toml;

use std::collections::{BinaryHeap, HashSet, LinkedList};
use std::env;
use std::f64::consts::SQRT_2;
use std::fs::File;
use std::io::Read;
use std::rc::Rc;
use std::time::{Duration, SystemTime};
use toml::Value;

mod node;
mod obj;
mod util;
use node::*;
pub use obj::*;
use util::*;

#[cfg(feature = "debug")]
#[path = ""]
mod debug_block {
    #[cfg_attr(rustfmt, rustfmt_skip)]
    pub mod debug;
    pub use std::borrow::BorrowMut;

    use self::debug::Debugger;
    use conrod;
    use std::cell::RefCell;
    thread_local!(
        pub static DEBUGGER: RefCell<Debugger> = RefCell::new(Debugger::new())
    );
    pub const THRESHOLD: u32 = 750;
}

#[cfg(feature = "debug")]
pub use debug_block::*;

const UNIT_COST: f64 = 1f64; //TODO: dynamically balance G and H cost base on weights
const OFFSET: [(i32, i32, f64); 8] = [
    (-1, -1, UNIT_COST * SQRT_2),
    (1, 1, UNIT_COST * SQRT_2),
    (-1, 1, UNIT_COST * SQRT_2),
    (1, -1, UNIT_COST * SQRT_2),
    (1, 0, UNIT_COST),
    (-1, 0, UNIT_COST),
    (0, -1, UNIT_COST),
    (0, 1, UNIT_COST),
];

const CFG_FILE_NAME: &'static str = "pathfinder.toml";
const EQUATORIAL_RADIUS: f64 = 63781370.0;
const POLAR_RADIUS: f64 = 6356752.0;
const RADIUS: f64 = 6371000.0;
const MIN_BUFFER: f32 = 5f32;
const DEFAULT_DIRECT_PATH_MODIFIER_WEIGHT: f64 = 0.001;
const DEFAULT_HEADING_MODIFIER_WEIGHT: f64 = 0.15;

#[allow(non_snake_case)]
pub struct Pathfinder {
    // exposed API
    grid_size: f32,             // In meters
    buffer: f32,                // In meters
    max_process_time: Duration, // In seconds
    flyzones: Vec<Vec<Point>>,
    obstacles: Vec<Obstacle>,
    // private
    initialized: bool,
    start_time: SystemTime,
    origin: Point,
    heading: (f32, f32),
    current_wp: Waypoint,
    wp_list: LinkedList<Waypoint>,
    obstacle_list: HashSet<Node>,
    start_node: Node,
    end_node: Node,
    open_heap: BinaryHeap<Rc<Node>>,
    open_set: HashSet<Rc<Node>>,
    close_list: HashSet<Rc<Node>>,
    obstacle_found: bool,
    critical_nodes: HashSet<Rc<Node>>,
    // #TODO: possibly use lazy static variable instead
    DIRECT_PATH_MODIFIER_WEIGHT: f64,
    HEADING_MODIFIER_WEIGHT: f64,
}

// utility functions
fn load_config() -> Result<String, Box<std::error::Error>> {
    let mut p = env::current_dir()?;
    p.push(CFG_FILE_NAME);
    let mut file = File::open(p.as_path())?;
    let mut buffer = String::new();
    file.read_to_string(&mut buffer)?;
    Ok(buffer)
}
fn get_modifiers() -> (f64, f64) {
    let (mut direct_path_modifier_weight, mut heading_modifier_weight) = (
        DEFAULT_DIRECT_PATH_MODIFIER_WEIGHT,
        DEFAULT_HEADING_MODIFIER_WEIGHT,
    );
    if let Ok(buffer) = load_config() {
        if let Ok(values) = buffer.parse::<Value>() {
            let convert = |index: &str| -> Option<f64> { Some(values.get(index)?.as_float()?) };
            if let Some(val) = convert("direct_path_modifier_weight") {
                direct_path_modifier_weight = val;
            }
            if let Some(val) = convert("heading_modifier_weight") {
                heading_modifier_weight = val;
            }
        }
    };
    (direct_path_modifier_weight, heading_modifier_weight)
}

impl Pathfinder {
    pub fn new() -> Pathfinder {
        let (mut direct_path_modifier_weight, mut heading_modifier_weight) = get_modifiers();
        if let Ok(env) = env::var("DIRECT_PATH_MODIFIER_WEIGHT") {
            if let Ok(val) = str::parse::<f64>(&env) {
                direct_path_modifier_weight = val;
            }
        }
        if let Ok(env) = env::var("HEADING_MODIFIER_WEIGHT") {
            if let Ok(val) = str::parse::<f64>(&env) {
                heading_modifier_weight = val;
            }
        }
        eprintln!(
            "Direct path modifier weight: {}",
            direct_path_modifier_weight
        );
        eprintln!("Heading modifier weight: {}", heading_modifier_weight);
        Pathfinder {
            // exposed API
            grid_size: 1f32,
            buffer: 1f32,
            start_time: SystemTime::now(),
            max_process_time: Duration::from_secs(10u64),
            flyzones: Vec::new(),
            obstacles: Vec::new(),
            wp_list: LinkedList::new(),
            // private
            initialized: false,
            origin: Point::from_degrees(0f64, 0f64, 0f32),
            heading: (0f32, 0f32),
            current_wp: Waypoint::new(0, Point::from_degrees(0f64, 0f64, 0f32), 0f32),
            obstacle_list: HashSet::new(),
            start_node: Node::new(0, 0),
            end_node: Node::new(0, 0),
            open_heap: BinaryHeap::new(),
            open_set: HashSet::new(),
            close_list: HashSet::new(),
            obstacle_found: false,
            critical_nodes: HashSet::new(),
            DIRECT_PATH_MODIFIER_WEIGHT: direct_path_modifier_weight,
            HEADING_MODIFIER_WEIGHT: heading_modifier_weight,
        }
    }

    pub fn init(&mut self, grid_size: f32, flyzones: Vec<Vec<Point>>, obstacles: Vec<Obstacle>) {
        self.grid_size = grid_size;
        self.buffer = grid_size.max(MIN_BUFFER);
        self.origin = Pathfinder::find_origin(&flyzones);
        self.flyzones = flyzones;
        self.obstacles = obstacles;
        self.populate_map();
        self.initialized = true;

        #[cfg(feature = "debug")]
        DEBUGGER.with(|debugger| {
            let (mut max_x, mut max_y): (u32, u32) = (0, 0);
            for i in &self.obstacle_list {
                if i.x > 0 && i.x as u32 > max_x {
                    max_x = i.x as u32;
                }
                if i.y > 0 && i.y as u32 > max_y {
                    max_y = i.y as u32;
                }
            }

            debugger
                .borrow_mut()
                .set_size(max_x.min(THRESHOLD), max_y.min(THRESHOLD));
            debugger
                .borrow_mut()
                .set_obstacles(self.obstacle_list.clone());
        });
    }

    // Initilization
    fn find_origin(flyzones: &Vec<Vec<Point>>) -> Point {
        const MAX_RADIAN: f64 = 2f64 * std::f64::consts::PI;
        let mut min_lat = MAX_RADIAN;
        let mut min_lon = MAX_RADIAN;
        let mut max_lon = 0f64;
        let mut lon = min_lon;

        assert!(flyzones.len() > 0, "Require at least one flyzone");
        for i in 0..flyzones.len() {
            let flyzone_points = &flyzones[i];
            assert!(
                flyzone_points.len() > 2,
                "Require at least 3 points to construct fly zone."
            );

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

        Point::from_radians(min_lat, lon, 0f32)
    }

    fn populate_map(&mut self) {
        self.obstacle_list.clear();
        self.generate_fly_zone();
        self.generate_obstacles();
    }

    fn draw_line(
        &mut self,
        mut indep: f32,
        mut dep: f32,
        indep_goal: f32,
        dep_goal: f32,
        slope: f32,
        invert: bool,
    ) {
        const INCREMENT: f32 = 0.1;
        if indep > indep_goal {
            while indep > indep_goal && !(indep == indep_goal && dep == dep_goal) {
                indep -= INCREMENT;
                dep -= INCREMENT * slope;
                let buffer;
                if invert {
                    self.obstacle_list
                        .insert(Node::new(dep.floor() as i32, indep.floor() as i32));
                    buffer = Node::new(dep as i32, indep as i32 + 1);
                } else {
                    self.obstacle_list
                        .insert(Node::new(indep.floor() as i32, dep.floor() as i32));
                    buffer = Node::new(indep as i32 + 1, dep as i32);
                }
                self.obstacle_list.insert(buffer);
            }
        } else {
            while indep < indep_goal && !(indep == indep_goal && dep == dep_goal) {
                indep += INCREMENT;
                dep += INCREMENT * slope;
                let buffer;
                if invert {
                    self.obstacle_list
                        .insert(Node::new(dep.floor() as i32, indep.floor() as i32));
                    buffer = Node::new(dep as i32, indep as i32 - 1);
                } else {
                    self.obstacle_list
                        .insert(Node::new(indep.floor() as i32, dep.floor() as i32));
                    buffer = Node::new(indep as i32 - 1, dep as i32);
                }
                self.obstacle_list.insert(buffer);
            }
        }
    }

    fn generate_fly_zone(&mut self) {
        for i in 0..self.flyzones.len() {
            let flyzone_points = self.flyzones[i].clone();
            let mut pre_node: Node = flyzone_points[flyzone_points.len() - 1].to_node(&self);
            let mut end_node;

            for end_point in flyzone_points {
                end_node = end_point.to_node(&self);

                let slope = (end_node.y - pre_node.y) as f32 / (end_node.x - pre_node.x) as f32;
                if slope.abs() <= 1f32 {
                    self.draw_line(
                        pre_node.x as f32,
                        pre_node.y as f32,
                        end_node.x as f32,
                        end_node.y as f32,
                        slope,
                        false,
                    );
                } else {
                    self.draw_line(
                        pre_node.y as f32,
                        pre_node.x as f32,
                        end_node.y as f32,
                        end_node.x as f32,
                        1f32 / slope,
                        true,
                    );
                }
                pre_node = end_node;
            }
        }
    }

    fn generate_obstacles(&mut self) {
        for obst in &self.obstacles {
            let radius = ((obst.radius + self.buffer) / (self.grid_size)) as i32;
            let n = obst.coords.to_node(&self);

            for x in n.x - radius..n.x + radius {
                let dy = ((radius.pow(2) - (x - n.x).pow(2)) as f32).sqrt() as i32;
                self.obstacle_list.insert(Node::new(x, n.y + dy));
                self.obstacle_list.insert(Node::new(x, n.y + dy - 1));
                self.obstacle_list.insert(Node::new(x, n.y - dy));
                self.obstacle_list.insert(Node::new(x, n.y - dy - 1));
            }
            for y in n.y - radius..n.y + radius {
                let dx = ((radius.pow(2) - (y - n.y).pow(2)) as f32).sqrt() as i32;
                self.obstacle_list.insert(Node::new(n.x + dx, y));
                self.obstacle_list.insert(Node::new(n.x + dx - 1, y));
                self.obstacle_list.insert(Node::new(n.x - dx, y));
                self.obstacle_list.insert(Node::new(n.x - dx - 1, y));
            }
        }
    }

    pub fn get_adjust_path(
        &mut self,
        plane: Plane,
        mut wp_list: LinkedList<Waypoint>,
    ) -> &LinkedList<Waypoint> {
        assert!(self.initialized);
        self.start_time = SystemTime::now();
        self.wp_list = LinkedList::new();
        let mut current_loc: Point;
        let mut next_loc: Point;

        #[cfg(feature = "debug")]
        DEBUGGER.with(|debugger| {
            debugger.borrow().draw();
        });
        // First destination is first waypoint
        match wp_list.pop_front() {
            Some(wp) => self.current_wp = wp,
            None => return &self.wp_list,
        }

        if plane.yaw > 0f32 {
            let adjust_yaw = (90f32 - plane.yaw).to_radians();
            self.heading = (adjust_yaw.cos(), adjust_yaw.sin());
        };
        current_loc = plane.location;
        next_loc = self.current_wp.location;
        self.adjust_path(current_loc, next_loc);
        // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint

        loop {
            let mut reference = current_loc;
            if let Some(previous) = self.wp_list.back() {
                // New waypoints inserted
                if previous.index == self.current_wp.index {
                    reference = previous.location;
                }
            }
            let (p_node, c_node) = (reference.to_node(&self), next_loc.to_node(&self));
            let (dx, dy) = node_vector(&p_node, &c_node);
            let dist = dx.hypot(dy);
            self.heading = (dx / dist, dy / dist); //normalize vector

            current_loc = self.current_wp.location;
            match wp_list.pop_front() {
                Some(wp) => self.current_wp = wp,
                None => break,
            }
            next_loc = self.current_wp.location;

            if let Some(mut wp_list) = self.adjust_path(current_loc, next_loc) {
                self.wp_list.append(&mut wp_list);
            } else {
                break;
            }
            // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint
        }

        #[cfg(feature = "debug")]
        DEBUGGER.with(|debugger| {
            debugger.borrow().draw();
        });

        &self.wp_list
    }

    // Find best path using the a* algorithm
    // Return path if found and none if any error occured or no path found
    fn adjust_path(&mut self, start: Point, end: Point) -> Option<LinkedList<Waypoint>> {
        self.open_heap = BinaryHeap::new();
        self.open_set = HashSet::new();
        self.close_list = HashSet::new();
        self.start_node = start.to_node(&self);
        self.end_node = end.to_node(&self);
        self.obstacle_found = false;

        let dx = (self.end_node.x - self.start_node.x).abs();
        let dy = (self.end_node.y - self.start_node.y).abs();
	// here we define the new dz variable
	let dz = (self.end_node.z - self.start_node.z).abs();
        let start_node = Rc::new(Node {
            x: self.start_node.x,
            y: self.start_node.y,
	    // here we define the z variable 
	    z: self.start_node.z,
            g_cost: 0f64,
            f_cost: (dx + dy) as f64 + (SQRT_2 - 2f64 * UNIT_COST) * dx.min(dy) as f64,
            parent: None,
            depth: 0,
        });
        self.open_set.insert(start_node.clone());
        self.open_heap.push(start_node.clone());
        let mut current_node: Rc<Node>;

        loop {
            if let Ok(elapsed) = self.start_time.elapsed() {
                if elapsed > self.max_process_time {
                    return None;
                }
            } else {
                return None;
            }

            if let Some(node) = self.open_heap.pop() {
                current_node = node;
            } else {
                break;
            }

            if *current_node == self.end_node {
                if self.obstacle_found {
                    return Some(self.generate_path(current_node.clone(), end.alt() - start.alt()));
                }
                return Some(LinkedList::new());
            }
            self.open_set.take(&current_node);
            self.close_list.insert(current_node.clone());

            // Regular a* node discovery
            self.discover_node(current_node.clone());
        }
        eprintln!("No path found!");
        return None;
    }

    fn discover_node(&mut self, current_node: Rc<Node>) {
        for &(x_offset, y_offset, g_cost) in OFFSET.into_iter() {
            let mut new_node;
            new_node = Node::new(current_node.x + x_offset, current_node.y + y_offset);

            #[cfg(feature = "debug")]
            DEBUGGER.with(|debugger| {
                debugger.borrow_mut().add_to_explored(new_node.clone());
            });

            if self.obstacle_list.contains(&new_node) {
                self.critical_nodes.insert(current_node.clone());
                self.obstacle_found = true;
                continue;
            }
            if self.close_list.contains(&new_node) {
                continue;
            }
            new_node.g_cost = current_node.g_cost + g_cost;

            if let Some(node) = self.open_set.take(&new_node) {
                if new_node.g_cost >= node.g_cost {
                    self.open_set.insert(node);
                    continue;
                }
            }

            let v = node_vector(&new_node, &self.end_node);
            let (dx, dy) = (v.0.abs() as f64, v.1.abs() as f64);
            let base_f_cost = dx + dy + (SQRT_2 - 2f64 * UNIT_COST) * dx.min(dy);

            let direct_path_modifier =
                cross_product(node_vector(&self.start_node, &self.end_node), v).abs();
            // Signs flipped because heading is start -> end whereas path backtracks
            // (except flipping start and end doesn't work??? WTF geometry????)
            // #TODO: figure out how the math is working
            /*
                Experimental vector normalization using matrices
                #BUG: should not be using self.heading as it is not an accurate representation
                    of the heading of the node
                    Solution 1: use immediate parent
                    Solution 2: trace back to last critical node
                        - Memoizing vs dynamic traceback
                    currently using solution 1
            */
            let current_heading = match current_node.parent {
                Some(ref parent) => node_vector(&parent, &current_node),
                None => self.heading,
            };
            println!("current heading: {:?}", current_heading);
            let dir_vector = halve_vector(current_heading, (x_offset as f32, y_offset as f32));
            println!("offset: {:?}", (x_offset, y_offset));
            println!("dir vector: {:?}", dir_vector);
            let heading_modifier = cross_product(current_heading, dir_vector).abs();
            println!("heading modifier: {:?}\n", heading_modifier);
            //*/
            // let heading_modifier =
            // if dot_product(self.heading, node_vector(&new_node, &self.end_node)) < 0f32 {
            //     cross_product(self.heading, node_vector(&new_node, &self.end_node)).abs()
            // } else {
            //     node_distance(&new_node, &self.end_node)
            // };
            new_node.f_cost = new_node.g_cost
                + direct_path_modifier as f64 * self.DIRECT_PATH_MODIFIER_WEIGHT
                // + heading_modifier as f64 * self.HEADING_MODIFIER_WEIGHT
                + base_f_cost;

            new_node.parent = Some(current_node.clone());
            new_node.depth = current_node.depth + 1;
            let new_node = Rc::new(new_node);
            self.open_set.insert(new_node.clone());
            self.open_heap.push(new_node.clone());
        }
    }

    fn generate_path(&mut self, mut current_node: Rc<Node>, alt_diff: f32) -> LinkedList<Waypoint> {
        let mut wp_list = LinkedList::new();
        let mut previous_node;
        let mut wp_cluster_count = 1;
        // Node representing the sum of a cluster of nodes
        let mut wp_cluster = Node::new(current_node.x, current_node.y);
        let mut last_critical_node = current_node.clone();
        let mut last_vertex = current_node.clone();
        let mut new_inst_dir;
        let mut new_dir;
        let mut inst_dir = (0, 0);
        let mut dir = (0, 0);
        let current_alt = self.current_wp.location.alt();
        let alt_increment = alt_diff / current_node.depth as f32;

        if let Some(ref parent) = current_node.parent {
            dir = (current_node.x - parent.x, current_node.y - parent.y);
        }

        loop {
            previous_node = current_node;
            current_node = match previous_node.parent {
                Some(ref parent) => parent.clone(),
                None => break,
            };

            #[cfg(feature = "debug")]
            DEBUGGER.with(|debugger| {
                debugger.borrow_mut().add_to_path((*current_node).clone());
            });

            new_inst_dir = (
                current_node.x - previous_node.x,
                current_node.y - previous_node.y,
            );
            if new_inst_dir.0 != inst_dir.0 || new_inst_dir.1 != inst_dir.1 {
                inst_dir = new_inst_dir;
                new_dir = (
                    current_node.x - last_vertex.x,
                    current_node.y - last_vertex.y,
                );

                if new_dir == dir {
                    continue;
                }
                dir = new_dir;
                last_vertex = current_node.clone();

                if !self.critical_nodes.contains(&current_node) {
                    continue;
                }
                let waypoint;
                if node_distance(&current_node, &last_critical_node) * self.grid_size
                    < 2f32 * self.current_wp.radius
                {
                    let _old = wp_list.pop_front();

                    #[cfg(feature = "debug")]
                    DEBUGGER.with(|debugger| {
                        if let Some(_old) = _old {
                            debugger.borrow_mut().remove_from_wp(&last_critical_node);
                        }
                    });

                    wp_cluster_count += 1;
                    wp_cluster.advance(current_node.x, current_node.y);
                    wp_cluster.depth += current_node.depth;
                    let mut midpoint = Node::new(
                        wp_cluster.x / wp_cluster_count,
                        wp_cluster.y / wp_cluster_count,
                    );
                    midpoint.depth = wp_cluster.depth / wp_cluster_count;

                    #[cfg(feature = "debug")]
                    DEBUGGER.with(|debugger| {
                        debugger.borrow_mut().add_to_wp(midpoint.clone());
                    });

                    waypoint = self.current_wp.extend(
                        midpoint.to_point(&self),
                        current_alt - alt_increment * midpoint.depth as f32,
                    );
                    last_critical_node = Rc::new(midpoint);
                } else {
                    wp_cluster_count = 1;
                    wp_cluster = Node::new(current_node.x, current_node.y);
                    last_critical_node = current_node.clone();

                    #[cfg(feature = "debug")]
                    DEBUGGER.with(|debugger| {
                        debugger.borrow_mut().add_to_wp((*current_node).clone());
                    });

                    waypoint = self.current_wp.extend(
                        current_node.to_point(&self),
                        current_alt - alt_increment * current_node.depth as f32,
                    );
                }
                wp_list.push_front(waypoint);
            }
        }

        wp_list
    }

    pub fn set_grid_size(&mut self, grid_size: f32) {
        self.grid_size = grid_size;
        self.populate_map();
    }

    pub fn set_buffer(&mut self, new_buffer: f32) {
        self.buffer = new_buffer;
        self.populate_map();
    }

    pub fn set_process_time(&mut self, max_process_time: u32) {
        self.max_process_time = Duration::from_secs(max_process_time as u64);
    }

    pub fn set_flyzone(&mut self, flyzone: Vec<Vec<Point>>) {
        self.flyzones = flyzone;
        self.populate_map();
    }

    pub fn set_obstacle_list(&mut self, obstacle_list: Vec<Obstacle>) {
        self.obstacles = obstacle_list;
        self.populate_map();
    }

    pub fn get_grid_size(&self) -> f32 {
        self.grid_size
    }

    pub fn get_buffer(&self) -> f32 {
        self.buffer
    }

    pub fn get_process_time(&self) -> u32 {
        self.max_process_time.as_secs() as u32
    }

    pub fn get_flyzone(&mut self) -> &Vec<Vec<Point>> {
        &self.flyzones
    }

    pub fn get_obstacle_list(&self) -> &Vec<Obstacle> {
        &self.obstacles
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn invalid_flyzones_test() {
        Pathfinder::new().init(1f32, vec![], Vec::new())
    }

    #[test]
    #[should_panic]
    fn invalid_flyzone_test() {
        Pathfinder::new().init(1f32, vec![vec![]], Vec::new())
    }

    #[test]
    fn origin_test() {
        let mut pathfinder = Pathfinder::new();
        pathfinder.init(
            1.0,
            vec![vec![
                Point::from_degrees(50.06638888888889, -5.714722222222222, 0f32),
                Point::from_degrees(58.64388888888889, -5.714722222222222, 0f32),
                Point::from_degrees(50.06638888888889, -3.0700000000000003, 0f32),
            ]],
            Vec::new(),
        );
        let origin = pathfinder.origin;
        println!("Origin: {}, {}", origin.lat_degree(), origin.lon_degree());
        assert_eq!(origin.lat_degree(), 50.06638888888889);
        assert_eq!(origin.lon_degree(), -5.714722222222222);
    }

    #[test]
    fn irregular_origin_test() {
        let mut pathfinder = Pathfinder::new();
        pathfinder.init(
            1.0,
            vec![vec![
                Point::from_degrees(30.32276, -97.60398, 0f32),
                Point::from_degrees(30.32173, -97.60008, 0f32),
                Point::from_degrees(30.32082, -97.60368, 0f32),
            ]],
            Vec::new(),
        );
        let origin = pathfinder.origin;
        println!("Origin: {}, {}", origin.lat_degree(), origin.lon_degree());
        assert_eq!(origin.lat_degree(), 30.32082);
        assert_eq!(origin.lon_degree(), -97.60398);
    }

}
