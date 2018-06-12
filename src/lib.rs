#![allow(dead_code)]
#![allow(unused_variables)]

#[cfg(feature = "debug")]
#[macro_use]
extern crate conrod;

use std::collections::BinaryHeap;
use std::collections::HashSet;
use std::collections::LinkedList;
use std::f64::consts::SQRT_2;
use std::rc::Rc;
use std::time::{Duration, SystemTime};

mod node;
mod obj;
use node::*;
pub use obj::*;

#[cfg(feature = "debug")]
#[path=""]
mod debug_block {
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

const OFFSET: [(i32, i32, f64); 8] = [
    (-1, -1, SQRT_2),
    (1, 1, SQRT_2),
    (-1, 1, SQRT_2),
    (1, -1, SQRT_2),
    (1, 0, 1f64),
    (-1, 0, 1f64),
    (0, -1, 1f64),
    (0, 1, 1f64),
];

const EQUATORIAL_RADIUS: f64 = 63781370.0;
const POLAR_RADIUS: f64 = 6356752.0;
const RADIUS: f64 = 6371000.0;

pub struct Pathfinder {
    initialized: bool,
    grid_size: f32, // In meters
    buffer: f32,    // In meters
    start_time: SystemTime,
    max_process_time: Duration, // In seconds
    origin: Point,
    fly_zones: Vec<Vec<Point>>,
    obstacles: Vec<Obstacle>,
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
}

impl Pathfinder {
    pub fn new() -> Pathfinder {
        Pathfinder {
            initialized: false,
            grid_size: 1f32,
            buffer: 1f32,
            start_time: SystemTime::now(),
            max_process_time: Duration::from_secs(10u64),
            origin: Point::from_degrees(0f64, 0f64, 0f32),
            fly_zones: Vec::new(),
            obstacles: Vec::new(),
            current_wp: Waypoint::new(0, Point::from_degrees(0f64, 0f64, 0f32), 0f32),
            wp_list: LinkedList::new(),
            obstacle_list: HashSet::new(),
            start_node: Node::new(0, 0),
            end_node: Node::new(0, 0),
            open_heap: BinaryHeap::new(),
            open_set: HashSet::new(),
            close_list: HashSet::new(),
            obstacle_found: false,
            critical_nodes: HashSet::new(),
        }
    }

    pub fn init(&mut self, grid_size: f32, flyzones: Vec<Vec<Point>>, obstacles: Vec<Obstacle>) {
        self.grid_size = grid_size;
        self.buffer = if grid_size > 5f32 { grid_size } else { 5f32 };
        self.origin = Pathfinder::find_origin(&flyzones);
        self.fly_zones = flyzones;
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

            if max_x > THRESHOLD {
                max_x = THRESHOLD;
            }
            if max_y > THRESHOLD {
                max_y = THRESHOLD;
            }
            debugger.borrow_mut().set_size(max_x, max_y);
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

        Point::from_radians(min_lat, lon, 0f32)
    }

    fn populate_map(&mut self) {
        self.obstacle_list.clear();
        self.generate_fly_zone();
        self.generate_obstacles();
    }

    fn generate_fly_zone(&mut self) {
        for i in 0..self.fly_zones.len() {
            let flyzone_points = &self.fly_zones[i];
            let first_node: Node = flyzone_points[0].to_node(&self);
            let mut pre_node: Node = flyzone_points[flyzone_points.len() - 1].to_node(&self);
            let mut end_node;
            let mut index = 0;

            for end_point in flyzone_points {
                end_node = end_point.to_node(&self);
                let point = end_node.to_point(&self);

                index += 1;
                let mut cur_x = pre_node.x;
                let mut cur_y = pre_node.y;

                if pre_node.x == end_node.x {
                    while cur_y != end_node.y {
                        //AddPoint cur_x, cur_x
                        self.obstacle_list.insert(Node::new(cur_x, cur_y));

                        if pre_node.y > end_node.y {
                            cur_y -= 1;
                        } else {
                            cur_y += 1;
                        }
                    }
                } else {
                    let top: f32 = (end_node.y - pre_node.y) as f32;
                    let bot: f32 = (end_node.x - pre_node.x) as f32;
                    let slope: f32 = top / bot;
                    let mut cur_x_f32: f32 = cur_x as f32;
                    let mut cur_y_f32: f32 = cur_y as f32;

                    while ((cur_x >= end_node.x && pre_node.x > end_node.x)
                        || (cur_x <= end_node.x && pre_node.x < end_node.x))
                        && !(cur_x == end_node.x && cur_y == end_node.y)
                    {
                        if pre_node.x > end_node.x {
                            cur_x_f32 = cur_x_f32 + (-1f32 * 0.1);
                            cur_y_f32 = cur_y_f32 + (-1f32 * 0.1 * slope);
                        } else {
                            cur_x_f32 = cur_x_f32 + 0.1;
                            cur_y_f32 = cur_y_f32 + (0.1f32 * slope);
                        }

                        if cur_x != cur_x_f32.floor() as i32 || cur_y != cur_y_f32.floor() as i32 {
                            cur_x = cur_x_f32.floor() as i32;
                            cur_y = cur_y_f32.floor() as i32;

                            self.obstacle_list.insert(Node::new(cur_x, cur_y));

                            if pre_node.x > end_node.x {
                                let add_buffer: Node = Node::new(cur_x + 1, cur_y);
                                self.obstacle_list.insert(add_buffer);
                            } else {
                                let add_buffer: Node = Node::new(cur_x - 1, cur_y);
                                self.obstacle_list.insert(add_buffer);
                            }
                        }
                    }
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
        let mut next_wp: Waypoint;
        match wp_list.pop_front() {
            Some(wp) => next_wp = wp,
            None => return &self.wp_list,
        }

        self.current_wp = next_wp; // First destination is first waypoint
        current_loc = self.current_wp.location;
        self.adjust_path(plane.location, current_loc);
        // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint

        loop {
            match wp_list.pop_front() {
                Some(wp) => next_wp = wp,
                None => break,
            }

            current_loc = self.current_wp.location;
            next_loc = next_wp.location;
            self.current_wp = next_wp;
            if !self.adjust_path(current_loc, next_loc) {
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
    // Return true if path found and false if any error occured or no path found
    fn adjust_path(&mut self, start: Point, end: Point) -> bool {
        self.open_heap = BinaryHeap::new();
        self.open_set = HashSet::new();
        self.close_list = HashSet::new();
        self.start_node = end.to_node(&self); // Reverse because backtracking at the end
        self.end_node = start.to_node(&self);
        self.obstacle_found = false;

        let dx = (self.end_node.x - self.start_node.x).abs();
        let dy = (self.end_node.y - self.start_node.y).abs();
        let start_node = Rc::new(Node {
            x: self.start_node.x,
            y: self.start_node.y,
            g_cost: 0f64,
            f_cost: (dx + dy) as f64 + (SQRT_2 - 2f64) * std::cmp::min(dx, dy) as f64,
            parent: None,
            depth: 0,
        });
        self.open_set.insert(start_node.clone());
        self.open_heap.push(start_node.clone());

        let mut current_node: Rc<Node>;

        loop {
            if let Ok(elapsed) = self.start_time.elapsed() {
                if elapsed > self.max_process_time {
                    return false;
                }
            } else {
                return false;
            }

            if let Some(node) = self.open_heap.pop() {
                current_node = node;
            } else {
                break;
            }

            if *current_node == self.end_node {
                if self.obstacle_found {
                    self.generate_path(current_node.clone(), end.alt() - start.alt());
                }
                return true;
            }
            self.open_set.take(&current_node);
            self.close_list.insert(current_node.clone());

            // Regular a* node discovery
            self.discover_node(current_node.clone());
        }
        eprintln!("No path found!");
        return false;
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

            let dx = self.end_node.x - new_node.x;
            let dy = self.end_node.y - new_node.y;
            let dx1 = self.start_node.x - self.end_node.x;
            let dy1 = self.start_node.y - self.end_node.y;
            let cross = (dx * dy1 - dx1 * dy).abs() as f64 * 0.001;
            let dx = dx.abs();
            let dy = dy.abs();
            new_node.f_cost = new_node.g_cost + cross + (dx + dy) as f64
                + (SQRT_2 - 2f64) * std::cmp::min(dx, dy) as f64;
            new_node.parent = Some(current_node.clone());
            new_node.depth = current_node.depth + 1;
            let new_node = Rc::new(new_node);
            self.open_set.insert(new_node.clone());
            self.open_heap.push(new_node.clone());
        }
    }

    fn generate_path(&mut self, mut current_node: Rc<Node>, alt_diff: f32) {
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
                if self.distance_between_nodes(&current_node, &last_critical_node)
                    < 2 * self.current_wp.radius as i64 {
                    let old = self.wp_list.pop_back();

                    #[cfg(feature = "debug")]
                    DEBUGGER.with(|debugger| {
                        if let Some(old) = old {
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

                    let waypoint = self.current_wp.extend(
                        midpoint.to_point(&self),
                        current_alt - alt_increment * midpoint.depth as f32,
                    );
                    self.wp_list.push_back(waypoint);
                    last_critical_node = Rc::new(midpoint);
                } else {
                    wp_cluster_count = 1;
                    wp_cluster = Node::new(current_node.x, current_node.y);
                    last_critical_node = current_node.clone();

                    #[cfg(feature = "debug")]
                    DEBUGGER.with(|debugger| {
                        debugger.borrow_mut().add_to_wp((*current_node).clone());
                    });

                    let waypoint = self.current_wp.extend(
                        current_node.to_point(&self),
                        current_alt - alt_increment * current_node.depth as f32,
                    );
                    self.wp_list.push_back(waypoint);
                }
            }
        }
    }

    pub fn set_buffer(&mut self, new_buffer: f32) {
        self.buffer = new_buffer;
        self.populate_map();
    }

    pub fn set_process_time(&mut self, max_process_time: u32) {
        self.max_process_time = Duration::from_secs(max_process_time as u64);
    }

    fn distance_between_nodes(&self, first_node: &Node, second_node: &Node) -> i64 {
        let x_diff: f64 = (first_node.x - second_node.x).pow(2) as f64;
        let y_diff: f64 = (first_node.y - second_node.y).pow(2) as f64;
        ((x_diff + y_diff).sqrt() * self.grid_size as f64).ceil() as i64
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

    pub fn get_origin(&self) -> Point {
        self.origin
    }

    pub fn get_process_time(&self) -> u32 {
        self.max_process_time.as_secs() as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
        let origin = pathfinder.get_origin();
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
        let origin = pathfinder.get_origin();
        println!("Origin: {}, {}", origin.lat_degree(), origin.lon_degree());
        assert_eq!(origin.lat_degree(), 30.32082);
        assert_eq!(origin.lon_degree(), -97.60398);
    }
}
