#![allow(dead_code)]
extern crate ordered_float;

use std::collections::{BinaryHeap, HashMap, HashSet, LinkedList};
use std::env;
use std::f64::consts::SQRT_2;
use std::fs::File;
use std::io::Read;
use std::rc::Rc;
use std::time::{Duration, SystemTime};

mod node;
mod obj;
pub use node::{Connection, Node};
pub use obj::{Obstacle, Plane, Point, Waypoint};

const EQUATORIAL_RADIUS: f64 = 63781370.0;
const POLAR_RADIUS: f64 = 6356752.0;
const RADIUS: f64 = 6371000.0;
const MIN_BUFFER: f32 = 5f32;
const TURNING_RADIUS: f32 = 5f32; // In meters

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
    current_wp: Waypoint,
    wp_list: LinkedList<Waypoint>,
    nodes: HashMap<Rc<Node>, HashSet<Connection>>,
}

impl Pathfinder {
    pub fn new() -> Pathfinder {
        Pathfinder {
            // exposed API
            grid_size: 1f32,
            buffer: 1f32,
            max_process_time: Duration::from_secs(10u64),
            flyzones: Vec::new(),
            obstacles: Vec::new(),
            // private
            initialized: false,
            start_time: SystemTime::now(),
            current_wp: Waypoint::from_degrees(0u32, 0f64, 0f64, 0f32, 1f32),
            wp_list: LinkedList::new(),
            nodes: HashMap::new(),
        }
    }

    pub fn init(&mut self, grid_size: f32, flyzones: Vec<Vec<Point>>, obstacles: Vec<Obstacle>) {
        self.grid_size = grid_size;
        self.buffer = grid_size.max(MIN_BUFFER);
        self.flyzones = flyzones;
        self.obstacles = obstacles;
        self.populate_nodes();
        self.initialized = true;
    }

    fn populate_nodes(&mut self) {
        for ref obs in self.obstacles.clone() {
            self.add_node(obs);
        }
    }

    fn add_node<T>(&mut self, input: T)
    where
        Node: From<T>,
    {
        let mut node = Node::from(input);
        node.set_index(self.nodes.len() as u32);
        self.nodes.insert(Rc::new(node), HashSet::new());
    }

    // check if a path is valid (i.e not block by obstacle or flightzone)
    fn valid_path(a: &Point, b: &Point) -> bool {
        false
    }

    // Generate all possible path (tangent lines) between two nodes, and return the
    // shortest valid path if one exists
    fn find_path(&self, a: &Rc<Node>, b: &Rc<Node>) -> Option<Connection> {
        unimplemented!();
    }

    fn build_graph(&mut self) {
        let mut candidates = self.nodes.clone();
        for (a, x) in &mut self.nodes.clone(){
            for (b, y) in &mut candidates {
                if let Some(path) = self.find_path(a, b) {
                    x.insert(path.reciprocal());
                    y.insert(path);
                }
            }
            candidates.remove(a);   // Remove a from candidate pool
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

        // First destination is first waypoint
        match wp_list.pop_front() {
            Some(wp) => self.current_wp = wp,
            None => return &self.wp_list,
        }

        current_loc = plane.location;
        next_loc = self.current_wp.location;
        self.adjust_path(current_loc, next_loc);
        // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint

        loop {
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

        &self.wp_list
    }

    // Find best path using the a* algorithm
    // Return path if found and none if any error occured or no path found
    fn adjust_path(&mut self, start: Point, end: Point) -> Option<LinkedList<Waypoint>> {
        unimplemented!();
    }

    pub fn set_process_time(&mut self, max_process_time: u32) {
        self.max_process_time = Duration::from_secs(max_process_time as u64);
    }

    pub fn set_flyzone(&mut self, flyzone: Vec<Vec<Point>>) {
        self.flyzones = flyzone;
    }

    pub fn set_obstacle_list(&mut self, obstacle_list: Vec<Obstacle>) {
        self.obstacles = obstacle_list;
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
}
