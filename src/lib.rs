#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]

extern crate ordered_float;

use std::cell::RefCell;
use std::collections::{BinaryHeap, HashMap, HashSet, LinkedList};
use std::f32::consts::PI;
use std::rc::Rc;
use std::time::{Duration, SystemTime};

pub mod obj;

mod graph;
mod node;
mod point;
use node::Node;
use obj::{Location, Obstacle, Plane, Waypoint};

const EQUATORIAL_RADIUS: f64 = 63781370.0;
const POLAR_RADIUS: f64 = 6356752.0;
const RADIUS: f64 = 6371000.0;
const MIN_BUFFER: f32 = 5f32;
const TURNING_RADIUS: f32 = 5f32; // In meters
const MAX_ANGLE: f64 = 3.141 / 6f64;

#[allow(non_snake_case)]
pub struct Pathfinder {
    // exposed API
    buffer: f32,                // In meters
    max_process_time: Duration, // In seconds
    flyzones: Vec<Vec<Location>>,
    obstacles: Vec<Obstacle>,
    // private
    initialized: bool,
    start_time: SystemTime,
    current_wp: Waypoint,
    wp_list: LinkedList<Waypoint>,
    origin: Location, // Reference point defining each node
    nodes: Vec<Rc<RefCell<Node>>>,
}

impl Pathfinder {
    pub fn new() -> Pathfinder {
        Self {
            // exposed API
            buffer: MIN_BUFFER,
            max_process_time: Duration::from_secs(10u64),
            flyzones: Vec::new(),
            obstacles: Vec::new(),
            // private
            initialized: false,
            start_time: SystemTime::now(),
            current_wp: Waypoint::from_degrees(0u32, 0f64, 0f64, 0f32, 1f32),
            wp_list: LinkedList::new(),
            origin: Location::from_degrees(0f64, 0f64, 0f32),
            nodes: Vec::new(),
        }
    }

    // Helper function to return an initialized pathfinder
    pub fn create(
        buffer_size: f32,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        let mut pathfinder = Pathfinder::new();
        pathfinder.init(buffer_size, flyzones, obstacles);
        pathfinder
    }

    pub fn init(
        &mut self,
        buffer_size: f32,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    ) {
        assert!(flyzones.len() >= 1);
        for flyzone in &flyzones {
            assert!(flyzone.len() >= 3);
        }
        self.buffer = buffer_size.max(MIN_BUFFER);
        self.flyzones = flyzones;
        self.obstacles = obstacles;
        self.build_graph();
        self.initialized = true;
    }

    pub fn get_adjust_path(
        &mut self,
        plane: Plane,
        mut wp_list: LinkedList<Waypoint>,
    ) -> &LinkedList<Waypoint> {
        assert!(self.initialized);
        self.start_time = SystemTime::now();
        self.wp_list = LinkedList::new();
        let mut current_loc: Location;
        let mut next_loc: Location;

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
    fn adjust_path(&mut self, start: Location, end: Location) -> Option<LinkedList<Waypoint>> {
        None
    }

    pub fn set_process_time(&mut self, max_process_time: u32) {
        self.max_process_time = Duration::from_secs(max_process_time as u64);
    }

    pub fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>) {
        self.flyzones = flyzone;
        self.build_graph();
    }

    pub fn set_obstacles(&mut self, obstacles: Vec<Obstacle>) {
        self.obstacles = obstacles;
        self.build_graph();
    }

    pub fn get_buffer_size(&self) -> f32 {
        self.buffer
    }

    pub fn get_buffer(&self) -> f32 {
        self.buffer
    }

    pub fn get_process_time(&self) -> u32 {
        self.max_process_time.as_secs() as u32
    }

    pub fn get_flyzone(&mut self) -> &Vec<Vec<Location>> {
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
