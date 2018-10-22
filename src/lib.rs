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
    fn valid_path(&mut self, a: &Point, b: &Point) -> bool {
        // latitude is y, longitude is x
        // flyzone is array connected by each index
        // implementing intersect code in: http://developer.classpath.org/doc/java/awt/geom/Line2D-source.html
        for flyzone in &self.flyzones {}
        false
    }

    // helper function for intersection calculation
    // returns the area between three points
    fn area(a: &Point, b: &Point, c: &Point) -> f64 {
        (b.lon() - a.lon()) * (c.lat() - a.lat()) - (c.lon() - a.lon()) * (b.lat() - a.lat())
    }

    // helper function for intersection calculation
    // returns true if point c is between a and b, false otherwise
    fn between(a: &Point, b: &Point, c: &Point) -> bool {
        if a.lon() != b.lon() {
            (a.lon() <= c.lon() && c.lon() <= b.lon()) || (a.lon() >= c.lon() && c.lon() >= b.lon())
        } else {
            (a.lat() <= c.lat() && c.lat() <= b.lat()) || (a.lat() >= c.lat() && c.lat() >= b.lat())
        }
    }

    // calculate the intersection between four given points
    // implement: http://developer.classpath.org/doc/java/awt/geom/Line2D-source.html
    // returns true if a line segment a to b and another segment c to d intersect
    fn intersect(a: &Point, b: &Point, c: &Point, d: &Point) -> bool {
        let (a1, a2, a3, a4) = (0f64, 0f64, 0f64, 0f64);
        // special cases of intersection
        let a1 = Self::area(a, b, c);
        let a2 = Self::area(a, b, d);
        let a3 = Self::area(c, d, a);
        let a4 = Self::area(c, d, b);
        if a1 == 0f64 {
            // checks if c is between a and b OR
            // d is colinear also AND between a and b or at opposite ends?
            if Self::between(a, b, c) {
                return true;
            } else {
                if Self::area(a, b, d) == 0f64 {
                    return Self::between(c, d, a) || Self::between(c, d, b);
                } else {
                    return false;
                }
            }
        } else if a2 == 0f64 {
            // check if d is between a and b since c is not colinear
            return Self::between(a, b, d);
        }
        if a3 == 0f64 {
            // checks if a is between c and d OR
            // b is colinear AND either between a and b or at opposite ends?
            if Self::between(c, d, a) {
                return true;
            } else {
                if Self::area(c, d, b) == 0f64 {
                    return Self::between(a, b, c) || Self::between(a, b, d);
                } else {
                    return false;
                }
            }
        } else if a4 == 0f64 {
            // check if b is between c and d since we know a is not colinear
            return Self::between(c, d, b);
        }
        //tests for regular intersection
        else {
            ((a1 > 0f64) ^ (a2 > 0f64)) && ((a3 > 0f64) ^ (a4 > 0f64))
        }
    }

    // Generate all possible path (tangent lines) between two nodes, and return the
    // shortest valid path if one exists
    fn find_path(&self, a: &Rc<Node>, b: &Rc<Node>) -> Option<Connection> {
        unimplemented!();
    }

    fn build_graph(&mut self) {
        let mut candidates = self.nodes.clone();
        for (a, x) in &mut self.nodes.clone() {
            for (b, y) in &mut candidates {
                if let Some(path) = self.find_path(a, b) {
                    x.insert(path.reciprocal());
                    y.insert(path);
                }
            }
            candidates.remove(a); // Remove a from candidate pool
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

    #[test]
    fn is_between() {
        let a = Point::from_radians(40f64, 40f64, 10f32);
        let b = Point::from_radians(40f64, 50f64, 10f32);
        let c = Point::from_radians(40f64, 60f64, 10f32);
        assert_eq!(Pathfinder::between(&a, &c, &b), true);
    }

    #[test]
    fn is_colinear() {
        let a = Point::from_radians(40f64, 40f64, 10f32);
        let b = Point::from_radians(40f64, 50f64, 10f32);
        let c = Point::from_radians(40f64, 60f64, 10f32);
        assert_eq!(Pathfinder::area(&a, &b, &c), 0f64);
    }

    #[test]
    fn yes_intersect() {
        let a = Point::from_radians(40f64, 0f64, 10f32);
        let b = Point::from_radians(40f64, 40f64, 10f32);
        let c = Point::from_radians(0f64, 0f64, 10f32);
        let d = Point::from_radians(0f64, 40f64, 10f32);
        assert_eq!(Pathfinder::intersect(&a, &d, &b, &c), true);
    }

    #[test]
    fn no_intersect() {
        let a = Point::from_radians(40f64, 0f64, 10f32);
        let b = Point::from_radians(40f64, 40f64, 10f32);
        let c = Point::from_radians(0f64, 0f64, 10f32);
        let d = Point::from_radians(0f64, 40f64, 10f32);
        assert_eq!(Pathfinder::intersect(&a, &c, &b, &d), false);
    }

}
