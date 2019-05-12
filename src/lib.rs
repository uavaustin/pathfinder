#![allow(dead_code)]
#![allow(unused_variables)]

extern crate ordered_float;

use std::cell::RefCell;
use std::collections::BinaryHeap;
use std::collections::HashSet;
use std::collections::LinkedList;
use std::f32::consts::PI;
use std::rc::Rc;
use std::time::{Duration, SystemTime};

mod graph;
pub mod obj;
mod queue;

use graph::util::{intersect, output_graph};
use graph::{Connection, Node, Point, Vertex};
use obj::{Location, Obstacle, Plane, Waypoint};

const EQUATORIAL_RADIUS: f64 = 63781370.0;
const POLAR_RADIUS: f64 = 6356752.0;
const RADIUS: f64 = 6371000.0;
const MIN_BUFFER: f32 = 2f32;
const TURNING_RADIUS: f32 = 5f32; // In meters
const MAX_ANGLE: f32 = PI / 6f32;
const MAX_ANGLE_ASCENT: f32 = PI / 3f32;
const MAX_ANGLE_DESCENT: f32 = -PI / 3f32;
const START_VERTEX_INDEX: i32 = -1;
const END_VERTEX_INDEX: i32 = -2;
const HEADER_VERTEX_INDEX: i32 = -3;

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
    num_vertices: i32,
}

// Simple wrapper around heap and set for efficient data retrival
struct Queue {
    heap: BinaryHeap<Rc<RefCell<Vertex>>>, // Efficiently get min
    set: HashSet<i32>,                     // Efficiently check of existence
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
            num_vertices: 0i32,
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
        for i in 0..self.flyzones.len() {
            if self.invalid_flyzone(i) {
                panic!();
            }
        }
        self.obstacles = obstacles;
        self.build_graph();
        self.initialized = true;
    }

    // determine if flyzone intersects itself (correct order)
    // inputs (self,flyzones indices), outputs true if invalid
    fn invalid_flyzone(&mut self, iter: usize) -> (bool) {
        let flyzone = &self.flyzones[iter];
        let mut vertices = Vec::new();
        for loc in 0..flyzone.len() {
            let i = flyzone[loc];
            let point = Point::from_location(&i, &self.origin);
            vertices.push(point);
        }
        let n = vertices.len();
        // compares any side of flyzone, ab, with any non-adjacent side, cd
        for ab in 0..n - 2 {
            let a = vertices[ab];
            let b = vertices[ab + 1];
            for i in 2..n - 1 {
                let cd = ab + i;
                let c = vertices[cd];
                let d = vertices[(cd + 1) % n];
                if intersect(&a, &b, &c, &d) {
                    return true;
                }
                if cd + 1 == n {
                    break;
                }
            }
        }
        return false;
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

        self.current_wp.location = plane.location;

        loop {
            current_loc = self.current_wp.location;
            match wp_list.pop_front() {
                Some(wp) => self.current_wp = wp,
                None => break,
            }
            next_loc = self.current_wp.location;

            if let Some(mut wp_list) = self.adjust_path(current_loc, next_loc) {
                println!("appending");
                self.wp_list.append(&mut wp_list);
            } else {
                println!("no path");
                break;
            }
            // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint
            // self.wp_list.push_back(Waypoint::from_degrees(0, 30.69, -97.69, 100f32, 10f32));
        }
        &self.wp_list
    }

    // Find best path using the a* algorithm
    // Return path if found and none if any error occured or no path found
    fn adjust_path(&mut self, start: Location, end: Location) -> Option<LinkedList<Waypoint>> {
        let mut path = None;
        let mut open_set = Queue::new(); // candidate vertices
        let mut closed_set: HashSet<i32> = HashSet::new(); // set of vertex already visited
        let mut temp_vertices: LinkedList<Rc<RefCell<Vertex>>> = LinkedList::new();
        let start_node = Rc::new(RefCell::new(Node::from_location(&start, &self.origin)));
        let end_node = Rc::new(RefCell::new(Node::from_location(&end, &self.origin)));
        let start_vertex = Rc::new(RefCell::new(Vertex::new(
            &mut START_VERTEX_INDEX,
            start_node.clone(),
            0f32,
            None,
        )));
        let end_point = Point::from_location(&end, &self.origin);
        let min_height = if start.alt() > end.alt() {
            end.alt()
        } else {
            start.alt()
        };
        println!("Height threshold {}", min_height);

        //Prepare graph for A*
        println!("\n[ Inserting temp vertices ]");
        for i in 0..self.nodes.len() {
            let temp_node = &self.nodes[i];
            let (temp_paths, _) = self.find_path(&start_node.borrow(), &temp_node.borrow());
            println!("[start {}]: path count -> {}", i, temp_paths.len());

            for (a, b, dist, threshold) in temp_paths {
                if min_height < threshold {
                    continue;
                }

                println!("Inserting start vertex {}", self.num_vertices);
                let mut vertex = Vertex::new(&mut self.num_vertices, temp_node.clone(), b, None);
                vertex.parent = Some(start_vertex.clone());
                vertex.g_cost = dist;
                vertex.f_cost = dist + vertex.location.distance(&end_point);
                let vertex_p = Rc::new(RefCell::new(vertex));
                temp_node.borrow_mut().insert_vertex(vertex_p.clone());
                open_set.push(vertex_p.clone());
                temp_vertices.push_back(vertex_p.clone());
            }

            let (temp_paths, _) = self.find_path(&temp_node.borrow(), &end_node.borrow());
            println!("[end {}]: path count -> {}", i, temp_paths.len());

            for (a, b, dist, threshold) in temp_paths {
                println!("Inserting end vertex {}", self.num_vertices);
                let end_vertex = Rc::new(RefCell::new(Vertex::new(
                    &mut END_VERTEX_INDEX,
                    end_node.clone(),
                    b,
                    None,
                )));
                let connection = Connection::new(end_vertex.clone(), dist, threshold);
                let vertex = Rc::new(RefCell::new(Vertex::new(
                    &mut self.num_vertices,
                    temp_node.clone(),
                    a,
                    Some(connection),
                )));
                temp_node.borrow_mut().insert_vertex(vertex.clone());
                temp_vertices.push_back(vertex.clone());
            }
        }

        output_graph(&self);
        println!("temporary vertices");
        for vert in &temp_vertices {
            let v_loc = vert.borrow().location.to_location(&self.origin);
            println!("{}, {}", v_loc.lat_degree(), v_loc.lon_degree());
        }

        //A* algorithm - find shortest path from plane to destination
        while let Some(cur) = open_set.pop() {
            assert!(cur.borrow().index != HEADER_VERTEX_INDEX);
            println!("current vertex {}", cur.borrow());
            if cur.borrow().index == END_VERTEX_INDEX {
                path = Some(self.generate_waypoint(cur));
                break;
            }
            closed_set.insert(cur.borrow().index);

            let mut update_vertex = |cur_g_cost: f32, next: Rc<RefCell<Vertex>>, dist: f32| {
                // Handle edge case when node only has one vertex
                if next.borrow().index == cur.borrow().index {
                    return;
                }
                let new_g_cost = cur_g_cost + dist;
                // println!("add vertex to queue {}", next.borrow());
                {
                    if closed_set.contains(&next.borrow().index)    //vertex is already explored
                        || next.borrow().sentinel                   //vertex is a sentinel
                        || (open_set.contains(&next) && new_g_cost >= next.borrow().g_cost)
                    {
                        //vertex has been visited and the current cost is better
                        // println!("Vertex addition skipped");
                        return;
                    }
                    let mut next_mut = next.borrow_mut();
                    let new_f_cost = new_g_cost + next_mut.location.distance(&end_point);
                    next_mut.g_cost = new_g_cost;
                    next_mut.f_cost = new_f_cost;
                    next_mut.parent = Some(cur.clone());
                }
                open_set.push(next.clone());
            };

            let cur_vertex = cur.borrow();
            let g_cost = cur_vertex.g_cost;
            if let Some(ref connection) = cur_vertex.connection {
                // println!(
                //     "Adding connection {} to queue",
                //     connection.neighbor.borrow().index
                // );
                // Only add vertex if height meets threshold requirement
                if min_height > connection.threshold {
                    // println!("Met threshold requirement of {}", connection.threshold);
                    let mut next = connection.neighbor.clone();
                    let dist = connection.distance;
                    update_vertex(g_cost, next, dist);
                }
            }

            let mut weight = cur_vertex.get_neighbor_weight();
            if let Some(ref next_vertex) = cur_vertex.next {
                // println!("Adding neighbor {} to queue", next_vertex.borrow().index);
                let mut next = next_vertex.clone();
                // If next is header, skip to header neighbor
                if next.borrow().index == HEADER_VERTEX_INDEX {
                    weight += next.borrow().get_neighbor_weight();
                    next = match next_vertex.borrow().next {
                        Some(ref true_next) => true_next.clone(),
                        None => panic!("broken chain"),
                    }
                }

                update_vertex(g_cost, next, weight);
            }
        }

        Node::prune_vertices(temp_vertices);
        path
    }

    fn generate_waypoint(&self, end_vertex: Rc<RefCell<Vertex>>) -> LinkedList<Waypoint> {
        let mut waypoint_list = LinkedList::new();
        let mut cur_vertex = end_vertex;
        let mut index = END_VERTEX_INDEX;
        println!("Generating waypoints");
        while index != START_VERTEX_INDEX {
            let loc = cur_vertex.borrow().location.to_location(&self.origin);
            let radius = cur_vertex.borrow().radius;

            waypoint_list.push_front(Waypoint::new(1, loc, radius));
            println!("{}", cur_vertex.borrow());
            let parent = match cur_vertex.borrow().parent {
                Some(ref cur_parent) => cur_parent.clone(),
                None => panic!("Missing a parent without reaching start point"),
            };
            cur_vertex = parent;
            index = cur_vertex.borrow().index;
        }
        waypoint_list
    }

    // Note: bypasses min buffer restriction
    pub fn set_buffer(&mut self, buffer: f32) {
        self.buffer = buffer;
        self.build_graph();
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
    use graph::Point;

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
    #[should_panic]
    fn fz_fz_intersection_test() {
        let origin = Location::from_degrees(0f64, 0f64, 0f32);
        let a = Point::new(0f32, 0f32, 10f32).to_location(&origin);
        let b = Point::new(20f32, 0f32, 10f32).to_location(&origin);
        let c = Point::new(20f32, 20f32, 10f32).to_location(&origin);
        let d = Point::new(0f32, 20f32, 10f32).to_location(&origin);
        let test_flyzone = vec![vec![a, b, d, c]];
        let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
        assert!(pathfinder.invalid_flyzone(0));
    }
}
