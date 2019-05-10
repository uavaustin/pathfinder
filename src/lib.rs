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

use graph::util::intersect;
use graph::{Connection, Node, Point, Vertex};
pub use obj::{Location, Obstacle, Plane, Waypoint};

const EQUATORIAL_RADIUS: f64 = 63781370.0;
const POLAR_RADIUS: f64 = 6356752.0;
const RADIUS: f64 = 6371000.0;
const MIN_BUFFER: f32 = 5f32;
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
    } //          let p1 = a.to_point(*j + theta0);
      //          println!("{} {:?}", j, &p1);
      //          let p2 = b.to_point(*i + theta0);
      //          println!("{} {:?}", i, &

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
                println!("no path");
                break;
            }
            // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint
        }

        &self.wp_list
    }

    // Find best path using the a* algorithm
    // Return path if found and none if any error occured or no path found
    fn adjust_path(&mut self, start: Location, end: Location) -> Option<LinkedList<Waypoint>> {
        let mut num_vertices = self.num_vertices;
        let mut open_list: BinaryHeap<Rc<RefCell<Vertex>>> = BinaryHeap::new();
        let mut open_set: HashSet<i32> = HashSet::new();
        let mut closed_set: HashSet<i32> = HashSet::new();
        let mut vertices_to_remove: LinkedList<Rc<RefCell<Vertex>>> = LinkedList::new();
        let start_node = Rc::new(RefCell::new(Node::from_location(&start, &self.origin)));
        let end_node = Rc::new(RefCell::new(Node::from_location(&end, &self.origin)));
        let start_vertex = Vertex::new(start_node.clone(), &mut START_VERTEX_INDEX, 0f32, None);

        //Prepare graph for A*
        for i in 0..self.nodes.len() {
            let temp_node = &self.nodes[i];
            let (temp_paths, _) = self.find_path(&start_node.borrow(), &temp_node.borrow());
            for (a, b, dist, thresh) in temp_paths.iter() {
                let vertex = Rc::new(RefCell::new(Vertex::new(
                    temp_node.clone(),
                    &mut num_vertices,
                    *b,
                    None,
                )));
                temp_node.borrow_mut().insert_vertex(vertex.clone());
                open_list.push(vertex.clone());
                vertices_to_remove.push_back(vertex.clone());
                open_set.insert(vertex.borrow().index);
            }

            let (temp_paths, _) = self.find_path(&temp_node.borrow(), &end_node.borrow());
            for (a, b, dist, thresh) in temp_paths.iter() {
                let end_vertex = Rc::new(RefCell::new(Vertex::new(
                    end_node.clone(),
                    &mut END_VERTEX_INDEX,
                    *b,
                    None,
                )));
                let connection = Connection::new(end_vertex.clone(), *dist, *thresh);
                let vertex = Rc::new(RefCell::new(Vertex::new(
                    temp_node.clone(),
                    &mut num_vertices,
                    *a,
                    Some(connection),
                )));
                vertices_to_remove.push_back(vertex.clone());
                temp_node.borrow_mut().insert_vertex(vertex.clone());
            }
        }

        for node in &self.nodes {
            let loc = node.borrow().origin.to_location(&self.origin);
            println!("{}, {}", loc.lat_degree(), loc.lon_degree());
            // let loc = node.borrow().origin;
            // println!("{}, {}", loc.x, loc.y);
            if node.borrow().height > 0f32 {
                let mut current = node.borrow().left_ring.clone();
                loop {
                    let ref mut vertex = current.clone();
                    //print!("{:?}\n", current.borrow().index);
                    let index = match vertex.borrow().next {
                        Some(ref vert) => vert.borrow().index,
                        None => panic!("Next points to null"),
                    };
                    if index != HEADER_VERTEX_INDEX {
                        println!("vertex {}", vertex.borrow().location.to_location(&self.origin));
                    } else {
                        break;
                    }
                    current = match vertex.borrow().next {
                        Some(ref v) => v.clone(),
                        None => panic!("Next points to null"),
                    };
                }
            }
        }

        //A* algorithm - find shortest path from plane to destination
        while let Some(cur) = open_list.pop() {
            // println!("current vertex {}", cur.borrow());
            if cur.borrow().index == END_VERTEX_INDEX {
                Node::remove_extra_vertices(vertices_to_remove);
                return Some(self.generate_waypoint(cur));
            }
            closed_set.insert(cur.borrow().index);

            let mut update_vertex = |cur_g_cost: f32, next: Rc<RefCell<Vertex>>, dist: f32| {
                if next.borrow().index == cur.borrow().index {
                    return;
                }
                let new_g_cost = cur_g_cost + dist;
                // println!("add vertex to queue {}", next.borrow());
                {
                    let mut next_mut = next.borrow_mut();
                    if closed_set.contains(&next_mut.index)                                         //vertex is already explored
                        || next_mut.sentinel                                                        //vertex is a sentinel
                        || (open_set.contains(&next_mut.index) && new_g_cost >= next_mut.g_cost)
                    {
                        //vertex has been visited and the current cost is better
                        // println!("Vertex addition skipped");
                        return;
                    }
                    let new_f_cost = next_mut.g_cost
                        + next_mut
                            .location
                            .distance3d(&Point::from_location(&end, &self.origin));
                    next_mut.g_cost = new_g_cost;
                    next_mut.f_cost = new_f_cost;
                    next_mut.parent = Some(cur.clone());
                }
                open_list.push(next.clone());
                open_set.insert(next.borrow().index);
            };

            let mut cur_vertex = cur.borrow();
            let g_cost = cur_vertex.g_cost;
            if let Some(ref connection) = cur_vertex.connection {
                let mut next = connection.neighbor.clone();
                let dist = connection.distance;
                update_vertex(g_cost, next, dist);
            }

            let mut weight = cur_vertex.get_neighbor_weight();
            if let Some(ref next_vertex) = cur_vertex.next {
                let mut next = next_vertex.clone();
                if (next.borrow().index != HEADER_VERTEX_INDEX) {
                    update_vertex(g_cost, next, weight);
                } else {
                    weight += next.borrow().get_neighbor_weight();
                    match next.borrow().next {
                        Some(ref skip_next) => update_vertex(g_cost, skip_next.clone(), weight),
                        None => panic!("broken chain"),
                    }
                    //update_vertex(g_cost, );
                }
            }
        }
        None
        //TODO: Clean up the graph before we finish
    }

    fn generate_waypoint(&self, end_vertex: Rc<RefCell<Vertex>>) -> LinkedList<Waypoint> {
        let mut waypoint_list = LinkedList::new();
        let mut cur_vertex = end_vertex;
        let mut index = END_VERTEX_INDEX;
        while index != START_VERTEX_INDEX {
            let loc = cur_vertex.borrow().location.to_location(&self.origin);
            let radius = cur_vertex.borrow().radius;
            waypoint_list.push_front(Waypoint::new(1, loc, radius));
            println!("{:?}", cur_vertex);
            let parent = match cur_vertex.borrow().parent {
                Some(ref cur_parent) => cur_parent.clone(),
                None => panic!("Missing a parent without reaching start point"),
            };
            cur_vertex = parent;
            index = cur_vertex.borrow().index;
        }
        waypoint_list
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
