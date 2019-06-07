use super::*;

use std::cmp::Ordering;
use std::fmt;
use std::hash::Hash;
use std::hash::Hasher;

#[derive(Debug)]
pub struct Vertex {
    pub index: i32,                           // Index to identify vertex
    pub radius: f32,                          // Radius of the node vertex is attached to
    pub location: Point,                      // Location of the vertex
    pub angle: f32,                           // Angle with respect to the node
    pub g_cost: f32,                          //
    pub f_cost: f32,                          //
    pub parent: Option<Arc<RefCell<Vertex>>>, // Parent of vertex
    pub connection: Vec<Connection>,          // Edge connecting to another node
    pub prev: Option<Arc<RefCell<Vertex>>>,   // Previous neighbor vertex in the same node
    pub next: Option<Arc<RefCell<Vertex>>>,   // Neighbor vertex in the same node
    pub sentinel: bool,                       // Sentinel property marks end of path hugging
}

impl Hash for Vertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.index.hash(state);
    }
}

impl Eq for Vertex {}

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

impl Ord for Vertex {
    fn cmp(&self, other: &Self) -> Ordering {
        // Order flipped for max heap
        if self.f_cost < other.f_cost {
            Ordering::Greater
        } else if self.f_cost > other.f_cost {
            Ordering::Less
        } else {
            Ordering::Equal
        }
    }
}

impl PartialOrd for Vertex {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl fmt::Display for Vertex {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "(index={}, angle={}, connection={} next={} g_cost={} f_cost={})",
            self.index,
            self.angle,
            self.connection.len(),
            self.next.is_some(),
            self.g_cost,
            self.f_cost,
        )
    }
}

impl Vertex {
    pub fn new(num_vertex: &mut i32, node: &Node, angle: f32, connection: Vec<Connection>) -> Self {
        Self::base_vertex(
            num_vertex,
            node.radius,
            angle,
            Point::from((node, angle)),
            connection,
            false,
        )
    }

    pub fn new_sentinel(num_vertex: &mut i32, node: &Node, angle: f32) -> Self {
        Self::base_vertex(
            num_vertex,
            node.radius,
            angle,
            Point::from((node, angle)),
            vec![],
            true,
        )
    }

    pub fn new_head(num_vertex: &mut i32, origin: Point) -> Self {
        Self::base_vertex(num_vertex, 0f32, 0f32, origin, vec![], false)
    }

    fn base_vertex(
        num_vertex: &mut i32,
        radius: f32,
        angle: f32,
        location: Point,
        connection: Vec<Connection>,
        sentinel: bool,
    ) -> Self {
        if *num_vertex >= 0 {
            *num_vertex += 1;
        }

        Self {
            index: *num_vertex,
            radius: radius,
            angle: angle,
            location: location,
            f_cost: -1f32,
            g_cost: -1f32,
            parent: None,
            connection: connection,
            prev: None,
            next: None,
            sentinel: sentinel,
        }
    }

    pub fn get_neighbor_weight(&self) -> f32 {
        if let Some(ref neighbor) = self.next {
            return arc_length(self.angle, neighbor.borrow().angle, self.radius);
        } else {
            panic!("broken chain");
        }
    }
}
