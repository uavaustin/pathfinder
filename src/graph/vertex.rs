use super::*;

use std::fmt;
use std::hash::Hasher;
use std::hash::Hash;
use std::cmp::Ordering;

impl Vertex {
    pub fn new(node: Rc<RefCell<Node>>, num_vertex: &mut i32, angle: f32, connection: Option<Connection>) -> Vertex {
        *num_vertex += 1;
        Vertex {
            index: *num_vertex,
            radius: node.borrow().radius,
            angle: angle,
            location: Point::from_node_and_angle(&node.borrow(), angle),
            f_cost: -1f32,
            g_cost: -1f32,
            parent: None,
            connection: connection,
            next: None,
            sentinel: false,
        }
    }

    pub fn new_sentinel(angle: f32) -> Vertex {
        Vertex {
            angle: angle,
            connection: None,
            next: None,
            sentinel: true,
        }
    }

    pub fn get_neighbor_weight(&self) -> f32 {
        if let Some(ref neighbor) = self.next {
            let angle = (self.angle - neighbor.borrow().angle).abs(); 
            let radius = self.radius;
            (angle * radius)
        } else {
            0f32
        }
    }
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
        if self.f_cost < other.f_cost {
            Ordering::Less
        } else if self.f_cost > other.f_cost {
            Ordering::Greater
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
            "(angle={}, connection={} next={})",
            self.angle,
            self.connection.is_some(),
            self.next.is_some()
        )
    }
}
