use super::*;

use std::cmp::Ordering;
use std::fmt;
use std::hash::Hash;
use std::hash::Hasher;

impl Vertex {
    pub fn new(
        num_vertex: &mut i32,
        node: &Node,
        angle: f32,
        connection: Vec<Connection>,
    ) -> Vertex {
        Vertex::base_vertex(
            num_vertex,
            node.radius,
            angle,
            Point::from_reference(node, angle),
            connection,
            false,
        )
    }

    pub fn new_sentinel(num_vertex: &mut i32, node: &Node, angle: f32) -> Vertex {
        Vertex::base_vertex(
            num_vertex,
            node.radius,
            angle,
            Point::from_reference(node, angle),
            vec![],
            true,
        )
    }

    pub fn new_head(num_vertex: &mut i32, origin: Point) -> Vertex {
        Vertex::base_vertex(num_vertex, 0f32, 0f32, origin, vec![], false)
    }

    fn base_vertex(
        num_vertex: &mut i32,
        radius: f32,
        angle: f32,
        location: Point,
        connection: Vec<Connection>,
        sentinel: bool,
    ) -> Vertex {
        if *num_vertex >= 0 {
            *num_vertex += 1;
        }
        Vertex {
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
