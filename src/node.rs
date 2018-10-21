use super::{std, TURNING_RADIUS, ordered_float::OrderedFloat};
use obj::{Obstacle, Plane, Point, Waypoint};

use std::hash::{Hash, Hasher};
use std::rc::Rc;

#[derive(Clone, Eq, Hash, PartialEq)]
pub enum Direction {
    Left,
    Right
}

#[derive(Clone, Eq, Hash, PartialEq)]
pub struct Port {
    reference: Rc<Node>,
    angle: OrderedFloat<f32>,
    side: Direction
}

impl Port {
    pub fn new(reference: Rc<Node>, angle: OrderedFloat<f32>, side: Direction) -> Port{
        Port {
            reference: reference.clone(),
            angle: angle,
            side: side
        }
    }

    pub fn reciprocal(&self) -> Self {
        Port {
            reference: self.reference.clone(),
            angle: self.angle,
            side: if self.side == Direction::Right {Direction::Left} else {Direction::Right}
        }
    }
}

// Represent a vertex that's half of a connection between two nodes
// Contains the coordinate of tangent line and distance
#[derive(Clone, Eq, Hash, PartialEq)]
pub struct Vertex {
    start: Port,
    end: Port,
    distance: OrderedFloat<f32>,
}

impl Vertex {
    pub fn new(start: Port, end: Port, distance: f32) -> Self {
        Vertex {
            start: start,
            end: end,
            distance: distance.into(),
        }
    }

    pub fn set_path(&mut self, start: Port, end: Port) {
        self.start = start;
        self.end = end;
    }

    pub fn reciprocal(&self) -> Self {
        Vertex {
            start: self.end.reciprocal(),
            end: self.start.reciprocal(),
            distance: self.distance,
        }
    }
}

#[derive(Clone)]
pub struct Node {
    index: u32,
    location: Point,
    radius: f32,
    height: f32,
}

impl Eq for Node {}

impl PartialEq for Node {
    fn eq(&self, other: &Node) -> bool {
        self.index == other.index
    }
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.index.hash(state);
    }
}

impl<'a> From<&'a Obstacle> for Node {
    fn from(obs: &Obstacle) -> Node {
        Node::new(0, obs.coords, obs.radius, obs.height)
    }
}

impl<'a> From<&'a Point> for Node {
    fn from(p: &Point) -> Node {
        Node::new(0, *p, 10f32, std::f32::MAX)
    }
}

impl<'a> From<&'a Plane> for Node {
    fn from(plane: &Plane) -> Node {
        Node::new(0, plane.location, TURNING_RADIUS, plane.location.alt())
    }
}

impl<'a> From<&'a Waypoint> for Node {
    fn from(waypoint: &Waypoint) -> Node {
        Node::new(0, waypoint.location, waypoint.radius, 0f32)
    }
}

impl Node {
    fn new(index: u32, location: Point, radius: f32, height: f32) -> Self {
        Node {
            index: index,
            location: location,
            radius: radius,
            height: height,
        }
    }
    pub fn set_index(&mut self, index: u32) {
        self.index = index;
    }
}
