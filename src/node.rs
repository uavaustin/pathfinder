use super::{std, TURNING_RADIUS, ordered_float::OrderedFloat, PI};
use obj::{Obstacle, Plane, Point, Waypoint};

use std::hash::{Hash, Hasher};
use std::rc::Rc;

#[derive(Clone, Eq, Hash, PartialEq)]
pub struct Vertex {
    pub reference: Rc<Node>,
    pub angle: OrderedFloat<f32>
}

impl Vertex {
    pub fn new(reference: Rc<Node>, angle: f32) -> Vertex{
        Vertex {
            reference: reference.clone(),
            angle: OrderedFloat(angle)
        }
    }

    pub fn to_point(&self) -> Point {
        let a1: f32 = self.angle.into();
        Point::from_radians(self.reference.location.lat() + (self.reference.radius * a1.sin()) as f64,
                self.reference.location.lon() + (self.reference.radius * a1.cos()) as f64,
                0f32)
    }

    pub fn reciprocal(&self) -> Self {
        let theta: f32 = self.angle.into();
        Vertex {
            reference: self.reference.clone(),
            angle: OrderedFloat((2f32 * PI - theta) % (2f32 * PI)),
        }
    }

}

// Represent a connection between two nodes
// Contains the coordinate of tangent line and distance
#[derive(Clone, Eq, Hash, PartialEq)]
pub struct Connection {
    pub start: Rc<Vertex>,
    pub end: Rc<Vertex>,
    distance: OrderedFloat<f32>,
}

impl Connection {
    pub fn new(start: Rc<Vertex>, end: Rc<Vertex>, distance: f32) -> Self {
        Connection {
            start: start,
            end: end,
            distance: distance.into(),
        }
    }

    pub fn reciprocal(&self) -> Self {
        Connection {
            start: Rc::new(self.end.reciprocal()),
            end: Rc::new(self.start.reciprocal()),
            distance: self.distance,
        }
    }
}

#[derive(Clone)]
pub struct Node {
    index: u32,
    pub location: Point,
    pub radius: f32,
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
