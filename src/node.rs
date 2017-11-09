use std::cmp::Ordering;
use std::hash::{Hash, Hasher};

use super::*;

#[derive(Clone, Copy, Debug)]
pub struct Node {
    pub x: i32,
    pub y: i32,
    pub g_cost: f32,
    pub f_cost: f32
}

impl PartialEq for Node {
    fn eq(&self, other: &Node) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Eq for Node {}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Node) -> Option<Ordering> {
        other.f_cost.partial_cmp(&self.f_cost)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Node) -> Ordering  {
        self.partial_cmp(other).unwrap()
    }
}

impl Node {
    pub fn new(x:i32, y:i32) -> Node {
        Node {
            x: x,
            y: y,
            g_cost: std::f32::MAX,
            f_cost: std::f32::MAX,
        }
    }
    pub fn to_point(&self, path_finder: &PathFinder) -> Point {
        let lat = self.y as f32 / RADIUS + path_finder.origin.lat;
        let lon = ((self.x as f32 / RADIUS / 2.0).powi(2).sin() /
         path_finder.origin.lat.cos() / lat.cos()).sqrt().asin() * 2.0 + path_finder.origin.lon;
        Point::from_radians(lat, lon)
    }
}
