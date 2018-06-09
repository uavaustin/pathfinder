use std::cmp::Ordering;
use std::hash::{Hash, Hasher};

use super::*;

#[derive(Clone, Debug)]
pub struct Node {
    pub x: i32,
    pub y: i32,
    pub g_cost: f32,
    pub f_cost: f32,
    pub parent: Option<Rc<Node>>,
    pub depth: i32,
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Node) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Node) -> Option<Ordering> {
        other.f_cost.partial_cmp(&self.f_cost)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Node) -> Ordering  {
        let ord = self.partial_cmp(other).unwrap();
        match ord {
            Ordering::Greater => Ordering::Less,
            Ordering::Less => Ordering::Greater,
            Ordering::Equal => ord
        }
    }
}

impl Node {
    pub fn new(x:i32, y:i32) -> Node {
        Node {
            x: x,
            y: y,
            g_cost: std::f32::MAX,
            f_cost: std::f32::MAX,
            parent: None,
            depth: 0
        }
    }
    pub fn to_point(&self, path_finder: &Pathfinder) -> Point {
        // let x = 2f64*RADIUS*(self.lat.cos()*((self.lon-origin.lon)/2f64).sin()).asin();
        let lat = self.y as f64 * path_finder.grid_size as f64 / RADIUS + path_finder.origin.lat();
        let lon = ((self.x as f64 * path_finder.grid_size as f64 / RADIUS / 2f64).sin() / lat.cos()).asin() * 2f64
            + path_finder.origin.lon();
        Point::from_radians(lat, lon, 0f32)
    }
    pub fn advance(&mut self, x_dir: i32, y_dir: i32) {
        self.x += x_dir;
        self.y += y_dir;
    }
}
