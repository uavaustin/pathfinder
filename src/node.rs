use super::obj::Point;
use super::{std, Pathfinder, RADIUS};
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

#[derive(Clone, Debug)]
pub struct Node {
    pub x: i32,
    pub y: i32,
    pub z: i32, // added a z variable
    pub g_cost: f64,
    pub f_cost: f64,
    pub parent: Option<Rc<Node>>,
    pub depth: i32, 
}

impl Hash for Node {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
	self.z.hash(state); //added, this is saying if two nodes have same x, y, z then they are the same node.
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Node) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z // added the self.z == other.z, this is doing the actual equating logic for same node.
    }
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Node) -> Option<Ordering> {
        other.f_cost.partial_cmp(&self.f_cost)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Node) -> Ordering {
        let ord = self.partial_cmp(other).unwrap();
        match ord {
            Ordering::Greater => Ordering::Less,
            Ordering::Less => Ordering::Greater,
            Ordering::Equal => ord,
        }
    }
}

impl Node {
    pub fn new(x: i32, y: i32, z: i32) -> Self {	// z added
        Node {
            x: x,
            y: y,
	    z: z,	// z added
            g_cost: std::f64::MAX,
            f_cost: std::f64::MAX,
            parent: None,
            depth: 0,
        }
    }
    pub fn to_point(&self, path_finder: &Pathfinder) -> Point {
        // let x = 2f64*RADIUS*(self.lat.cos()*((self.lon-origin.lon)/2f64).sin()).asin();
        let lat = self.y as f64 * path_finder.grid_size as f64 / RADIUS + path_finder.origin.lat();
        let lon = ((self.x as f64 * path_finder.grid_size as f64 / RADIUS / 2f64).sin() / lat.cos())
            .asin() * 2f64 + path_finder.origin.lon();
        Point::from_radians(lat, lon, 0f32)
    }
    pub fn advance(&mut self, x_dir: i32, y_dir: i32) {
        self.x += x_dir;
        self.y += y_dir;
    }
}
