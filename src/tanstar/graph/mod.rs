// Visibility related code for modularity
use super::*;

mod flyzones;
#[cfg(test)]
mod test;

pub mod connection;
pub mod node;
pub mod point;
pub mod util;
pub mod vertex;

pub use self::connection::Connection;
pub use self::node::Node;
pub use self::point::Point;
pub use self::util::*;
pub use self::vertex::Vertex;

use obj::{Location, Obstacle};

/// An enum describing the validity of the path in reference to obstacles, flyzones, and the ground
// #TODO: Ensure that the ground is considered
pub enum PathValidity {
    /// The path does not intersect with anything the plane can't fly through
    Valid,
    /// The path intersects with an obstacle, the ground, or goes outside the flyzone
    Invalid,
    /// The path is okay to traverse at the given height
    ///
    /// From the top-down 2D perspective the path intersects with 1 or more obstacles, but the
    /// obstacles all have a height less than or equal to the given height.
    Flyover(f32),
}

impl From<PathValidity> for bool {
    /// Converts the validity into a boolean
    ///
    /// The path is only intraversable if it is 'Invalid'.
    fn from(pv: PathValidity) -> bool {
        match pv {
            PathValidity::Invalid => false,
            _ => true,
        }
    }
}

/// A ['Vec'] of path segments: (i, j, distance, threshold)
///
/// i: counter-clockwise deflection angle from the positive x-axis around the first obstacle that a segment is tangent to
/// j: counter-clockwise deflection from the positive x-axis around the second obstacle that a segment is tangent to
/// distance: distance between the 2 points coincident to the segment and the obstacles
/// threshold: minimum height the segment must be flown at to be valid
type Path = Vec<(f32, f32, f32, f32)>;
/// A ['Vec'] of obstacle intersections
///
/// The values are the deflection angles from the displacement vector (from the first obstacle
/// to the second) on the first obstacle and on the second obstacle.
/// I.e. for a tuple (i, j):
/// i would be the counter-clockwise deflection angle of the point relative to the first obstacle
/// j would be the counter-clockwise deflection angle of the point relative to the second obstacle (same displacement vector)
type Sentinel = Vec<(f32, f32)>;

impl Tanstar {
    fn insert_edge(
        &mut self,
        i: usize,
        j: usize,
        (alpha, beta, distance, threshold): (f32, f32, f32, f32),
    ) {
        // Insert edge from u -> v
        let v = self.nodes[j].borrow().get_vertex(
            &mut self.num_vertices,
            beta,
            self.config.vertex_merge_threshold,
        );
        let edge = Connection::new(v.clone(), distance, threshold);
        let u = self.nodes[i].borrow().get_vertex(
            &mut self.num_vertices,
            alpha,
            self.config.vertex_merge_threshold,
        );
        u.borrow_mut().connection.push(edge);
    }

    pub fn build_graph(&mut self) {
        self.populate_nodes();
        for i in 0..self.nodes.len() {
            let node = self.nodes[i].clone();
            self.insert_flyzone_sentinel(&mut node.borrow_mut());

            for j in i + 1..self.nodes.len() {
                let (paths, obs_sentinels) =
                    self.find_path(&self.nodes[i].borrow(), &self.nodes[j].borrow());
                println!("[{} {}]: path count -> {}", i, j, paths.len());

                // Inserting edge
                for mut path in paths {
                    // Edge from i to j
                    self.insert_edge(i, j, path);
                    // Reciprocal edge from j to i
                    let (beta, alpha) = (reverse_polarity(path.0), reverse_polarity(path.1));
                    path.0 = alpha;
                    path.1 = beta;
                    self.insert_edge(j, i, path);
                }

                // Inserting sentinels
                if obs_sentinels.is_some() {
                    println!("inserting sentinels");
                    for (alpha_s, beta_s) in obs_sentinels.unwrap() {
                        let mut a = Vertex::new_sentinel(
                            &mut self.num_vertices,
                            &self.nodes[i].borrow(),
                            alpha_s,
                        );
                        //a.sentinel = true;
                        let mut b = Vertex::new_sentinel(
                            &mut self.num_vertices,
                            &self.nodes[j].borrow(),
                            beta_s,
                        );
                        //b.;
                        let s_a = Rc::new(RefCell::new(a));
                        let s_b = Rc::new(RefCell::new(b));
                        self.nodes[i].borrow_mut().insert_vertex(s_a);
                        self.nodes[j].borrow_mut().insert_vertex(s_b);
                    }
                }
            }
        }

        // output_graph(&self);
    }

    /// Sets up 'Tanstar' for pathfinding
    ///
    /// Clears 'nodes' and fills it with newly created ['Node']s from 'obstacles'. If it should it
    /// also virtualizes the flyzones.
    fn populate_nodes(&mut self) {
        self.nodes.clear();
        self.origin = Self::find_origin(&self.flyzones);
        for i in 0..self.obstacles.len() {
            let mut node = (&self.obstacles[i], &self.origin, self.config.buffer_size).into();
            self.nodes.push(Rc::new(RefCell::new(node)));
        }
        if self.config.virtualize_flyzone {
            for i in 0..self.flyzones.len() {
                self.virtualize_flyzone(i);
            }
        }
    }

    /// Finds the shortest valid path with 'Sentinel's at obstacle intersections, if any;
    /// returns an empty vector for 'Path' if none exists
    ///
    /// Generates all possible path segments (tangent lines) between two ['Node']s and finds
    /// the shortest valid one.
    pub fn find_path(&self, node1: &Node, node2: &Node) -> (Path, Option<Sentinel>) {
        let pos1: Point = node1.origin;
        let pos2: Point = node2.origin;
        let rad1: f32 = node1.radius;
        let rad2: f32 = node2.radius;
        let dist: f32 = pos1.distance(&pos2);

        // theta1 and theta2 represents the normalize angle
        // normalized between 0 and 2pi
        let theta = (pos2.y - pos1.y).atan2(pos2.x - pos1.x);
        let (theta1, theta2) = if theta > 0f32 {
            (theta, theta + PI)
        } else {
            (theta + 2f32 * PI, theta + PI)
        };

        println!(
            "x1:{}, y1:{}, r1:{}, x2:{}, y2:{}, r2:{}",
            pos1.x, pos1.y, rad1, pos2.x, pos2.y, rad2
        );

        println!(
            "theta: {}, theta1: {}, theta2: {}",
            theta.to_degrees(),
            theta1.to_degrees(),
            theta2.to_degrees()
        );

        // gamma1 and gamma2 are the angle between reference axis and the tangents
        // gamma1 is angle to inner tangent, gamma2 is angle to outer tangent
        let gamma1 = ((rad1 + rad2).abs() / dist).acos();
        let mut gamma2 = ((rad1 - rad2).abs() / dist).acos();

        // we assume rad1 is greater than rad2 for the math to work, so find complement if otherwise
        if rad2 > rad1 {
            gamma2 = PI - gamma2;
        }

        println!(
            "gamma1: {}, gamma2: {}",
            gamma1.to_degrees(),
            gamma2.to_degrees()
        );

        // Outer tangent always exists
        let mut candidates = vec![
            (
                normalize_angle(true, theta1 - gamma2),
                normalize_angle(true, theta2 + PI - gamma2),
            ),
            (
                normalize_angle(false, theta1 - 2f32 * PI + gamma2),
                normalize_angle(false, theta2 - 3f32 * PI + gamma2),
            ),
        ];

        let mut sentinels = None;
        if rad1 != 0f32 && rad2 != 0f32 && dist > rad1 + rad2 {
            candidates.append(&mut vec![
                // Inner left tangent
                (
                    normalize_angle(true, theta1 - gamma1),
                    normalize_angle(false, theta2 - 2f32 * PI - gamma1),
                ),
                // Inner right tangent
                (
                    normalize_angle(false, theta1 - 2f32 * PI + gamma1),
                    normalize_angle(true, theta2 + gamma1),
                ),
            ]);
        } else {
            println!("obstacle sentinels detected");
            //determine angle locations of sentinels
            let theta_s = ((rad1.powi(2) + dist.powi(2) - rad2.powi(2)) / (2f32 * rad1 * dist)).acos();
            let phi_s = ((rad2.powi(2) + dist.powi(2) - rad1.powi(2)) / (2f32 * rad2 * dist)).acos();

            //sentinel vertices on node1
            let node1_s1 = theta_s;
            let node1_s2 = -theta_s;
            let node1_s3 = -2f32 * PI + theta_s;
            let node1_s4 = 2f32 * PI - theta_s;
            //sentinel vertices on node2
            let node2_s1 = PI - phi_s;
            let node2_s2 = PI + phi_s;
            let node2_s3 = -PI + phi_s;
            let node2_s4 = -PI - phi_s;
            sentinels = Some(vec![(node1_s1, node2_s1), (node1_s2, node2_s2), (node1_s3, node2_s3), (node1_s4, node2_s4)]);
        }

        let mut connections = Vec::new();
        let mut point_connections = Vec::new();
        for (i, j) in candidates {
            let p1 = Point::from((node1, i));
            let p2 = Point::from((node2, j));
            println!("angles {} -> {}", i.to_degrees(), j.to_degrees());
            println!("validating path {:?} -> {:?}", p1, p2);

            match self.valid_path(&p1, &p2) {
                PathValidity::Valid => {
                    println!("This path is Valid without Flyover.");
                    connections.push((i, j, p1.distance(&p2), 0f32));
                    point_connections.push((p1, p2));
                }
                PathValidity::Flyover(height_min) => {
                    println!("This path is Valid with Flyover.");
                    connections.push((i, j, p1.distance(&p2), height_min));
                    point_connections.push((p1, p2));
                }
                _ => {
                    println!("This Path is Invalid.");
                }
            }
        }

        (connections, sentinels)
    }

    // check if a path is valid (not blocked by flightzone or obstacles)
    fn valid_path(&self, a: &Point, b: &Point) -> PathValidity {
        // let theta_o = (b.z - a.z).atan2(a.distance(b));
        // //check if angle of waypoints is valid
        // if theta_o > MAX_ANGLE_ASCENT {
        //     return PathValidity::Invalid;
        // }

        println!("validating path: {:?}, {:?}", a, b);
        // latitude is y, longitude is x
        // flyzone is array connected by each index
        // some messy code to link flyzone points, can definitely be better
        for flyzone in &self.flyzones {
            let mut tempzone = flyzone.clone();
            let first = Point::from((&tempzone.remove(0), &self.origin));
            let mut temp = first;
            for location in tempzone {
                //println!("origin: {:?}", &self.origin);
                let point = Point::from((&location, &self.origin));
                //println!("test intersect for {:?} {:?} {:?} {:?}", a, b, &temp, &point);
                if intersect(a, b, &temp, &point) {
                    println!("false due to flyzone");
                    return PathValidity::Invalid;
                }
                temp = point;
            }
            //println!("test intersect for {:?} {:?} {:?} {:?}", a, b, &temp, &first);
            if intersect(a, b, &temp, &first) {
                println!("false due to flyzone");
                return PathValidity::Invalid;
            }
        }

        // test for obstacles
        let mut max_height = 0f32;
        for obstacle in &self.obstacles {
            // catch the simple cases for now: if a or b are inside the radius of obstacle, invalid
            // check if there are two points of intersect, for flyover cases
            if let (Some(_p1), Some(_p2)) = perpendicular_intersect(&self.origin, a, b, obstacle) {
                println!(
                    "found intersection at height {} with obstacle {:?}",
                    obstacle.height, obstacle
                );
                if obstacle.height > max_height {
                    max_height = obstacle.height;
                }
                // return PathValidity::Invalid; // Temporarily disable fly over
            }
        }
        println!("path valid with threshold {}", max_height);
        PathValidity::Flyover(max_height)
    }
}
