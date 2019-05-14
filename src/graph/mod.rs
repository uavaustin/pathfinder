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

pub enum PathValidity {
    Valid,
    Invalid,
    Flyover(f32),
}

impl From<PathValidity> for bool {
    fn from(pv: PathValidity) -> bool {
        match pv {
            PathValidity::Invalid => false,
            _ => true,
        }
    }
}

impl Pathfinder {
    fn insert_edge(
        &mut self,
        i: usize,
        j: usize,
        (alpha, beta, distance, threshold): (f32, f32, f32, f32),
    ) {
        // Insert edge from u -> v
        let v = self.nodes[j]
            .borrow()
            .get_vertex(&mut self.num_vertices, beta);
        let edge = Connection::new(v.clone(), distance, threshold);
        let u = self.nodes[i]
            .borrow()
            .get_vertex(&mut self.num_vertices, alpha);
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

    fn populate_nodes(&mut self) {
        self.nodes.clear();
        self.find_origin();
        for i in 0..self.obstacles.len() {
            let mut node = (&self.obstacles[i], &self.origin, self.buffer).into();
            self.nodes.push(Rc::new(RefCell::new(node)));
        }
        for i in 0..self.flyzones.len() {
            self.virtualize_flyzone(i);
        }
    }

    fn find_origin(&mut self) {
        const MAX_RADIAN: f64 = 2f64 * ::std::f64::consts::PI;
        let mut min_lat = MAX_RADIAN;
        let mut min_lon = MAX_RADIAN;
        let mut max_lon = 0f64;
        let mut lon = min_lon;

        assert!(self.flyzones.len() > 0, "Require at least one flyzone");
        for i in 0..self.flyzones.len() {
            let flyzone_points = &self.flyzones[i];
            assert!(
                flyzone_points.len() > 2,
                "Require at least 3 points to construct fly zone."
            );

            for point in flyzone_points {
                if point.lat() < min_lat {
                    min_lat = point.lat();
                }
                if point.lon() < min_lon {
                    min_lon = point.lon();
                }
                if point.lon() > max_lon {
                    max_lon = point.lon();
                }
            }
            lon = min_lon;
            if max_lon - min_lon > MAX_RADIAN {
                lon = max_lon;
            }
        }

        self.origin = Location::from_radians(min_lat, lon, 0f32);
        println!(
            "Found origin: {}, {}",
            self.origin.lat_degree(),
            self.origin.lon_degree()
        );
    }

    // Generate all valid possible path (tangent lines) between two nodes, and return the
    // shortest valid path if one exists

    // returns: (i, j, distance, threshold), (a_sentinels, b_sentinels)
    pub fn find_path(
        &self,
        a: &Node,
        b: &Node,
    ) -> (Vec<(f32, f32, f32, f32)>, Option<Vec<(f32, f32)>>) {
        let c1: Point = a.origin;
        let c2: Point = b.origin;
        let r1: f32 = a.radius;
        let r2: f32 = b.radius;
        let dist: f32 = c1.distance(&c2);

        // theta1 and theta2 represents the normalize angle
        // normalized between 0 and 2pi
        let theta = (c2.y - c1.y).atan2(c2.x - c1.x);
        let (theta1, theta2) = if theta > 0f32 {
            (theta, theta + PI)
        } else {
            (theta + 2f32 * PI, theta + PI)
        };

        println!(
            "x1:{}, y1:{}, r1:{}, x2:{}, y2:{}, r2:{}",
            c1.x, c1.y, r1, c2.x, c2.y, r2
        );

        println!(
            "theta: {}, theta1: {}, theta2: {}",
            theta.to_degrees(),
            theta1.to_degrees(),
            theta2.to_degrees()
        );

        // gamma1 and gamma2 are the angle between reference axis and the tangents
        // gamma1 is angle to inner tangent, gamma2 is angle to outer tangent
        let gamma1 = ((r1 + r2).abs() / dist).acos();
        let mut gamma2 = ((r1 - r2).abs() / dist).acos();

        // we assume r1 is greater than r2 for the math to work, so find complement if otherwise
        if r2 > r1 {
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
        if r1 != 0f32 && r2 != 0f32 && dist > r1 + r2 {
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
            let theta_s = ((r1.powi(2) + dist.powi(2) - r2.powi(2)) / (2f32 * r1 * dist)).acos();
            let phi_s = ((r2.powi(2) + dist.powi(2) - r1.powi(2)) / (2f32 * r2 * dist)).acos();

            //sentinel vertices on A
            let a_s1 = theta_s;
            let a_s2 = -theta_s;
            let a_s3 = -2f32 * PI + theta_s;
            let a_s4 = 2f32 * PI - theta_s;
            //sentinel vertices on B
            let b_s1 = PI - phi_s;
            let b_s2 = PI + phi_s;
            let b_s3 = -PI + phi_s;
            let b_s4 = -PI - phi_s;
            sentinels = Some(vec![(a_s1, b_s1), (a_s2, b_s2), (a_s3, b_s3), (a_s4, b_s4)]);
        }

        let mut connections = Vec::new();
        let mut point_connections = Vec::new();
        for (i, j) in candidates {
            let p1 = Point::from((a, i));
            let p2 = Point::from((b, j));
            println!("angles {} -> {}", i.to_degrees(), j.to_degrees());
            println!("validating path {:?} -> {:?}", p1, p2);

            match self.valid_path(&p1, &p2) {
                PathValidity::Valid => {
                    println!("This path is Valid without Flyover.");
                    connections.push((i, j, p1.distance(&p2), 0f32));
                    point_connections.push((p1, p2));
                }
                PathValidity::Flyover(h_min) => {
                    println!("This path is Valid with Flyover.");
                    connections.push((i, j, p1.distance(&p2), h_min));
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
