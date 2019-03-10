// Visibility related code for modularity
use super::*;

#[cfg(test)]
mod test;

mod connection;
mod node;
mod point;
pub mod util;
mod vertex;

pub use graph::util::*;
use obj::{Location, Obstacle};

#[derive(Copy, Clone, Debug)]
pub struct Point {
    pub x: f32, // horizontal distance from origin in meters
    pub y: f32, // vertical distance from origin in meters
    pub z: f32,
}

#[derive(Debug)]
pub struct Vertex {
    pub index: i32,                          // Index to identify vertex
    pub radius: f32,                         // Radius of the node vertex is attached to
    pub location: Point,                     // Location of the vertex
    pub angle: f32,                          // Angle with respect to the node
    pub g_cost: f32,                         //
    pub f_cost: f32,                         //
    pub parent: Option<Rc<RefCell<Vertex>>>, //
    pub connection: Option<Connection>,      // Edge connecting to another node
    pub next: Option<Rc<RefCell<Vertex>>>,   // Neighbor vertex in the same node
    pub sentinel: bool,                      // Sentinel property marks end of path hugging
}

// Represent a connection between two nodes
// Contains the coordinate of tangent line and distance
#[derive(Debug)]
pub struct Connection {
    pub neighbor: Rc<RefCell<Vertex>>, // Connected node through a tangent
    pub distance: f32,
    // starting and ending vertices must be above threshold to take the connection
    pub threshold: f32,
}

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

#[derive(Debug)]
pub struct Node {
    pub origin: Point,
    pub radius: f32,
    height: f32,
    left_ring: Rc<RefCell<Vertex>>,
    right_ring: Rc<RefCell<Vertex>>,
}

impl Pathfinder {
    pub fn build_graph(&mut self) {
        self.populate_nodes();
        for i in 0..self.nodes.len() {
            for j in i + 1..self.nodes.len() {
                let (paths, obs_sentinels) =
                    self.find_path(&self.nodes[i].borrow(), &self.nodes[j].borrow());
                for (alpha, beta, distance, threshold) in paths {
                    // println!("path: {} {} {}", alpha, beta, distance);
                    let v = Rc::new(RefCell::new(Vertex::new(
                        self.nodes[i].clone(),
                        &mut self.num_vertices,
                        beta,
                        None,
                    )));
                    let edge = Connection::new(v, distance, threshold);
                    let u = Rc::new(RefCell::new(Vertex::new(
                        self.nodes[j].clone(),
                        &mut self.num_vertices,
                        alpha,
                        Some(edge),
                    )));
                    self.nodes[i].borrow_mut().insert_vertex(u);
                    // reciprocal
                    let v = Rc::new(RefCell::new(Vertex::new(
                        self.nodes[i].clone(),
                        &mut self.num_vertices,
                        (2f32 * PI - alpha) % (2f32 * PI),
                        None,
                    )));
                    let edge = Connection::new(v, distance, threshold);
                    let u = Rc::new(RefCell::new(Vertex::new(
                        self.nodes[j].clone(),
                        &mut self.num_vertices,
                        (2f32 * PI - beta) % (2f32 * PI),
                        Some(edge),
                    )));
                    self.nodes[j].borrow_mut().insert_vertex(u);
                }
                if obs_sentinels.is_some() {
                    for (alpha_s, beta_s) in obs_sentinels.unwrap() {
                        let mut a = Vertex::new_sentinel(
                            &mut self.num_vertices,
                            self.nodes[i].borrow().origin.clone(),
                            alpha_s,
                        );
                        //a.sentinel = true;
                        let mut b = Vertex::new_sentinel(
                            &mut self.num_vertices,
                            self.nodes[j].borrow().origin.clone(),
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

        // for i in 0..self.nodes.len() {
        //     println!("Node {}: {}\n", i, self.nodes[i].borrow());
        // }
    }

    fn populate_nodes(&mut self) {
        self.nodes.clear();
        self.find_origin();
        for obs in &self.obstacles {
            let mut node = Node::from_obstacle(obs, &self.origin);
            self.nodes.push(Rc::new(RefCell::new(node)));
        }
        for i in 0..self.flyzones.len() {
            self.virtualize_flyzone(i);
        }
    }

    pub fn virtualize_flyzone(&mut self, index: usize) {
        let flyzone = &self.flyzones[index];
        // convert flyzone to points
        let mut flyzone_points = Vec::new();
        for location in flyzone {
            let point = Point::from_location(&location, &self.origin);
            flyzone_points.push(point);
        }
        // determine flyzone directions
        let size = flyzone.len() as isize - 1;
        let (clockwise, _) = vertex_direction(&flyzone_points);
        let (direction, mut iter): (isize, isize) = if clockwise == true {
            (1, 0)
        } else {
            (-1, size)
        };
        // edge conditions for flyzone
        while (iter <= size) && (iter > -1) {
            let (prev, next) = if iter == 0 {
                (size, iter + 1)
            } else if iter == size {
                (iter - 1, 0)
            } else {
                (iter - 1, iter + 1)
            };
            println!("prev iter next: {} {} {}", prev, iter, next);
            // initalize obstacle location
            let a = flyzone_points[prev as usize];
            let vertex = flyzone_points[iter as usize];
            let b = flyzone_points[next as usize];
            println!("{:?} {:?} {:?}", a, vertex, b);
            let vec_a = (a.x - vertex.x, a.y - vertex.y);
            let vec_b = (b.x - vertex.x, b.y - vertex.y);
            let mag_a = ((vec_a.0).powi(2) + (vec_a.1).powi(2)).sqrt();
            let mag_b = ((vec_b.0).powi(2) + (vec_b.1).powi(2)).sqrt();
            let bisect = (
                mag_b * vec_a.0 + mag_a * vec_b.0,
                mag_b * vec_a.1 + mag_a * vec_b.1,
            );
            let mag_bisection = ((bisect.0).powi(2) + (bisect.1).powi(2)).powf(0.5);
            let bisection = (bisect.0 / mag_bisection, bisect.1 / mag_bisection);
            let theta = ((vec_a.0 * vec_b.0 + vec_a.1 * vec_b.1) / (mag_a * mag_b)).acos();
            // section direction
            let (iter_clockwise, straight) = vertex_direction(&vec![a, vertex, b]);
            // straight line condition
            if straight == true {
                println!("Straight!");
            } else {
                let d = if (iter_clockwise == false && direction == 1)
                    || (iter_clockwise == true && direction == -1)
                {
                    TURNING_RADIUS
                } else {
                    TURNING_RADIUS / ((theta / 2f32).sin())
                };

                if d > mag_a || d > mag_b {
                    println!("small angle");
                } else {
                    // normal angle node
                    let dis = d;
                    let center = Point::new(
                        dis * bisection.0 + vertex.x,
                        dis * bisection.1 + vertex.y,
                        0f32,
                    );
                    println!("center: {:?}", center);
                    let virt_ob = Node::new(center, TURNING_RADIUS, 0f32);
                    self.nodes.push(Rc::new(RefCell::new(virt_ob)));
                }
            }
            iter += direction;
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

        self.origin = Location::from_radians(lon, min_lat, 0f32);
    }

    // determines vertices of node and flyzone intersection
    fn sentinel_normal(&mut self, node: &mut Node) -> () {
        let center: Point = node.origin;
        let r: f32 = node.radius;
        for flyzone in self.flyzones.iter() {
            let size = flyzone.len() + 1;
            // iterate node over all vertices
            for i in 0..size {
                let mut v1 = flyzone[i];
                let mut v2 = flyzone[(i + 1) % size];
                let (x, y, dist, end) = intersect_distance(
                    &Point::from_location(&v1, &self.origin),
                    &Point::from_location(&v2, &self.origin),
                    &center,
                );
                // check intersect is true
                if dist > r {
                    continue;
                }
                // determine both intersect angles in left and right ring
                let theta = (dist / r).acos();
                let dx = x - center.x;
                let dy = y - center.y;
                let phi = dy.atan2(dx);
                let (left, right) = if dy > 0f32 {
                    (phi, -2f32 * PI + phi)
                } else {
                    (2f32 * PI + phi, phi)
                };
                let angles = vec![(left, theta), (right, theta)];
                // create and impliment vertices
                for (i, j) in angles {
                    let a = i + j;
                    let b = i - j;
                    let vertex_a = Rc::new(RefCell::new(Vertex::new_sentinel(
                        &mut self.num_vertices,
                        node.origin,
                        a,
                    )));
                    let vertex_b = Rc::new(RefCell::new(Vertex::new_sentinel(
                        &mut self.num_vertices,
                        node.origin,
                        b,
                    )));
                    node.insert_vertex(vertex_a);
                    node.insert_vertex(vertex_b);
                }
            }
        }
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
        println!(
            "finding path between {:?} and {:?} w/ distance {}",
            c1, c2, dist
        );

        let theta1 = ((r2 - r1).abs() / dist).acos();
        let theta2 = -theta1;
        let phi1 = theta1;
        let phi2 = -phi1;
        let theta3 = ((r1 + r2) / dist).acos();
        let theta4 = -theta3;
        let phi3 = -PI + theta3;
        let phi4 = -phi3;
        let candidates;
        let mut sentinels = None;
        if r1 != 0f32 && r2 != 0f32 && dist > r1 + r2 {
            candidates = vec![
                (theta1, phi1),
                (theta2, phi2),
                (theta3, phi3),
                (theta4, phi4),
            ];
        } else {
            candidates = vec![(theta1, phi1), (theta2, phi2)];
            //determine angle locations of sentinels
            let theta_s = ((r1.powi(2) + dist.powi(2) - r2.powi(2)) / (2f32 * r1 * dist)).acos();
            let phi_s = ((r2.powi(2) + dist.powi(2) - r1.powi(2)) / (2f32 * r2 * dist)).acos();
            println!(
                "Generating Sentinels: Theta = {:?}, Phi = {:?}",
                theta_s, phi_s
            );
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
            println!("{:?}", sentinels)
        }

        let mut connections = Vec::new();
        let mut point_connections = Vec::new();
        for (i, j) in candidates.iter() {
            //          let p1 = a.to_point(*j + theta0);
            //			println!("{} {:?}", j, &p1);
            //          let p2 = b.to_point(*i + theta0);
            //			println!("{} {:?}", i, &p2);
            //          match self.valid_path(&p1, &p2) {
            let p1;
            let p2;
            if c1.x > c2.x || c1.y > c2.y {
                p1 = a.to_point(PI - *i);
                p2 = b.to_point(PI - *j);
            } else {
                p1 = a.to_point(*i);
                p2 = b.to_point(*j);
            }
            println!(
                "finding distance between {:?} with angle {:?} and {:?} with angle {:?}",
                p1, i, p2, j
            );
            //            if self.valid_path(&p1, &p2) {
            //probably want to push points in this case later
            match self.valid_path(&p1, &p2) {
                PathValidity::Valid => {
                    connections.push((*i, *j, p1.distance(&p2), 0f32));
                    point_connections.push((p1, p2));
                }
                PathValidity::Flyover(h_min) => {
                    connections.push((*i, *j, p1.distance(&p2), h_min));
                    point_connections.push((p1, p2));
                }
                _ => {
                    println!("im sad");
                }
            }
        }
        (connections, sentinels)
    }

    // check if a path is valid (not blocked by flightzone or obstacles)
    fn valid_path(&self, a: &Point, b: &Point) -> PathValidity {
        let theta_o = (b.z - a.z).atan2(a.distance(b));
        //check if angle of waypoints is valid
        if theta_o > MAX_ANGLE_ASCENT {
            return PathValidity::Invalid;
        }
        println!("validating path: {:?}, {:?}", a, b);
        // latitude is y, longitude is x
        // flyzone is array connected by each index
        // some messy code to link flyzone points, can definitely be better
        for flyzone in &self.flyzones {
            let mut tempzone = flyzone.clone();
            let first = Point::from_location(&tempzone.remove(0), &self.origin);
            let mut temp = first;
            for location in tempzone {
                //println!("origin: {:?}", &self.origin);
                let point = Point::from_location(&location, &self.origin);
                //println!("test intersect for {:?} {:?} {:?} {:?}", a, b, &temp, &point);
                if intersect(a, b, &temp, &point) {
                    //println!("false due to flyzone");
                    return PathValidity::Invalid;
                }
                temp = point;
            }
            //println!("test intersect for {:?} {:?} {:?} {:?}", a, b, &temp, &first);
            if intersect(a, b, &temp, &first) {
                //println!("false due to flyzone");
                return PathValidity::Invalid;
            }
        }

        // test for obstacles
        for obstacle in &self.obstacles {
            // catch the simple cases for now: if a or b are inside the radius of obstacle, invalid
            // check if there are two points of intersect, for flyover cases
            if let (Some(p1), Some(p2)) = self.perpendicular_intersect(a, b, obstacle) {
                println!("p1:{:?}, p2:{:?}", p1, p2);
                let theta1 =
                //if a.z > b.z {
                //   (p2.z - a.z).atan2(a.distance(&p2))
                //}
				//else if a.z < b.z {
                //    (p1.z - a.z).atan2(a.distance(&p1))
                //}
				//else {
				//	0.0
				//};
				match (a.z, b.z) {
					(ah, bh) if ah > bh => (p2.z - a.z).atan2(a.distance(&p2)),
					(ah, bh) if ah < bh =>	(p1.z - a.z).atan2(a.distance(&p1)),
					_ => 0f32
				};
                if theta1 == 0f32 && a.z < obstacle.height {
                    return PathValidity::Invalid;
                } else if theta_o < theta1 {
                    return PathValidity::Invalid;
                } else {
                    return PathValidity::Flyover(obstacle.height);
                }
            }
        }
        PathValidity::Valid
    }

    // temporary placeholder function to test functionality of point determination
    pub fn perpendicular_intersect(
        &self,
        a: &Point,
        b: &Point,
        c: &Obstacle,
    ) -> (Option<Point>, Option<Point>) {
        // intersect distance gives x and y of intersect point, then distance
        // calculates the shortest distance between the segment and obstacle. If less than radius, it intersects.
        let (x, y, distance, endpoint) =
            intersect_distance(a, b, &Point::from_location(&c.location, &self.origin));
        if distance.sqrt() < c.radius as f32 {
            // immediately check if the endpoint is the shortest distance; can't fly over in this case
            // EXCEPTION: endpoint is inside obstacle but still generates a perpendicular.
            // if endpoint {
            //     // not technically none, but should be considered as such as we will stop calculations
            //     return (None, None);
            // }
            let mag = (c.radius.powi(2) - distance).sqrt();
            //println!("mag: {}", mag);
            //calculate unit vectors for y and x directions
            let dx = (a.x - b.x) / a.distance(b);
            let dy = (a.y - b.y) / a.distance(b);

            let p1 = Point::new(x + dx * mag, y + dy * mag, c.height);
            let p2 = Point::new(x - dx * mag, y - dy * mag, c.height);
            return (Some(p1), Some(p2));
        } else {
            return (None, None);
        }
    }

    // Return intersection point(s) of line given by Point A and B and circle at point C with radius r
    pub fn circular_intersect(
        &self,
        a: &Point,
        b: &Point,
        obstacle: &Obstacle,
    ) -> (Option<Point>, Option<Point>) {
        //y = mx + b for point a and b

        let mut c = Point::from_location(&obstacle.location, &self.origin);
        c.z = obstacle.height;
        let dx = b.x - a.x;
        let dy = b.y - a.y;

        let (indep, dep, slope, slope_intercept) = if dx >= dy {
            let indep = c.x;
            let dep = c.y;
            let slope = (b.y - a.y) / (b.x - a.x);
            let slope_intercept = b.y - slope * b.x;
            (indep, dep, slope, slope_intercept)
        } else {
            let indep = c.y;
            let dep = c.x;
            let slope = (b.x - a.x) / (b.y - a.y);
            let slope_intercept = b.x - slope * b.y;
            (indep, dep, slope, slope_intercept)
        };

        //Quadratic to solve for intersects
        let quad_a = slope.powi(2) + 1.0;
        let quad_b = 2.0 * (slope * slope_intercept - slope * dep - indep);
        let quad_c = indep.powi(2) + dep.powi(2) + slope_intercept.powi(2)
            - 2.0 * slope_intercept * dep
            - obstacle.radius.powi(2);

        //Check discriminant (if > 0, 2 intersects; if = 0, 1 intersect; if < 0, no intersects)
        let discriminant = quad_b.powi(2) - 4.0 * quad_a * quad_c;

        //Returning value of NAN for no solution points
        if discriminant < 0.0 {
            (None, None)
        } else if discriminant == 0.0 {
            let intersect_1: Point = if dx >= dy {
                Point::new(
                    (-1.0) * quad_b / (2.0 * quad_a),
                    slope * ((-1.0) * quad_b / (2.0 * quad_a)) + slope_intercept,
                    c.z,
                ) //CURRENTLY JUST USES OBS HEIGHT
            } else {
                Point::new(
                    slope * ((-1.0) * quad_b / (2.0 * quad_a)) + slope_intercept,
                    (-1.0) * quad_b / (2.0 * quad_a),
                    c.z,
                ) //CURRENTLY JUST USES OBS HEIGHT
            };
            (Some(intersect_1), None)
        } else
        //if(discriminant > 0.0)
        {
            let (intersect_1, intersect_2) = if dx >= dy {
                (
                    Point::new(
                        ((-1.0) * quad_b - (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                            / (2.0 * quad_a),
                        slope
                            * (((-1.0) * quad_b - (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                                / (2.0 * quad_a))
                            + slope_intercept,
                        c.z,
                    ),
                    Point::new(
                        ((-1.0) * quad_b + (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                            / (2.0 * quad_a),
                        slope
                            * (((-1.0) * quad_b + (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                                / (2.0 * quad_a))
                            + slope_intercept,
                        c.z,
                    ),
                )
            } else {
                (
                    Point::new(
                        slope
                            * (((-1.0) * quad_b - (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                                / (2.0 * quad_a))
                            + slope_intercept,
                        ((-1.0) * quad_b - (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                            / (2.0 * quad_a),
                        c.z,
                    ),
                    Point::new(
                        slope
                            * (((-1.0) * quad_b + (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                                / (2.0 * quad_a))
                            + slope_intercept,
                        ((-1.0) * quad_b + (quad_b.powi(2) - 4.0 * quad_a * quad_c).sqrt())
                            / (2.0 * quad_a),
                        c.z,
                    ),
                )
            };
            (Some(intersect_1), Some(intersect_2))
        }
    }
}
