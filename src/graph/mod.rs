// Visibility related code for modularity
use super::*;

mod connection;
mod node;
mod point;
mod util;
mod vertex;

use graph::util::*;
use obj::{Location, Obstacle};

#[derive(Copy, Clone, Debug)]
pub struct Point {
    pub x: f32, // horizontal distance from origin in meters
    pub y: f32, // vertical distance from origin in meters
    pub z: f32,
}

#[derive(Debug)]
pub struct Vertex {
    pub angle: f32,                        // Angle with respect to the node
    pub connection: Option<Connection>,    // Edge connecting to another node
    pub next: Option<Rc<RefCell<Vertex>>>, // Neighbor vertex in the same node
    pub sentinel: bool,                    // Sentinel property marks end of path hugging
}

// Represent a connection between two nodes
// Contains the coordinate of tangent line and distance
#[derive(Debug)]
pub struct Connection {
    pub neighbor: Rc<RefCell<Vertex>>, // Connected node through a tangent
    distance: f32,
}

pub enum PathValidity {
    Valid,
    Invalid,
    Flyover(Point),
}

impl From<PathValidity> for bool {
    fn from(pv: PathValidity) -> bool {
        match pv {
            PathValidity::Invalid => false,
            _ => true,
        }
    }
}

pub struct Node {
    pub origin: Point,
    pub radius: f32,
    height: f32,
    left_ring: Option<Rc<RefCell<Vertex>>>,
    right_ring: Option<Rc<RefCell<Vertex>>>,
}

impl Pathfinder {
    pub fn build_graph(&mut self) {
        self.populate_nodes();
        for i in 0..self.nodes.len() {
            for j in i + 1..self.nodes.len() {
                let (paths, sentinels) = self.find_path(&self.nodes[i].borrow(), &self.nodes[j].borrow());
                for (alpha, beta, distance) in paths {
                    // println!("path: {} {} {}", alpha, beta, distance);
                    let v = Rc::new(RefCell::new(Vertex::new(beta, None)));
                    let edge = Connection::new(v, distance);
                    let u = Rc::new(RefCell::new(Vertex::new(alpha, Some(edge))));
                    self.nodes[i].borrow_mut().insert_vertex(u);

                    // reciprocal
                    let v = Rc::new(RefCell::new(Vertex::new(
                        (2f32 * PI - alpha) % (2f32 * PI),
                        None,
                    )));
                    let edge = Connection::new(v, distance);
                    let u = Rc::new(RefCell::new(Vertex::new(
                        (2f32 * PI - beta) % (2f32 * PI),
                        Some(edge),
                    )));
                    self.nodes[j].borrow_mut().insert_vertex(u);
                }
				if sentinels.is_some() {
					for (alpha_s, beta_s) in sentinels.unwrap() {
						let mut a = Vertex::new(alpha_s, None);
						a.set_sentinel();
						let mut b = Vertex::new(beta_s, None);
						b.set_sentinel();
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
            let vec_a = (a.x - vertex.x , a.y - vertex.y);
            let vec_b = (b.x - vertex.x , b.y - vertex.y);
            let mag_a = ((vec_a.0).powi(2) + (vec_a.1).powi(2)).sqrt();
            let mag_b = ((vec_b.0).powi(2) + (vec_b.1).powi(2)).sqrt();
            let bisect = (
                mag_b * vec_a.0 + mag_a * vec_b.0,
                mag_b * vec_a.1 + mag_a * vec_b.1,
            );
            //println!("{:?}", vec_a);
            //println!("{:?}", vec_b);
            let mag_bisection = ((bisect.0).powi(2) + (bisect.1).powi(2)).powf(0.5);
            let bisection = (bisect.0 / mag_bisection, bisect.1 / mag_bisection);
            //println!("bisection: {:?}", bisection);
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

    // calculate distance of shortest distance from point to a segment defined by two lines

    // Generate all valid possible path (tangent lines) between two nodes, and return the
    // shortest valid path if one exists

    fn find_path(&self, a: &Node, b: &Node) -> (Vec<(f32, f32, f32)>, Option<Vec<(f32, f32)>>) {
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
        if dist > r1 + r2 {
            candidates = vec![
                (theta1, phi1),
                (theta2, phi2),
                (theta3, phi3),
                (theta4, phi4),
            ];
        } else {
            candidates = vec![(theta1, phi1), (theta2, phi2)];
			//determine angle locations of sentinels
			let theta_s = ((r1.powi(2) + dist.powi(2) - r2.powi(2))/(2f32 * r1 * dist)).acos();
			let phi_s = ((r2.powi(2) + dist.powi(2) - r1.powi(2))/(2f32 * r2 * dist)).acos();
			println!("Generating Sentinels: Theta = {:?}, Phi = {:?}", theta_s, phi_s);
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
                    connections.push((*i, *j, p1.distance(&p2)));
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

#[cfg(test)]
mod test {
    use super::*;
    use obj::Obstacle;
    const THRESHOLD: f64 = 0.001;

    //assert equal, equal practically because floating points suck for intersection
    macro_rules! assert_eqp {
        ($x:expr, $y:expr, $d:expr) => {
            if !((($x - $y) as f64).abs() < $d) {
                //println!("{} vs {}", $x, $y);
                panic!();
            }
        };
    }

	//compare two vectors with tuple of 2 elements
    fn assert_vec2_eqp(v1: &Vec<(f32, f32)>, v2: &Vec<(f32, f32)>) {
        for i in 0..v1.len() {
            let a = v1[i];
            let b = v2[i];
			println!("comparing {:?} with {:?}", a, b);
            assert_eqp!(a.0, b.0, THRESHOLD);
            assert_eqp!(a.1, b.1, THRESHOLD);
        }
    }
	
	//compare two vectors of tuple with 3 elements
    fn assert_vec3_eqp(v1: &Vec<(f32, f32, f32)>, v2: &Vec<(f32, f32, f32)>) {
        for i in 0..v1.len() {
            let a = v1[i];
            let b = v2[i];
            assert_eqp!(a.0, b.0, THRESHOLD);
            assert_eqp!(a.1, b.1, THRESHOLD);
            assert_eqp!(a.2, b.2, THRESHOLD);
        }
    }

    fn assert_point_eq(a: &Point, b: &Point) {
        assert_eqp!(a.x, b.x, THRESHOLD);
        assert_eqp!(a.y, b.y, THRESHOLD);
    }

    fn dummy_origin() -> Location {
        Location::from_radians(0f64, 0f64, 0f32)
    }

    // Helper function to create an obstacle based on its position in transformed graph
    fn obstacle_from_meters(x: f32, y: f32, radius: f32, height: f32) -> Obstacle {
        Obstacle::new(
            Location::from_meters(x, y, height, &dummy_origin()),
            radius,
            height,
        )
    }

    fn dummy_flyzones() -> Vec<Vec<Location>> {
        let a = Point::new(0f32, 0f32, 10f32);
        let b = Point::new(0f32, 400f32, 10f32);
        let c = Point::new(400f32, 400f32, 10f32);
        let d = Point::new(400f32, 0f32, 10f32);
        vec![points_to_flyzone(vec![a, b, c, d])]
    }

    fn dummy_pathfinder() -> Pathfinder {
        Pathfinder::create(1f32, dummy_flyzones(), Vec::new())
    }

    fn points_to_flyzone(points: Vec<Point>) -> Vec<Location> {
        let mut flyzone = Vec::new();
        for point in points {
            flyzone.push(point.to_location(&dummy_origin()));
        }
        flyzone
    }

    #[test]
    fn flyzone_pathing() {
        let a = Point::new(40f32, 0f32, 10f32);
        let b = Point::new(40f32, 40f32, 10f32);
        let c = Point::new(0f32, 40f32, 10f32);
        let d = Point::new(0f32, 0f32, 10f32);
        let flyzones = vec![points_to_flyzone(vec![a, b, c, d])];
        let pathfinder = Pathfinder::create(1f32, flyzones, Vec::new());

        let e = Point::new(20f32, 20f32, 10f32);
        let f = Point::new(30f32, 30f32, 10f32);
        let g = Point::new(20f32, 50f32, 10f32);

        let h = Point::new(50f32, 50f32, 10f32);
        let i = Point::new(50f32, 0f32, 10f32);

        assert_eq!(bool::from(pathfinder.valid_path(&e, &f)), true);
        assert_eq!(bool::from(pathfinder.valid_path(&e, &g)), false);
        assert_eq!(bool::from(pathfinder.valid_path(&f, &g)), false);
        assert_eq!(bool::from(pathfinder.valid_path(&a, &b)), false);
        assert_eq!(bool::from(pathfinder.valid_path(&a, &h)), false);

        //here some points are outside of the flyzone; should this be a special case?
        //should we assume that the points we evaluate will always be inside the flyzone?
        assert_eq!(bool::from(pathfinder.valid_path(&h, &i)), true);
        assert_eq!(bool::from(pathfinder.valid_path(&h, &e)), false);
    }

    #[test]
    fn flyzones_pathing() {
        let a = Point::new(40f32, 0f32, 10f32);
        let b = Point::new(40f32, 40f32, 10f32);
        let c = Point::new(0f32, 40f32, 10f32);
        let d = Point::new(0f32, 0f32, 10f32);

        let e = Point::new(30f32, 10f32, 10f32);
        let f = Point::new(30f32, 30f32, 10f32);
        let g = Point::new(10f32, 30f32, 10f32);
        let h = Point::new(10f32, 10f32, 10f32);

        let flyzone1 = points_to_flyzone(vec![a, b, d, c]);
        let flyzone2 = points_to_flyzone(vec![e, f, h, g]);

        let flyzones = vec![flyzone1, flyzone2];

        let mut pathfinder = Pathfinder::new();
        pathfinder.init(1f32, flyzones, Vec::new());

        //test breaks with multiple flyzones; must declare every flyzone from meters at (0,0)
        /*let i = Point::new(15f32, 15f32, 10f32);
        let j = Point::new(25f32, 25f32, 10f32);
        let k = Point::new(35f32, 5f32, 10f32);
        let l = Point::new(50f32, 50f32, 10f32);
        let m = Point::new(35f32, 25f32, 10f32);

        assert_eq!(pathfinder.valid_path(&i, &j), true);
        assert_eq!(pathfinder.valid_path(&i, &k), false);
        assert_eq!(pathfinder.valid_path(&i, &l), false);
        assert_eq!(pathfinder.valid_path(&k, &l), false);
        assert_eq!(pathfinder.valid_path(&k, &m), true);
		*/    }

    #[test]
    fn obstacles_pathing() {
        let a = Point::new(20f32, 40f32, 10f32);
        let b = Point::new(20f32, 1f32, 10f32);
        let c = Point::new(20f32, 60f32, 10f32);
        let d = Point::new(60f32, 20f32, 10f32);
        let e = Point::new(20f32, 30f32, 10f32);

        let ob = obstacle_from_meters(20f32, 20f32, 20f32, 20f32);
        let obstacles = vec![ob];

        let mut pathfinder = Pathfinder::new();
        pathfinder.init(1f32, dummy_flyzones(), obstacles);

        assert_eq!(bool::from(pathfinder.valid_path(&a, &b)), false);
        assert_eq!(bool::from(pathfinder.valid_path(&c, &d)), true);
        assert_eq!(bool::from(pathfinder.valid_path(&c, &e)), false);
    }

    #[test]
    fn intersects_circle() {
        //Desmos Visual: https://www.desmos.com/calculator/fxknkpinao

        //Test Object - Desmos Eq 1
        let pathfinder = dummy_pathfinder();
        let ob = obstacle_from_meters(15f32, 0f32, 5f32, 20f32);

        //Check intersections of line from (0,0) to (30,0) with circle of radius 5 centered at (15,0)
        //2 sol - Desmos Eq 2
        let a = Point::new(0f32, 0f32, 0f32);
        let b = Point::new(30f32, 0f32, 0f32);

        let (c1, c2) = pathfinder.circular_intersect(&a, &b, &ob);
        assert!(c1.is_some());
        assert_eq!(c1.unwrap().x, 10f32);
        assert_eq!(c1.unwrap().y, 0f32);

        assert!(c2.is_some());
        assert_eq!(c2.unwrap().x, 20f32);
        assert_eq!(c2.unwrap().y, 0f32);

        //Check intersections of line from (0,5) to (30,5) with circle of radius 5 centered at (15,0)
        //1 sol - Desmos Eq 3
        let d = Point::new(0f32, 5f32, 0f32);
        let e = Point::new(30f32, 5f32, 0f32);

        let (f1, f2) = pathfinder.circular_intersect(&d, &e, &ob);
        assert!(f1.is_some());
        assert_eq!(f1.unwrap().x, 15f32);
        assert_eq!(f1.unwrap().y, 5f32);

        assert!(f2.is_none());

        //Check intersections of line from (10,-5) to (10,5) with circle of radius 5 centered at (15,0)
        //1 sol - Desmos Eq 4
        let g = Point::new(10f32, -5f32, 0f32);
        let h = Point::new(10f32, 5f32, 0f32);

        let (i1, i2) = pathfinder.circular_intersect(&g, &h, &ob);
        assert!(i1.is_some());
        assert_eq!(i1.unwrap().x, 10f32);
        assert_eq!(i1.unwrap().y, 0f32);

        assert!(i2.is_none());

        //Check intersections of line from (10,-5) to (20,5) , y = x-15, with circle of radius 5 centered at (15,0)
        //2 sol - Desmos Eq 5
        let j = Point::new(10f32, -5f32, 0f32);
        let k = Point::new(20f32, 5f32, 0f32);

        let (l1, l2) = pathfinder.circular_intersect(&j, &k, &ob);
        assert!(l1.is_some());
        assert_eq!((l1.unwrap().x * 1000.0).round() / 1000.0, 11.464f32); //Rounded to 3 decimal
        assert_eq!((l1.unwrap().y * 1000.0).round() / 1000.0, -3.536f32); //Rounded to 3 decimal

        assert!(l2.is_some());
        assert_eq!((l2.unwrap().x * 1000.0).round() / 1000.0, 18.536f32);
        assert_eq!((l2.unwrap().y * 1000.0).round() / 1000.0, 3.536f32);

        //Check intersections of line from (10,10) to (15,-10) with circle of radius 5 centered at (15,0)
        //2 sol - Desmos Eq 6
        let m = Point::new(10f32, 10f32, 0f32);
        let n = Point::new(15f32, -10f32, 0f32);

        let (o1, o2) = pathfinder.circular_intersect(&m, &n, &ob);
        assert!(o1.is_some());
        assert_eq!((o1.unwrap().x * 1000.0).round() / 1000.0, 11.587f32); //Rounded to 3 decimal
        assert_eq!((o1.unwrap().y * 1000.0).round() / 1000.0, 3.654f32); //Rounded to 3 decimal

        assert!(o2.is_some());
        assert_eq!((o2.unwrap().x * 1000.0).round() / 1000.0, 13.708f32);
        assert_eq!((o2.unwrap().y * 1000.0).round() / 1000.0, -4.83f32);
    }

    #[test]
    fn intersection_distance() {
        let ax = Point::new(0f32, 0f32, 0f32);
        let ay = Point::new(30f32, 0f32, 0f32);

        let bx = Point::new(10f32, 0f32, 0f32);
        let by = Point::new(20f32, 0f32, 0f32);

        let ob = obstacle_from_meters(15f32, 0f32, 5f32, 20f32);
        let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());

        //intercepts at (10,0), (20,0)
        assert_eq!(
            intersect_distance(
                &ax,
                &ay,
                &Point::from_location(&ob.location, &dummy_origin())
            ).2,
            0f32
        );
        let result = pathfinder.perpendicular_intersect(&ax, &ay, &ob);
        println!("{:?} {:?}", result.0.unwrap(), result.1.unwrap());
        assert_point_eq(&result.0.unwrap(), &bx);
        assert_point_eq(&result.1.unwrap(), &by);
    }

    #[test]
    fn circular_intersection() {
        //Desmos Visual: https://www.desmos.com/calculator/zkfgbbexkm

        //Check intersections of line from (0,0) to (30,0) with circle of radius 5 centered at (15,0)
        //2 sol
        let a = Point::new(0f32, 0f32, 0f32);
        let b = Point::new(30f32, 0f32, 0f32);

        let ob = obstacle_from_meters(15f32, 0f32, 5f32, 20f32);

        let pathfinder = dummy_pathfinder();
        let (c1, c2) = pathfinder.perpendicular_intersect(&a, &b, &ob);
        assert!(c1.is_some());
        assert_eq!(c1.unwrap().y, 0f32);
        assert_eq!(c1.unwrap().x, 10f32);

        assert!(c2.is_some());
        assert_eq!(c2.unwrap().y, 0f32);
        assert_eq!(c2.unwrap().x, 20f32);

        //Check intersections of line from (0,5) to (30,5) with circle of radius 5 centered at (15,0)
        //intersects at 1 point, should be considered valid
        let d = Point::new(0f32, 5f32, 0f32);
        let e = Point::new(30f32, 5f32, 0f32);

        let (f1, f2) = pathfinder.perpendicular_intersect(&d, &e, &ob);
        assert!(f1.is_none());
        assert!(f2.is_none());

        //Check intersections of line from (0,5) to (15,5) with circle of radius 5 centered at (15,0)
        //intersects at 1 point, should be considered valid
        let g = Point::new(10f32, -5f32, 0f32);
        let h = Point::new(10f32, 5f32, 0f32);

        let (i1, i2) = pathfinder.perpendicular_intersect(&g, &h, &ob);
        assert!(i1.is_none());
        //assert_eq!(i1.unwrap().y, 15f32);
        //assert_eq!(i1.unwrap().x, 5f32);

        assert!(i2.is_none());

        //should intersect at two points
        let j = Point::new(8f32, -2f32, 0f32);
        let k = Point::new(16f32, 6f32, 0f32);

        let (l1, l2) = pathfinder.perpendicular_intersect(&j, &k, &ob);
        assert!(l1.is_some());

        assert_eqp!(l1.unwrap().y, 0f32, 0.0001);
        assert_eqp!(l1.unwrap().x, 10f32, 0.0001);

        assert!(l2.is_some());
        assert_eqp!(l2.unwrap().y, 5f32, 0.0001);
        assert_eqp!(l2.unwrap().x, 15f32, 0.0001);

        //should intersect at two points
        let m = Point::new(8f32, 4f32, 0f32);
        let n = Point::new(30f32, -6f32, 0f32);

        let (o1, o2) = pathfinder.perpendicular_intersect(&m, &n, &ob);
        assert_eqp!(o1.unwrap().x, 10.807f32, 0.001);
        assert_eqp!(o1.unwrap().y, 2.724f32, 0.001);
        assert_eqp!(o2.unwrap().x, 19.809f32, 0.001);
        assert_eqp!(o2.unwrap().y, -1.368f32, 0.001);
    }

    #[test]
    fn obstacle_flyover() {
        //Graphical Visualization: https://www.geogebra.org/3d/a55hmxfy
        let a = Point::new(10f32, 10f32, 10f32);
        let b = Point::new(10f32, 40f32, 10f32);
        let c = Point::new(10f32, 30f32, 30f32);

        let d = Point::new(10f32, 40f32, 25f32);
        let e = Point::new(10f32, 10f32, 25f32);
        let f = Point::new(20f32, 40f32, 30f32);
        let g = Point::new(10f32, 30f32, 40f32);
        let ob = obstacle_from_meters(10f32, 25f32, 5f32, 20f32);
        let obstacles = vec![ob];
        let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), obstacles);
        assert_eq!(bool::from(pathfinder.valid_path(&a, &b)), false);
        assert_eq!(bool::from(pathfinder.valid_path(&a, &d)), false);
        assert_eq!(bool::from(pathfinder.valid_path(&e, &b)), false);
        assert_eq!(bool::from(pathfinder.valid_path(&b, &e)), false);

        assert_eq!(bool::from(pathfinder.valid_path(&d, &e)), true);
        assert_eq!(bool::from(pathfinder.valid_path(&a, &c)), true);
        assert_eq!(bool::from(pathfinder.valid_path(&a, &g)), true);

        assert_eq!(bool::from(pathfinder.valid_path(&a, &f)), false);
    }

    #[test]
    fn generate_graph() {
        let a = Point::new(40f32, 0f32, 0f32);
        let b = Point::new(40f32, 40f32, 0f32);
        let c = Point::new(0f32, 40f32, 0f32);
        let d = Point::new(0f32, 0f32, 0f32);
        let flyzones = vec![points_to_flyzone(vec![a, b, c, d])];
        let obstacles = vec![
            obstacle_from_meters(10f32, 20f32, 10f32, 10f32),
            obstacle_from_meters(30f32, 20f32, 10f32, 10f32),
        ];
        let mut pathfinder = Pathfinder::new();
        pathfinder.init(5f32, flyzones, obstacles);
    }

    #[test]
    fn same_radius_test() {
        let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());

        let n1 = Node::new(Point::new(30_f32, 30_f32, 0_f32), 1_f32, 0_f32);
        let n2 = Node::new(Point::new(20_f32, 30_f32, 0_f32), 1_f32, 0_f32);
        let a1 = Rc::new(n1);
        let b1 = Rc::new(n2);
        let expected = vec![
            (PI / 2_f32, PI / 2_f32, 10f32),
            (-PI / 2_f32, -PI / 2_f32, 10f32),
            (
                (2_f32 / 10f32).acos(),
                -PI + (2_f32 / 10f32).acos(),
                96f32.sqrt(),
            ),
            (
                -(2_f32 / 10f32).acos(),
                PI - (2_f32 / 10f32).acos(),
                96f32.sqrt(),
            ),
        ];
        assert_vec3_eqp(&pathfinder.find_path(&a1, &b1).0, &expected);
    }

    #[test]
    fn overlap_test() {
        let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
        let n3 = Node::new(Point::new(15_f32, 10_f32, 0_f32), 5_f32, 0_f32);
        let n4 = Node::new(Point::new(20_f32, 10_f32, 0_f32), 4_f32, 0_f32);
        let c = Rc::new(n3);
        let d = Rc::new(n4);
        let expected = vec![
            ((1_f32 / 5_f32).acos(), (1_f32 / 5_f32).acos(), 24f32.sqrt()),
            (
                -(1_f32 / 5_f32).acos(),
                -(1_f32 / 5_f32).acos(),
                24f32.sqrt(),
            ),
        ];
        assert_vec3_eqp(&pathfinder.find_path(&c, &d).0, &expected);
    }

	#[test]
	fn sentinel_test() {
		let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
        let n3 = Node::new(Point::new(15_f32, 10_f32, 0_f32), 5_f32, 0_f32);
        let n4 = Node::new(Point::new(20_f32, 10_f32, 0_f32), 5_f32, 0_f32);
        let c = Rc::new(n3);
        let d = Rc::new(n4);
        let expected = vec![(PI/3f32, 2f32 * PI/3f32), (-PI/3f32, 4f32 * PI /3f32), (-5f32*PI/3f32, -2f32*PI/3f32), (5f32 * PI/3f32, -4f32 * PI/3f32)];
		println!("{:?}", expected);
        assert_vec2_eqp(&pathfinder.find_path(&c, &d).1.unwrap(), &expected);
	}

    #[test]
    fn different_radius_no_overlap_test() {
        let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
        let n5 = Node::new(Point::new(20_f32, 10_f32, 0_f32), 2_f32, 0_f32);
        let n6 = Node::new(Point::new(12_f32, 10_f32, 0_f32), 1_f32, 0_f32);
        let e = Rc::new(n5);
        let f = Rc::new(n6);
        let expected = vec![
            ((1_f32 / 8_f32).acos(), (1_f32 / 8_f32).acos(), 63f32.sqrt()),
            (
                -(1_f32 / 8_f32).acos(),
                -(1_f32 / 8_f32).acos(),
                63f32.sqrt(),
            ),
            (
                (3_f32 / 8_f32).acos(),
                -PI + (3_f32 / 8_f32).acos(),
                55f32.sqrt(),
            ),
            (
                -(3_f32 / 8_f32).acos(),
                PI - (3_f32 / 8_f32).acos(),
                55f32.sqrt(),
            ),
        ];
        assert_vec3_eqp(&pathfinder.find_path(&e, &f).0, &expected);
    }

    #[test]
    fn virtualize_flyzone_square() {
        let origin = Location::from_degrees(0f64, 0f64, 0f32);
        let a = Point::new(0f32, 0f32, 10f32).to_location(&origin);
        let b = Point::new(20f32, 0f32, 10f32).to_location(&origin);
        let c = Point::new(20f32, 20f32, 10f32).to_location(&origin);
        let d = Point::new(0f32, 20f32, 10f32).to_location(&origin);
        let test_flyzone = vec![vec![d, c, b, a]];
        let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
        let node_a = Point::new(5f32, 5f32, 0f32);
        let node_b = Point::new(15f32, 5f32, 0f32);
        let node_c = Point::new(15f32, 15f32, 0f32);
        let node_d = Point::new(5f32, 15f32, 0f32);
        let expected = vec![node_d, node_c, node_b, node_a];
        let test_flyzone = vec![vec![d, c, b, a]];
        for i in 0..4 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
        let test_flyzone = vec![vec![a, b, c, d]];
        pathfinder.set_flyzone(test_flyzone);
        for i in 0..4 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
    }

    #[test]
    fn virtualize_flyzone_plus() {
        let origin = Location::from_degrees(0f64, 0f64, 0f32);
        let a =Point::new(20f32, 0f32, 10f32).to_location(&origin);
        let b = Point::new(40f32, 0f32, 10f32).to_location(&origin);
        let c = Point::new(40f32, 20f32, 10f32).to_location(&origin);
        let d = Point::new(60f32, 20f32, 10f32).to_location(&origin);
        let e = Point::new(60f32, 40f32, 10f32).to_location(&origin);
        let f = Point::new(40f32, 40f32, 10f32).to_location(&origin);
        let g = Point::new(40f32, 60f32, 10f32).to_location(&origin);
        let h = Point::new(20f32, 60f32, 10f32).to_location(&origin);
        let i = Point::new(20f32, 40f32, 10f32).to_location(&origin);
        let j = Point::new(0f32, 40f32, 10f32).to_location(&origin);
        let k = Point::new(0f32, 20f32, 10f32).to_location(&origin);
        let l =Point::new(20f32, 20f32, 10f32).to_location(&origin);
        let test_flyzone = vec![vec![l, k, j, i, h, g, f, e, d, c, b, a]];
        let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
        let node_a = Point::new(25f32,5f32, 0f32);
        let node_b = Point::new(35f32, 5f32, 0f32);
        let node_c = Point::new(40f32+(25f32/2f32).sqrt(), 20f32-(25f32/2f32).sqrt(), 0f32);
        let node_d = Point::new(55f32, 25f32, 0f32);
        let node_e = Point::new(55f32,35f32, 0f32);
        let node_f = Point::new(40f32+(25f32/2f32).sqrt(), 40f32+(25f32/2f32).sqrt(), 0f32);
        let node_g = Point::new(35f32, 55f32, 0f32);
        let node_h = Point::new(25f32, 55f32, 0f32);
        let node_i = Point::new(20f32-(25f32/2f32).sqrt(), 40f32+(25f32/2f32).sqrt(), 0f32);
        let node_j = Point::new(5f32, 35f32, 0f32);
        let node_k = Point::new(5f32, 25f32, 0f32);
        let node_l = Point::new(20f32-(25f32/2f32).sqrt(), 20f32-(25f32/2f32).sqrt(), 0f32);
        let expected = vec![node_l, node_k, node_j, node_i, node_h, node_g, node_f,
            node_e, node_d, node_c, node_b, node_a];
        for i in 0..11 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
        let test_flyzone = vec![vec![a, b, c, d, e, f, g, h, i, j, k, l]];
        pathfinder.set_flyzone(test_flyzone);
        for i in 0..4 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
    }

    #[test]
    fn virtualize_flyzone_linear() {
        let origin = Location::from_degrees(0f64, 0f64, 0f32);
        let a = Point::new(0f32, 0f32, 10f32).to_location(&origin);
        let b = Point::new(20f32, 0f32, 10f32).to_location(&origin);
        let c = Point::new(20f32, 10f32, 10f32).to_location(&origin);
        let d = Point::new(20f32, 20f32, 10f32).to_location(&origin);
        let e = Point::new(0f32, 20f32, 10f32).to_location(&origin);
        let test_flyzone = vec![vec![e, d, c, b, a]];
        let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
        let node_a = Point::new(5f32, 5f32, 0f32);
        let node_b = Point::new(15f32, 5f32, 0f32);
        let node_c = Point::new(15f32, 15f32, 0f32);
        let node_d = Point::new(5f32, 15f32, 0f32);
        let expected = vec![node_d, node_c, node_b, node_a];
        for i in 0..4 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
        let test_flyzone = vec![vec![a, b, c, d, e]];
        pathfinder.set_flyzone(test_flyzone);
        for i in 0..4 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
    }

    #[test]
    fn virtualize_flyzone_small_angle() {
        let origin = Location::from_degrees(0f64, 0f64, 0f32);
        let a = Point::new(10f32, 0f32, 10f32).to_location(&origin);
        let b = Point::new(30f32, 0f32, 10f32).to_location(&origin);
        let c = Point::new(30f32, 20f32, 10f32).to_location(&origin);
        let d = Point::new(10f32, 20f32, 10f32).to_location(&origin);
        let e = Point::new(10f32, 11f32, 10f32).to_location(&origin);
        let f = Point::new(0f32, 10f32, 10f32).to_location(&origin);
        let g = Point::new(10f32, 9f32, 10f32).to_location(&origin);
        let test_flyzone = vec![vec![g, f, e, d, c, b, a]];
        let mut pathfinder = Pathfinder::create(1f32, test_flyzone, Vec::new());
        let node_a = Point::new(15f32, 5f32, 0f32);
        let node_b = Point::new(25f32, 5f32, 0f32);
        let node_c = Point::new(25f32, 15f32, 0f32);
        let node_d = Point::new(15f32, 15f32, 0f32);
        /*
        let e_x:f32 = 10f32-5f32*(90f32/(((90f32).powi(2)+(9f32-9f32*(101f32).sqrt()).powi(2)).sqrt()));
        let e_y:f32 = 9f32+5f32*((9f32-9f32*(101f32).sqrt())/(((90f32).powi(2)+(9f32-9f32*(101f32).sqrt()).powi(2)).sqrt()));
        let f_x:f32 = 10f32-5f32*(-90f32/(((90f32).powi(2)+(9f32-9f32*(101f32).sqrt()).powi(2)).sqrt()));
        let f_y:f32 = 11f32-5f32*((9f32-9f32*(101f32).sqrt())/(((90f32).powi(2)+(9f32-9f32*(101f32).sqrt()).powi(2)).sqrt()));
        */
        let node_f = Point::new(6.2927, 5.6450, 0f32);
        let node_e = Point::new(6.2927, 14.3550, 0f32);
        let expected = vec![node_f, node_e, node_d, node_c, node_b, node_a];
        for i in 0..6 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
        let test_flyzone = vec![vec![a, b, c, d, e, f, g]];
        pathfinder.set_flyzone(test_flyzone);
        for i in 0..6 {
            assert_point_eq(&pathfinder.nodes[i].borrow().origin, &expected[i]);
        }
    }

}
