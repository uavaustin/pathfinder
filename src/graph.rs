// Visibility related code for modularity
use node::{Connection, Node, Vertex};
use obj::{Location, Obstacle};
use point::Point;
use {Pathfinder, PI};

use std::cell::RefCell;
use std::rc::Rc;

impl Pathfinder {
    pub fn build_graph(&mut self) {
        self.populate_nodes();
        for i in 0..self.nodes.len() {
            for j in i + 1..self.nodes.len() {
                let paths = self.find_path(&self.nodes[i].borrow(), &self.nodes[j].borrow());
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
        // #TODO: generate virtual obstacles using flyzone
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
    }

    // calculate distance of shortest distance from point to a segment defined by two lines

    // Generate all possible path (tangent lines) between two nodes, and return the
    // shortest valid path if one exists

    fn find_path(&self, a: &Node, b: &Node) -> Vec<(f32, f32, f32)> {
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
        let phi3 = PI - theta4;
        let phi4 = -phi3;
        let candidates;
        if dist > r1 + r2 {
            candidates = vec![
                (theta1, phi1),
                (theta2, phi2),
                (theta3, phi3),
                (theta4, phi4),
            ];
        } else {
            candidates = vec![(theta1, phi1), (theta2, phi2)];
        }

        let mut connections = Vec::new();
        for (i, j) in candidates.iter() {
            let p1 = a.to_point(*i);
            let p2 = b.to_point(*j);
            if self.valid_path(&p1, &p2) {
                connections.push((*i, *j, p1.distance(&p2)));
            }
        }
        connections
    }

    // check if a path is valid (not blocked by flightzone or obstacles)
    fn valid_path(&self, a: &Point, b: &Point) -> bool {
        // latitude is y, longitude is x
        // flyzone is array connected by each index
        // some messy code to link flyzone points, can definitely be better
        for flyzone in &self.flyzones {
            let mut tempzone = flyzone.clone();
            let first = Point::from_location(&tempzone.remove(0), &self.origin);
            let mut temp = first;
            for location in tempzone {
                let point = Point::from_location(&location, &self.origin);
                //println!("test intersect for {} {} {} {}", a, b, &temp, &point);
                if intersect(a, b, &temp, &point) {
                    return false;
                }
                temp = point;
            }
            //println!("test intersect for {} {} {} {}", a, b, &temp, &first);
            if intersect(a, b, &temp, &first) {
                return false;
            }
        }

        self.valid_obstacle_relative(a, b)
    }

    fn valid_obstacle_relative(&self, a: &Point, b: &Point) -> bool {
        // test for obstacles, intersection/width/height
        for obstacle in &self.obstacles {
            //intersect distance gives x and y of intersect point, then distance
            //calculates the shortest distance between the segment and obstacle. If less than radius, it intersects.
            let int_data = intersect_distance(
                a,
                b,
                &Point::from_location(&obstacle.location, &self.origin),
            );
            println!("{:?}", int_data);
            if int_data.2.sqrt() < obstacle.radius {
                //immediately check if the endpoint is the shortest distance; can't fly over in this case
                //EXCEPTION: endpoint is inside obstacle but still generates a perpendicular.
                if int_data.3 {
                    return false;
                }
                //otherwise, we can check if height allows flyover
                let mag = (obstacle.radius.powi(2) - int_data.2).sqrt();
                //calculate unit vectors for y and x directions
                let dy = (a.y - b.y) / a.distance(b);
                let dx = (a.x - b.x) / a.distance(b);
                //p1 closer to a, p2 closer to b
                let p1 = Point::new(
                    int_data.1 + dy * mag,
                    int_data.0 + dx * mag,
                    obstacle.height,
                );
                let p2 = Point::new(
                    int_data.1 - dy * mag,
                    int_data.0 - dx * mag,
                    obstacle.height,
                );
                //if a descends to b, need to clear p2. if a ascends to b, need to clear p1.
                let theta_o = (b.z - a.z).atan2(a.distance(b));
                if a.z > b.z {
                    let theta1 = (p2.z - a.z).atan2(a.distance(&p2));
                    println!("descending, {}, {}", theta1, theta_o);
                    return theta_o >= theta1;
                } else if a.z < b.z {
                    let theta1 = (p1.z - a.z).atan2(a.distance(&p1));
                    println!("ascending, {}, {}", theta1, theta_o);
                    return theta_o >= theta1;
                }
                //else it's a straight altitude line, just check altitude
                else {
                    return a.z >= obstacle.height;
                }
            }
        }

        true
    }

    // temporary placeholder function to test functionality of point determination
    pub fn valid_path_obs(
        &self,
        a: &Point,
        b: &Point,
        c: &Obstacle,
    ) -> (Option<Point>, Option<Point>) {
        //intersect distance gives x and y of intersect point, then distance
        //calculates the shortest distance between the segment and obstacle. If less than radius, it intersects.
        let int_data = intersect_distance(a, b, &Point::from_location(&c.location, &self.origin));
        if int_data.2.sqrt() < c.radius as f32 {
            //println!("distance: {}", int_data.2.sqrt());
            //immediately check if the endpoint is the shortest distance; can't fly over in this case
            //EXCEPTION: endpoint is inside obstacle but still generates a perpendicular.
            //if int_data.3 {
            //not technically none, but should be considered as such as we will stop calculations
            //return (None, None)
            //}
            let mag = (c.radius.powi(2) - int_data.2).sqrt();
            //println!("mag: {}", mag);
            //calculate unit vectors for y and x directions
            let dy = (a.y - b.y) / a.distance(b);
            let dx = (a.x - b.x) / a.distance(b);

            let p1 = Point::new(int_data.0 + dx * mag, int_data.1 + dy * mag, 0f32);
            let p2 = Point::new(int_data.0 - dx * mag, int_data.1 - dy * mag, 0f32);
            return (Some(p1), Some(p2));
        } else {
            return (None, None);
        }
    }
}

// helper function for intersection calculation
// returns the area between three points
fn area(a: &Point, b: &Point, c: &Point) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)
}

// helper function for intersection calculation
// returns true if point c is between a and b, false otherwise
fn between(a: &Point, b: &Point, c: &Point) -> bool {
    if a.x != b.x {
        (a.x <= c.x && c.x <= b.x) || (a.x >= c.x && c.x >= b.x)
    } else {
        (a.y <= c.y && c.y <= b.y) || (a.y >= c.y && c.y >= b.y)
    }
}

// calculate the intersection between four given points
// implement: http://developer.classpath.org/doc/java/awt/geom/Line2D-source.html
// returns true if a line segment a to b and another segment c to d intersect
fn intersect(a: &Point, b: &Point, c: &Point, d: &Point) -> bool {
    // special cases of intersection
    let a1 = area(a, b, c);
    let a2 = area(a, b, d);
    let a3 = area(c, d, a);
    let a4 = area(c, d, b);
    if a1 == 0f32 {
        // checks if c is between a and b OR
        // d is colinear also AND between a and b or at opposite ends?
        if between(a, b, c) {
            return true;
        } else {
            if area(a, b, d) == 0f32 {
                return between(c, d, a) || between(c, d, b);
            } else {
                return false;
            }
        }
    } else if a2 == 0f32 {
        // check if d is between a and b since c is not colinear
        return between(a, b, d);
    }
    if a3 == 0f32 {
        // checks if a is between c and d OR
        // b is colinear AND either between a and b or at opposite ends?
        if between(c, d, a) {
            return true;
        } else {
            if area(c, d, b) == 0f32 {
                return between(a, b, c) || between(a, b, d);
            } else {
                return false;
            }
        }
    } else if a4 == 0f32 {
        // check if b is between c and d since we know a is not colinear
        return between(c, d, b);
    }
    //tests for regular intersection
    else {
        ((a1 > 0f32) ^ (a2 > 0f32)) && ((a3 > 0f32) ^ (a4 > 0f32))
    }
}

// calculate distance of shortest distance from obstacle c to a segment defined by a and b
fn intersect_distance(a: &Point, b: &Point, c: &Point) -> (f32, f32, f32, bool) {
    //calculate distance from a to b, squared
    let pd2 = (a.x - b.x).powi(2) + (a.y - b.y).powi(2);
    // there may be a case where the shortest point is to an endpoint.
    //check for conincidence of points
    let (x, y, endpoint) = if pd2 == 0f32 {
        (a.x, b.y, false)
    }
    //all other cases
    else {
        // calculate "distances" to points a and b (if a and b are shortest distance)
        let u = ((c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y)) / pd2;
        // shortest distance is to point a
        if u < 0f32 {
            (a.x, a.y, true)
        }
        // shortest distance is to point b
        else if u > 1f32 {
            (b.x, b.y, true)
        } else {
            // to perpendicular point on the segment
            (a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), false)
        }
    };
    // returns distance
    (
        x,
        y,
        (x - c.x) * (x - c.x) + (y - c.y) * (y - c.y),
        endpoint,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use obj::Obstacle;
    const THRESHOLD: f64 = 0.001;

    //assert equal, equal practically because floating points suck for intersection
    macro_rules! assert_eqp {
        ($x:expr, $y:expr, $d:expr) => {
            if !((($x - $y) as f64).abs() < $d) {
                println!("{} vs {}", $x, $y);
                panic!();
            }
        };
    }

    fn assert_point_eq(a: &Point, b: &Point) {
        assert_eqp!(a.x, b.x, THRESHOLD);
        assert_eqp!(a.y, b.y, THRESHOLD);
    }

    fn dummy_origin() -> Location {
        Location::from_radians(0f64, 0f64, 0f32)
    }

    fn dummy_flyzones() -> Vec<Vec<Location>> {
        let a = Point::new(40f32, 0f32, 10f32);
        let b = Point::new(40f32, 40f32, 10f32);
        let c = Point::new(0f32, 40f32, 10f32);
        let d = Point::new(0f32, 0f32, 10f32);
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
    fn is_between() {
        let a = Point::new(40f32, 40f32, 10f32);
        let b = Point::new(40f32, 50f32, 10f32);
        let c = Point::new(40f32, 60f32, 10f32);
        assert_eq!(between(&a, &c, &b), true);
        assert_eq!(between(&a, &b, &c), false);
    }

    #[test]
    fn is_colinear() {
        let a = Point::new(40f32, 40f32, 10f32);
        let b = Point::new(40f32, 50f32, 10f32);
        let c = Point::new(40f32, 60f32, 10f32);
        assert_eq!(area(&a, &b, &c), 0f32);
    }

    #[test]
    fn yes_intersect() {
        let a = Point::new(40f32, 0f32, 10f32);
        let b = Point::new(40f32, 40f32, 10f32);
        let c = Point::new(0f32, 0f32, 10f32);
        let d = Point::new(0f32, 40f32, 10f32);
        assert_eq!(intersect(&a, &d, &b, &c), true);
    }

    #[test]
    fn no_intersect() {
        let a = Point::new(40f32, 0f32, 10f32);
        let b = Point::new(40f32, 40f32, 10f32);
        let c = Point::new(0f32, 0f32, 10f32);
        let d = Point::new(0f32, 40f32, 10f32);
        assert_eq!(intersect(&a, &c, &b, &d), false);
        assert_eq!(intersect(&c, &d, &a, &b), false);
    }

    #[test]
    fn special_intersect() {
        let a = Point::new(0f32, 0f32, 10f32);
        let b = Point::new(10f32, 5f32, 10f32);
        let c = Point::new(20f32, 10f32, 10f32);
        let d = Point::new(30f32, 15f32, 10f32);
        assert_eq!(intersect(&a, &b, &c, &d), false);
        assert_eq!(intersect(&a, &c, &b, &d), true);
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

        assert_eq!(pathfinder.valid_path(&e, &f), true);
        assert_eq!(pathfinder.valid_path(&e, &g), false);
        assert_eq!(pathfinder.valid_path(&f, &g), false);
        assert_eq!(pathfinder.valid_path(&a, &b), false);
        assert_eq!(pathfinder.valid_path(&a, &h), false);

        //here some points are outside of the flyzone; should this be a special case?
        //should we assume that the points we evaluate will always be inside the flyzone?
        assert_eq!(pathfinder.valid_path(&h, &i), true);
        assert_eq!(pathfinder.valid_path(&h, &e), false);
    }

    #[test]
    fn flyzones_pathing() {
        let a = Point::new(40f32, 0f32, 10f32);
        let b = Point::new(40f32, 40f32, 10f32);
        let c = Point::new(0f32, 40f32, 10f32);
        let d = Point::new(0f32, 0f32, 10f32);

        let e = Point::new(30f32, 10f32, 10f32);
        let f = Point::new(30f32, 30f32, 10f32);
        let g = Point::new(10f32, 10f32, 10f32);
        let h = Point::new(10f32, 30f32, 10f32);

        let flyzone1 = points_to_flyzone(vec![a, b, d, c]);
        let flyzone2 = points_to_flyzone(vec![e, f, h, g]);

        let flyzones = vec![flyzone1, flyzone2];

        let mut pathfinder = Pathfinder::new();
        pathfinder.init(1f32, flyzones, Vec::new());

        let i = Point::new(15f32, 15f32, 10f32);
        let j = Point::new(25f32, 25f32, 10f32);
        let k = Point::new(35f32, 5f32, 10f32);
        let l = Point::new(50f32, 50f32, 10f32);
        let m = Point::new(35f32, 25f32, 10f32);

        assert_eq!(pathfinder.valid_path(&i, &j), true);
        assert_eq!(pathfinder.valid_path(&i, &k), false);
        assert_eq!(pathfinder.valid_path(&i, &l), false);
        assert_eq!(pathfinder.valid_path(&k, &l), false);
        assert_eq!(pathfinder.valid_path(&k, &m), true);
    }

    #[test]
    fn obstacles_pathing() {
        let a = Point::new(40f32, 20f32, 10f32);
        let b = Point::new(0f32, 20f32, 10f32);
        let c = Point::new(60f32, 20f32, 10f32);
        let d = Point::new(20f32, 60f32, 10f32);
        let e = Point::new(30f32, 20f32, 10f32);

        let ob = Obstacle::new(
            Location::from_meters(20f32, 20f32, 20f32, &dummy_origin()),
            20f32,
            20f32,
        );

        let obstacles = vec![ob];

        let mut pathfinder = Pathfinder::new();
        pathfinder.init(1f32, Vec::new(), obstacles);

        assert_eq!(pathfinder.valid_path(&a, &b), false);
        assert_eq!(pathfinder.valid_path(&c, &d), true);
        assert_eq!(pathfinder.valid_path(&c, &e), false);
    }

    #[test]
    fn intersection_distance() {
        let ax = Point::new(0f32, 0f32, 0f32);
        let ay = Point::new(30f32, 0f32, 0f32);

        let bx = Point::new(10f32, 0f32, 0f32);
        let by = Point::new(20f32, 0f32, 0f32);

        let ob = Obstacle::new(
            Location::from_meters(15f32, 0f32, 0f32, &dummy_origin()),
            5f32,
            20f32,
        );

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
        let result = pathfinder.valid_path_obs(&ax, &ay, &ob);
        println!("{:?} {:?}", result.0.unwrap(), result.1.unwrap());
        assert_point_eq(&result.0.unwrap(), &bx);
        assert_point_eq(&result.1.unwrap(), &by);
    }

    #[test]
    fn circle_intersection() {
        //Desmos Visual: https://www.desmos.com/calculator/zkfgbbexkm

        //Check intersections of line from (0,0) to (30,0) with circle of radius 5 centered at (15,0)
        //2 sol
        let a = Point::new(0f32, 0f32, 0f32);
        let b = Point::new(30f32, 0f32, 0f32);

        let ob = Obstacle::new(
            Location::from_meters(15f32, 0f32, 0f32, &dummy_origin()),
            5f32,
            20f32,
        );

        let pathfinder = dummy_pathfinder();
        let (c1, c2) = pathfinder.valid_path_obs(&a, &b, &ob);
        assert!(c1.is_some());
        assert_eq!(c1.unwrap().y, 10f32);
        assert_eq!(c1.unwrap().x, 0f32);

        assert!(c2.is_some());
        assert_eq!(c2.unwrap().y, 20f32);
        assert_eq!(c2.unwrap().x, 0f32);

        //Check intersections of line from (0,5) to (30,5) with circle of radius 5 centered at (15,0)
        //intersects at 1 point, should be considered valid
        let d = Point::new(0f32, 5f32, 0f32);
        let e = Point::new(30f32, 5f32, 0f32);

        let (f1, f2) = pathfinder.valid_path_obs(&d, &e, &ob);
        assert!(f1.is_none());
        assert!(f2.is_none());

        //Check intersections of line from (0,5) to (15,5) with circle of radius 5 centered at (15,0)
        //intersects at 1 point, should be considered valid
        let g = Point::new(10f32, -5f32, 0f32);
        let h = Point::new(10f32, 5f32, 0f32);

        let (i1, i2) = pathfinder.valid_path_obs(&g, &h, &ob);
        assert!(i1.is_none());
        //assert_eq!(i1.unwrap().y, 15f32);
        //assert_eq!(i1.unwrap().x, 5f32);

        assert!(i2.is_none());

        //should intersect at two points
        let j = Point::new(8f32, -2f32, 0f32);
        let k = Point::new(16f32, 6f32, 0f32);

        let (l1, l2) = pathfinder.valid_path_obs(&j, &k, &ob);
        assert!(l1.is_some());

        assert_eqp!(l1.unwrap().y, 10f32, 0.0001);
        assert_eqp!(l1.unwrap().x, 0f32, 0.0001);

        assert!(l2.is_some());
        assert_eqp!(l2.unwrap().y, 15f32, 0.0001);
        assert_eqp!(l2.unwrap().x, 5f32, 0.0001);

        //should intersect at two points
        let m = Point::new(8f32, 4f32, 0f32);
        let n = Point::new(30f32, -6f32, 0f32);

        let (o1, o2) = pathfinder.valid_path_obs(&m, &n, &ob);
        assert_eqp!(o1.unwrap().y, 10.807f32, 0.001);
        assert_eqp!(o1.unwrap().x, 2.724f32, 0.001);
        assert_eqp!(o2.unwrap().y, 19.809f32, 0.001);
        assert_eqp!(o2.unwrap().x, -1.368f32, 0.001);
    }

    #[test]
    fn obstacle_flyover() {
        //Graphical Visualization: https://www.geogebra.org/3d/a55hmxfy
        let a = Point::new(0f32, 0f32, 10f32);
        let b = Point::new(30f32, 0f32, 10f32);
        let c = Point::new(20f32, 0f32, 30f32);

        let d = Point::new(30f32, 0f32, 25f32);
        let e = Point::new(0f32, 0f32, 25f32);
        let f = Point::new(30f32, 10f32, 30f32);
        let g = Point::new(20f32, 0f32, 40f32);

        let ob = Obstacle::new(
            Location::from_meters(15f32, 0f32, 0f32, &dummy_origin()),
            5f32,
            20f32,
        );

        let obstacles = vec![ob];
        let mut pathfinder = dummy_pathfinder();
        pathfinder.set_obstacles(obstacles);
        assert_eq!(pathfinder.valid_path(&a, &b), false);
        assert_eq!(pathfinder.valid_path(&a, &d), false);
        assert_eq!(pathfinder.valid_path(&e, &b), false);
        assert_eq!(pathfinder.valid_path(&b, &e), false);

        assert_eq!(pathfinder.valid_path(&d, &e), true);
        assert_eq!(pathfinder.valid_path(&a, &c), true);
        assert_eq!(pathfinder.valid_path(&a, &g), true);

        assert_eq!(pathfinder.valid_path(&a, &f), false);
    }

    #[test]
    fn generate_graph() {
        let a = Point::new(40f32, 0f32, 0f32);
        let b = Point::new(40f32, 40f32, 0f32);
        let c = Point::new(0f32, 40f32, 0f32);
        let d = Point::new(0f32, 0f32, 0f32);
        let flyzones = vec![points_to_flyzone(vec![a, b, c, d])];
        let obstacles = vec![
            Obstacle::from_degrees(10f64, 20f64, 10f32, 10f32),
            Obstacle::from_degrees(30f64, 20f64, 10f32, 10f32),
        ];
        let mut pathfinder = Pathfinder::new();
        pathfinder.init(5f32, flyzones, obstacles);
    }

    #[test]
    fn same_radius_test() {
        let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());

        let n1 = Node::new(Point::new(30_f32, 30_f32, 0_f32), 1_f32, 0_f32);
        let n2 = Node::new(Point::new(20_f32, 20_f32, 0_f32), 1_f32, 0_f32);
        let a1 = Rc::new(n1);
        let b1 = Rc::new(n2);
        let expected = vec![
            (PI / 2_f32, PI / 2_f32, 200f32.sqrt()),
            (-PI / 2_f32, -PI / 2_f32, 200f32.sqrt()),
            (
                (2_f32 / (200_f32.sqrt())).acos(),
                PI + (2_f32 / (200_f32.sqrt())).acos(),
                14f32,
            ),
            (
                -(2_f32 / (200_f32.sqrt())).acos(),
                -(PI + (2_f32 / (200_f32.sqrt())).acos()),
                14f32,
            ),
        ];
        assert_eq!(pathfinder.find_path(&a1, &b1), expected);
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
        assert_eq!(pathfinder.find_path(&c, &d), expected);
    }

    #[test]
    fn different_radius_no_overlap_test() {
        let pathfinder = Pathfinder::create(1f32, dummy_flyzones(), Vec::new());
        let n5 = Node::new(Point::new(20_f32, 33_f32, 0_f32), 2_f32, 0_f32);
        let n6 = Node::new(Point::new(12_f32, 33_f32, 0_f32), 1_f32, 0_f32);
        let e = Rc::new(n5);
        let f = Rc::new(n6);
        let expected = [
            ((1_f32 / 8_f32).acos(), (1_f32 / 8_f32).acos(), 7f32.sqrt()),
            (
                -(1_f32 / 8_f32).acos(),
                -(1_f32 / 8_f32).acos(),
                7f32.sqrt(),
            ),
            (
                (3_f32 / 8_f32).acos(),
                PI + (3_f32 / 8_f32).acos(),
                55f32.sqrt(),
            ),
            (
                -(3_f32 / 8_f32).acos(),
                -(PI + (3_f32 / 8_f32).acos()),
                55f32.sqrt(),
            ),
        ];
        assert_eq!(pathfinder.find_path(&e, &f), expected);
    }
}
