// Visibility related code for modularity
use node::{Connection, Node, Vertex};
use obj::Location;
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
        let origin = self.find_origin();
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

        true
    }
}

// check if path is valid (not blocked by obstacle)
//	fn valid_path_obs(&self, a:&Point, b: &Point) -> bool {
//		for obstacle in &self.obstacles {
//			//catch the simple cases for now: if a or b are inside the radius of obstacle, invalid
//			if a.distance(&obstacle.coords) < obstacle.radius || b.distance(&obstacle.coords) < obstacle.radius {
//				return false
//			}
//			//reciprocals of dy and dx in terms of unit vector
//			let mag = a.distance(b) as f64;
//			let dx = -(a.lat() - b.lat()) / mag;
//			let dy = (a.lon() - b.lon()) / mag;
//			// connect two points from perpendicular to a to b segment, guarantee "intersect"
//			let mut c = Point::from_radians(obstacle.coords.lat() + dy * obstacle.radius as f64, obstacle.coords.lon() + dx * obstacle.radius as f64, obstacle.height);
//			let mut d = Point::from_radians(obstacle.coords.lat() - dy * obstacle.radius as f64, obstacle.coords.lon() - dx * obstacle.radius as f64, obstacle.height);
//			//math seems to check out here, successfully generates appropriate "perpendicular" line
//			//println!("Test intersect for {} {} {} {}", a, b, &c, &d);
//			if Self::intersect(a, b, &c, &d) == true {
//				return false
//			}
//		}
//		true
//	}

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

#[cfg(test)]
mod tests {
    use super::*;
    use obj::Obstacle;

    fn get_dummy_flyzones() -> Vec<Vec<Location>> {
        let a = Point::new(40f32, 0f32, 10f32);
        let b = Point::new(40f32, 40f32, 10f32);
        let c = Point::new(0f32, 40f32, 10f32);
        let d = Point::new(0f32, 0f32, 10f32);
        vec![points_to_flyzone(vec![a, b, c, d])]
    }

    fn points_to_flyzone(points: Vec<Point>) -> Vec<Location> {
        let origin = Location::from_radians(0f64, 0f64, 0f32);
        let mut flyzone = Vec::new();
        for point in points {
            flyzone.push(point.to_location(&origin));
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
        let pathfinder = Pathfinder::create(1f32, get_dummy_flyzones(), Vec::new());

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
        let pathfinder = Pathfinder::create(1f32, get_dummy_flyzones(), Vec::new());
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
        let pathfinder = Pathfinder::create(1f32, get_dummy_flyzones(), Vec::new());
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
