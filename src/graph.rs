// Visibility related code for modularity
use ::{Pathfinder, PI};
use node::{Connection, Node, Vertex};
use obj::{Point};
use std::rc::Rc;

impl Pathfinder {
    // calculate distance of shortest distance from point to a segment defined by two lines

    // Generate all possible path (tangent lines) between two nodes, and return the
    // shortest valid path if one exists

    pub fn find_path(&self, a: &Rc<Node>, b: &Rc<Node>) -> Vec<Connection> {
        let c1: Point = a.location;
        let c2: Point = b.location;
        let r1: f32 = a.radius.into();
        let r2: f32 = b.radius.into();
        let dist: f32 = (((c1.lat() - c2.lat()).powi(2) + (c1.lon() - c2.lon()).powi(2)).sqrt()) as f32;
        //1 and 2 outer, 3 and 4 inner
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
            candidates = vec![(theta1, phi1), (theta2, phi2), (theta3, phi3), (theta4, phi4)];
        } else {
            candidates = vec![(theta1, phi1), (theta2, phi2)];
        }
        let mut connections: Vec<Connection> = Vec::new();
        
        for (i, j) in candidates.iter() {
            let v1 = Vertex::new(a.clone(), *i);
            let v2 = Vertex::new(b.clone(), *j);
            let p1 = v1.to_point();
            let p2 = v2.to_point();
            if self.valid_path(&p1, &p2) {
                connections.push(Connection::new(Rc::new(v1), Rc::new(v2), p1.distance(&p2)));
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
            let first = tempzone.remove(0);
            let mut temp = first;
            for point in tempzone {
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
        // test for obstacles
        for obstacle in &self.obstacles {
            //catch the simple cases for now: if a or b are inside the radius of obstacle, invalid
            if a.distance(&obstacle.coords) < obstacle.radius || b.distance(&obstacle.coords) < obstacle.radius {
                return false
            }
            //reciprocals of dy and dx in terms of unit vector
            let mag = a.distance(b) as f64;
            let dx = -(a.lat() - b.lat()) / mag;
            let dy = (a.lon() - b.lon()) / mag;
            // connect two points from perpendicular to a to b segment, guarantee "intersect"
            let mut c = Point::from_radians(obstacle.coords.lat() + dy * obstacle.radius as f64, obstacle.coords.lon() + dx * obstacle.radius as f64, obstacle.height);
            let mut d = Point::from_radians(obstacle.coords.lat() - dy * obstacle.radius as f64, obstacle.coords.lon() - dx * obstacle.radius as f64, obstacle.height);
            //math seems to check out here, successfully generates appropriate "perpendicular" line
            //println!("Test intersect for {} {} {} {}", a, b, &c, &d);
            if intersect(a, b, &c, &d) == true {
                return false
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
fn area(a: &Point, b: &Point, c: &Point) -> f64 {
    (b.lon() - a.lon()) * (c.lat() - a.lat()) - (c.lon() - a.lon()) * (b.lat() - a.lat())
}

// helper function for intersection calculation
// returns true if point c is between a and b, false otherwise
fn between(a: &Point, b: &Point, c: &Point) -> bool {
    if a.lon() != b.lon() {
        (a.lon() <= c.lon() && c.lon() <= b.lon()) || (a.lon() >= c.lon() && c.lon() >= b.lon())
    } else {
        (a.lat() <= c.lat() && c.lat() <= b.lat()) || (a.lat() >= c.lat() && c.lat() >= b.lat())
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
    if a1 == 0f64 {
        // checks if c is between a and b OR
        // d is colinear also AND between a and b or at opposite ends?
        if between(a, b, c) {
            return true;
        } else {
            if area(a, b, d) == 0f64 {
                return between(c, d, a) || between(c, d, b);
            } else {
                return false;
            }
        }
    } else if a2 == 0f64 {
        // check if d is between a and b since c is not colinear
        return between(a, b, d);
    }
    if a3 == 0f64 {
        // checks if a is between c and d OR
        // b is colinear AND either between a and b or at opposite ends?
        if between(c, d, a) {
            return true;
        } else {
            if area(c, d, b) == 0f64 {
                return between(a, b, c) || between(a, b, d);
            } else {
                return false;
            }
        }
    } else if a4 == 0f64 {
        // check if b is between c and d since we know a is not colinear
        return between(c, d, b);
    }
    //tests for regular intersection
    else {
        ((a1 > 0f64) ^ (a2 > 0f64)) && ((a3 > 0f64) ^ (a4 > 0f64))
    }
}

#[test]
fn is_between() {
    let a = Point::from_radians(40f64, 40f64, 10f32);
    let b = Point::from_radians(40f64, 50f64, 10f32);
    let c = Point::from_radians(40f64, 60f64, 10f32);
    assert_eq!(between(&a, &c, &b), true);
    assert_eq!(between(&a, &b, &c), false);
}

#[test]
fn is_colinear() {
    let a = Point::from_radians(40f64, 40f64, 10f32);
    let b = Point::from_radians(40f64, 50f64, 10f32);
    let c = Point::from_radians(40f64, 60f64, 10f32);
    assert_eq!(area(&a, &b, &c), 0f64);
}

#[test]
fn yes_intersect() {
    let a = Point::from_radians(40f64, 0f64, 10f32);
    let b = Point::from_radians(40f64, 40f64, 10f32);
    let c = Point::from_radians(0f64, 0f64, 10f32);
    let d = Point::from_radians(0f64, 40f64, 10f32);
    assert_eq!(intersect(&a, &d, &b, &c), true);
}

#[test]
fn no_intersect() {
    let a = Point::from_radians(40f64, 0f64, 10f32);
    let b = Point::from_radians(40f64, 40f64, 10f32);
    let c = Point::from_radians(0f64, 0f64, 10f32);
    let d = Point::from_radians(0f64, 40f64, 10f32);
    assert_eq!(intersect(&a, &c, &b, &d), false);
    assert_eq!(intersect(&c, &d, &a, &b), false);
}

#[test]
fn special_intersect() {
    let a = Point::from_radians(0f64, 0f64, 10f32);
    let b = Point::from_radians(10f64, 5f64, 10f32);
    let c = Point::from_radians(20f64, 10f64, 10f32);
    let d = Point::from_radians(30f64, 15f64, 10f32);
    assert_eq!(intersect(&a, &b, &c, &d), false);
    assert_eq!(intersect(&a, &c, &b, &d), true);
}

#[test]
fn flyzone_pathing() {
    let a = Point::from_radians(40f64, 0f64, 10f32);
    let b = Point::from_radians(40f64, 40f64, 10f32);
    let c = Point::from_radians(0f64, 0f64, 10f32);
    let d = Point::from_radians(0f64, 40f64, 10f32);
    let flyzone = vec![a, b, d, c];
    let flyzones = vec![flyzone];

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(1f32, flyzones, Vec::new());

    let e = Point::from_radians(20f64, 20f64, 10f32);
    let f = Point::from_radians(30f64, 30f64, 10f32);
    let g = Point::from_radians(20f64, 50f64, 10f32);

    let h = Point::from_radians(50f64, 50f64, 10f32);
    let i = Point::from_radians(50f64, 0f64, 10f32);

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
    let a = Point::from_radians(40f64, 0f64, 10f32);
    let b = Point::from_radians(40f64, 40f64, 10f32);
    let c = Point::from_radians(0f64, 0f64, 10f32);
    let d = Point::from_radians(0f64, 40f64, 10f32);

    let e = Point::from_radians(30f64, 10f64, 10f32);
    let f = Point::from_radians(30f64, 30f64, 10f32);
    let g = Point::from_radians(10f64, 10f64, 10f32);
    let h = Point::from_radians(10f64, 30f64, 10f32);

    let flyzone1 = vec![a, b, d, c];
    let flyzone2 = vec![e, f, h, g];

    let flyzones = vec![flyzone1, flyzone2];

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(1f32, flyzones, Vec::new());

    let i = Point::from_radians(15f64, 15f64, 10f32);
    let j = Point::from_radians(25f64, 25f64, 10f32);
    let k = Point::from_radians(35f64, 5f64, 10f32);
    let l = Point::from_radians(50f64, 50f64, 10f32);
    let m = Point::from_radians(35f64, 25f64, 10f32);

    assert_eq!(pathfinder.valid_path(&i, &j), true);
    assert_eq!(pathfinder.valid_path(&i, &k), false);
    assert_eq!(pathfinder.valid_path(&i, &l), false);
    assert_eq!(pathfinder.valid_path(&k, &l), false);
    assert_eq!(pathfinder.valid_path(&k, &m), true);
}

#[test]
fn find_path_tests() {
    let a = Point::from_radians(40f64, 0f64, 10f32);
    let b = Point::from_radians(40f64, 40f64, 10f32);
    let c = Point::from_radians(0f64, 0f64, 10f32);
    let d = Point::from_radians(0f64, 40f64, 10f32);
    let flyzone = vec![a, b, d, c];
    let flyzones = vec![flyzone];
    let mut pathfinder = Pathfinder::new();
    pathfinder.init(1f32, flyzones, Vec::new());

    let n1 = Node::new(0_u32, Point::from_radians(30_f64, 30_f64, 0_f32), 1_f32, 0_f32);
    let n2 = Node::new(1_u32, Point::from_radians(20_f64, 20_f64, 0_f32), 1_f32, 0_f32);
    let a1 = Rc::new(n1);
    let b1 = Rc::new(n2);
    let mut expected = Vec::new();
    let candidates = vec![(PI / 2_f32, PI / 2_f32),
                (-PI / 2_f32, -PI / 2_f32),
                ((2_f32 / (200_f32.sqrt())).acos(), PI + (2_f32 / (200_f32.sqrt())).acos()),
                (-(2_f32 / (200_f32.sqrt())).acos(), -(PI + (2_f32 / (200_f32.sqrt())).acos()))];
    for (i, j) in candidates.iter() {
        let v1 = Vertex::new(a1.clone(), *i);
        let v2 = Vertex::new(b1.clone(), *j);
        let p1 = v1.to_point();
        let p2 = v2.to_point();
        expected.push(Connection::new(Rc::new(v1), Rc::new(v2), p1.distance(&p2)));
    }
    assert_eq!(pathfinder.find_path(&a1, &b1), expected);



    let n3 = Node::new(0_u32, Point::from_radians(15_f64, 10_f64, 0_f32), 5_f32, 0_f32);
    let n4 = Node::new(1_u32, Point::from_radians(20_f64, 10_f64, 0_f32), 4_f32, 0_f32);
    let c = Rc::new(n3);
    let d = Rc::new(n4);
    let mut expected = Vec::new();
    let candidates = vec![((1_f32/5_f32).acos(), (1_f32/5_f32).acos()),
                (-(1_f32/5_f32).acos(), -(1_f32/5_f32).acos())];
    for (i, j) in candidates.iter() {
        let v1 = Vertex::new(c.clone(), *i);
        let v2 = Vertex::new(d.clone(), *j);
        let p1 = v1.to_point();
        let p2 = v2.to_point();
        expected.push(Connection::new(Rc::new(v1), Rc::new(v2), p1.distance(&p2)));
    }
    assert_eq!(pathfinder.find_path(&c, &d), expected);



    let n5 = Node::new(0_u32, Point::from_radians(20_f64, 33_f64, 0_f32), 2_f32, 0_f32);
    let n6 = Node::new(1_u32, Point::from_radians(12_f64, 33_f64, 0_f32), 1_f32, 0_f32);
    let e = Rc::new(n5);
    let f = Rc::new(n6);
    let mut expected = Vec::new();
    let candidates = [((1_f32/8_f32).acos(), (1_f32/8_f32).acos()),
                (-(1_f32/8_f32).acos(), -(1_f32/8_f32).acos()),
                ((3_f32/8_f32).acos(), PI + (3_f32/8_f32).acos()),
                (-(3_f32/8_f32).acos(), -(PI + (3_f32/8_f32).acos()))];
    for (i, j) in candidates.iter() {
        let v1 = Vertex::new(e.clone(), *i);
        let v2 = Vertex::new(f.clone(), *j);
        let p1 = v1.to_point();
        let p2 = v2.to_point();
        expected.push(Connection::new(Rc::new(v1), Rc::new(v2), p1.distance(&p2)));
    }
    assert_eq!(pathfinder.find_path(&e, &f), expected);

}