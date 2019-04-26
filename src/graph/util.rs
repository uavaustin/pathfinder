// Utility functions to help with graph calculations

use super::*;

// helper function to remap angle between range
// range can be either -2PI to 0 or 0 to 2PI
// Input:
// positive is true if range is between 0 to 2PI, -2PI to 0 otherwise
// n is angle in radians
pub fn normalize_angle(positive: bool, n: f32) -> f32 {
    let angle = (n - (n / (2f32*PI)).floor() * (2f32 * PI)).abs();
    if positive {
        angle
    } else {
        angle - 2f32*PI
    }
}

// Reverse the polarity of an angle
// Left side angle is converted to the equivilent right side angle and vice versa
pub fn reverse_polarity(alpha: f32) -> f32 {
    if alpha < 0f32 {
        alpha + 2f32*PI
    } else {
        alpha - 2f32*PI
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
pub fn intersect(a: &Point, b: &Point, c: &Point, d: &Point) -> bool {
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
// returns x, y of intersection, distance SQUARED, and whether intersection is at endpoint
pub fn intersect_distance(a: &Point, b: &Point, c: &Point) -> (f32, f32, f32, bool) {
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
    (x, y, (x - c.x).powi(2) + (y - c.y).powi(2), endpoint)
}
// determine if set of order points is clockwise, c-clockwise, or straight
// input vector of points, output (direction, straight)
pub fn vertex_direction(points: &Vec<Point>) -> (bool, bool) {
    let mut sum_whole = 0f32;
    for i in 0..points.len() {
        let first = points[i];
        let second = if i + 1 == points.len() {
            points[0]
        } else {
            points[i + 1]
        };
        sum_whole += (second.x - first.x) * (second.y + first.y);
    }
    if sum_whole > 0f32 {
        (true, false) // clockwise
    } else if sum_whole < 0f32 {
        (false, false) // counter-clockwise
    } else {
        (false, true) // straight-line
    }
}

fn output_ring(origin: &Location, mut current: Rc<RefCell<Vertex>>) {
    let temp = match current.borrow().next {
        Some(ref v) => v.clone(),
        None => panic!("Next points to null"),
    };
    current = temp;
    loop {
        let ref mut vertex = current.clone();
        if vertex.borrow().index != HEADER_VERTEX_INDEX {
            let v_loc = vertex.borrow().location.to_location(origin);
            println!("{}, {}", v_loc.lat_degree(), v_loc.lon_degree());
        } else {
            break;
        }
        current = match vertex.borrow().next {
            Some(ref v) => v.clone(),
            None => panic!("Next points to null"),
        };
    }
}

// Debug method to output vertices
pub fn output_graph(finder: &Pathfinder) {
    println!("\n------------------------------");
    println!("pathfinder graph");
    println!("node count: {}", finder.nodes.len());
    println!("vertex count: {}\n", finder.num_vertices);
    println!("---- Node List ----");
    for node in &finder.nodes {
        let loc = node.borrow().origin.to_location(&finder.origin);
        println!("{}, {}", loc.lat_degree(), loc.lon_degree());
    }
    println!("\n---- Left Vertex List ----");

    for node in &finder.nodes {
        let loc = node.borrow().origin.to_location(&finder.origin);
        // let loc = node.borrow().origin;
        // println!("{}, {}", loc.x, loc.y);
        if node.borrow().height > 0f32 {
            output_ring(&finder.origin, node.borrow().left_ring.clone());
        }
    }

    println!("\n---- Right Vertex List ----");

    for node in &finder.nodes {
        let loc = node.borrow().origin.to_location(&finder.origin);
        // let loc = node.borrow().origin;
        // println!("{}, {}", loc.x, loc.y);
        if node.borrow().height > 0f32 {
            output_ring(&finder.origin, node.borrow().right_ring.clone());
        }
    }

    println!("------------------------------");
}

#[cfg(test)]
mod test {
    use super::*;

    const THRESHOLD: f32 = 0.001;

    //assert equa for float
    macro_rules! assert_eqf {
        ($x:expr, $y:expr) => {
            if !((($x - $y) as f32).abs() < THRESHOLD) {
                //println!("{} vs {}", $x, $y);
                panic!();
            }
        };
    }

    #[test]
    fn normalize_angle_test() {
        assert_eqf!(normalize_angle(true, 3f32*PI), PI);
        assert_eqf!(normalize_angle(false, 3f32*PI), -PI);
        assert_eqf!(normalize_angle(true, -3f32*PI), PI);
        assert_eqf!(normalize_angle(false, -3f32*PI), -PI);
        assert_eqf!(normalize_angle(true, 3f32*PI/2f32), 3f32*PI/2f32);
        assert_eqf!(normalize_angle(false, 3f32*PI/2f32), -PI/2f32);
        assert_eqf!(normalize_angle(true, -3f32*PI/2f32), PI/2f32);
        assert_eqf!(normalize_angle(false, -3f32*PI/2f32), -3f32*PI/2f32);
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
    fn vertex_direction_test() {
        let a = Point::new(0f32, 0f32, 10f32);
        let b = Point::new(0f32, 10f32, 10f32);
        let c = Point::new(10f32, 10f32, 10f32);
        let d = Point::new(10f32, 0f32, 10f32);
        let e = Point::new(0f32, 20f32, 10f32);
        let clockwise_flyzone = vec![a, b, c, d];
        let anticlockwise_flyzone = vec![d, c, b, a];
        let line_flyzone = vec![a, b, e];
        assert_eq!(vertex_direction(&clockwise_flyzone), (true, false));
        assert_eq!(vertex_direction(&anticlockwise_flyzone), (false, false));
        assert_eq!(vertex_direction(&line_flyzone), (false, true));
    }
}
