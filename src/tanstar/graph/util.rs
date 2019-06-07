// util.rs
// Contains general utility functions to help with graph calculations

use super::*;

impl Tanstar {
    // Helper function to create and init tanstar object
    pub fn create(
        buffer_size: f32,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        let mut pathfinder = Self::new();
        let mut config = TConfig::default();
        config.buffer_size = buffer_size;
        pathfinder.init(config, flyzones, obstacles);
        pathfinder
    }
}

// helper function to remap angle between range
// range can be either -2PI to 0 or 0 to 2PI
// Input:
// positive is true if range is between 0 to 2PI, -2PI to 0 otherwise
// n is angle in radians
pub fn normalize_angle(positive: bool, n: f32) -> f32 {
    let angle = (n - (n / (2f32 * PI)).floor() * (2f32 * PI)).abs();
    if positive {
        angle
    } else {
        angle - 2f32 * PI
    }
}

// Reverse the polarity of an angle
// Left side angle is converted to the equivilent right side angle and vice versa
pub fn reverse_polarity(alpha: f32) -> f32 {
    if alpha < 0f32 {
        alpha + 2f32 * PI
    } else {
        alpha - 2f32 * PI
    }
}

// Calculate the arc length from angle a to angle b on a circle of radius r
pub fn arc_length(a: f32, b: f32, r: f32) -> f32 {
    let mut angle = if a >= 0f32 {
        // Left case
        b - a
    } else {
        a - b
    };

    if angle < 0f32 {
        angle += 2f32 * PI;
    }

    (angle * r)
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

// calculate distance of shortest distance from point c to a segment defined by a and b
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

fn output_ring(origin: &Location, mut current: Arc<RefCell<Vertex>>) {
    let temp = match current.borrow().next {
        Some(ref v) => v.clone(),
        None => panic!("Next points to null"),
    };
    current = temp;
    let mut count = 0;
    loop {
        let ref mut vertex = current.clone();
        if vertex.borrow().index != HEADER_VERTEX_INDEX {
            count += 1;
            let v_loc = Location::from((&vertex.borrow().location, origin));
            println!("{}, {}", v_loc.lat_degree(), v_loc.lon_degree());
        } else {
            break;
        }
        current = match vertex.borrow().next {
            Some(ref v) => v.clone(),
            None => panic!("Next points to null"),
        };
    }
    // println!("Node vertex count {}", count);
}

// Debug method to output vertices
pub fn output_graph(finder: &Tanstar) {
    println!("\n------------------------------");
    println!("pathfinder graph");
    println!("node count: {}", finder.nodes.len());
    println!("vertex count: {}\n", finder.num_vertices);
    println!("---- Node List ----");
    // for node in &finder.nodes {
    //     let loc = Location::from((&node.borrow().origin, &finder.origin));
    //     println!("{}, {}", loc.lat_degree(), loc.lon_degree());
    // }
    //println!("\n---- Left Vertex List ----");

    for node in &finder.nodes {
        // let loc = Location::from((&node.borrow().origin, &finder.origin));
        // println!("Node origin {:?}", node.borrow().origin);
        if node.borrow().height > 0f32 {
            output_ring(&finder.origin, node.borrow().left_ring.clone());
        }
    }

    //println!("\n---- Right Vertex List ----");

    for node in &finder.nodes {
        // let loc = Location::from((&node.borrow().origin, &finder.origin));
        // let loc = node.borrow().origin;
        // println!("Node origin {:?}", node.borrow().origin);
        if node.borrow().height > 0f32 {
            output_ring(&finder.origin, node.borrow().right_ring.clone());
        }
    }

    println!("------------------------------");
}

// find the intersection of line ab with obstacle c, if they exist
pub fn perpendicular_intersect(
    origin: &Location,
    a: &Point,
    b: &Point,
    c: &Obstacle,
) -> (Option<Point>, Option<Point>) {
    // intersect distance gives x and y of intersect point, then distance squared
    // calculates the shortest distance between the segment and obstacle. If less than radius, it intersects.
    // #TODO: endpoint not used, why is it here?
    let (x, y, distance, _endpoint) = intersect_distance(a, b, &Point::from((&c.location, origin)));
    if distance.sqrt() < c.radius as f32 {
        println!(
            "intersect with obstacle: dist {} r {}",
            distance.sqrt(),
            c.radius
        );
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
// #TODO: benchmark versus perpendicular_intersect
#[allow(dead_code)]
pub fn circular_intersect(
    origin: &Location,
    a: &Point,
    b: &Point,
    obstacle: &Obstacle,
) -> (Option<Point>, Option<Point>) {
    //y = mx + b for point a and b

    let mut c = Point::from((&obstacle.location, origin));
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
        assert_eqf!(normalize_angle(true, 3f32 * PI), PI);
        assert_eqf!(normalize_angle(false, 3f32 * PI), -PI);
        assert_eqf!(normalize_angle(true, -3f32 * PI), PI);
        assert_eqf!(normalize_angle(false, -3f32 * PI), -PI);
        assert_eqf!(normalize_angle(true, 3f32 * PI / 2f32), 3f32 * PI / 2f32);
        assert_eqf!(normalize_angle(false, 3f32 * PI / 2f32), -PI / 2f32);
        assert_eqf!(normalize_angle(true, -3f32 * PI / 2f32), PI / 2f32);
        assert_eqf!(normalize_angle(false, -3f32 * PI / 2f32), -3f32 * PI / 2f32);
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
}
