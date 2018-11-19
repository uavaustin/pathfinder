// Utility functions to help with graph calculations

use super::*;

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
// returns x, y of intersection, distance, and whether intersection is at endpoint
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

#[cfg(test)]
mod test {
    use super::*;

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
