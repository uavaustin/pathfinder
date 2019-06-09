// flyzones.rs
// Contains all functions relating to virtual flyzone

use super::*;

// determine if set of order points is clockwise, c-clockwise, or straight
// input vector of points, output (direction, straight)
fn vertex_direction(points: &[Point]) -> (bool, bool) {
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

impl Tanstar {
    // Find origin (lower left corner) of a flyzone
    pub(in tanstar) fn find_origin(flyzones: &[Vec<Location>]) -> Location {
        const MAX_RADIAN: f64 = 2f64 * ::std::f64::consts::PI;
        let mut min_lat = MAX_RADIAN;
        let mut min_lon = MAX_RADIAN;
        let mut max_lon = 0f64;
        let mut lon = min_lon;

        assert!(!flyzones.is_empty(), "Require at least one flyzone");
        for flyzone in flyzones {
            assert!(
                flyzone.len() > 2,
                "Require at least 3 points to construct fly zone."
            );

            for point in flyzone {
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

        println!(
            "Found origin: {}, {}",
            min_lat.to_degrees(),
            lon.to_degrees()
        );
        Location::from_radians(min_lat, lon, 0f32)
    }

    // Convert flyzone into virtual nodes
    pub(in tanstar::graph) fn virtualize_flyzone(&mut self, index: usize) {
        let flyzone = &self.flyzones[index];
        // convert flyzone to points
        let mut flyzone_points = Vec::new();
        for location in flyzone {
            let point = Point::from((location, &self.origin));
            flyzone_points.push(point);
        }
        // determine flyzone directions
        let size = flyzone.len() as isize - 1;
        let (clockwise, _) = vertex_direction(&flyzone_points);
        let (direction, mut iter): (isize, isize) = if clockwise { (1, 0) } else { (-1, size) };

        // edge conditions for flyzone
        while (iter <= size) && (iter > -1) {
            let (prev, next) = if iter == 0 {
                (size, iter + 1)
            } else if iter == size {
                (iter - 1, 0)
            } else {
                (iter - 1, iter + 1)
            };
            // initalize obstacle location
            let a = flyzone_points[prev as usize];
            let vertex = flyzone_points[iter as usize];
            let b = flyzone_points[next as usize];
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
            let (iter_clockwise, straight) = vertex_direction(&[a, vertex, b]);
            // straight line condition
            if !straight {
                let d =
                    if (!iter_clockwise && direction == 1) || (iter_clockwise && direction == -1) {
                        self.config.turning_radius
                    } else {
                        self.config.turning_radius / ((theta / 2f32).sin())
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
                    //println!("center: {:?}", center);
                    let virt_ob = Node::new(center, self.config.turning_radius, 0f32);
                    self.nodes.push(Wrapper::new(virt_ob));
                }
            }
            iter += direction;
        }
    }

    // determines vertices of node and flyzone intersection
    #[allow(clippy::many_single_char_names)]
    pub(in tanstar::graph) fn insert_flyzone_sentinel(&mut self, node: &mut Node) {
        let center: Point = node.origin;
        let r: f32 = node.radius;
        for flyzone in self.flyzones.iter() {
            let size = flyzone.len();
            // iterate node over all vertices
            for i in 0..size {
                let mut v1 = flyzone[i];
                let mut v2 = flyzone[(i + 1) % size];
                let (x, y, dist_squared, end) = intersect_distance(
                    &Point::from((&v1, &self.origin)),
                    &Point::from((&v2, &self.origin)),
                    &center,
                );
                let dist = dist_squared.sqrt();
                // println!("dist: {:?}",dist);
                // check intersect is true
                if dist > r {
                    continue;
                }
                // determine both intersect angles in left and right ring
                let theta = (dist / r).acos(); //check
                let dx = x - center.x; //check
                let dy = y - center.y; //check
                let phi = dy.atan2(dx); //check
                let a = phi + theta;
                let b = phi - theta;
                let vertex_a = Wrapper::new(Vertex::new_sentinel(&mut self.num_vertices, node, a));
                let vertex_b = Wrapper::new(Vertex::new_sentinel(&mut self.num_vertices, node, b));

                {
                    let v_a = vertex_a.lock();
                    let v_b = vertex_b.lock();
                    let a = Location::from((&v_a.borrow().location, &self.origin));
                    let b = Location::from((&v_b.borrow().location, &self.origin));
                    println!("flyzone/node vertices: {:?},{:?}", a.lat(), a.lon());
                    println!("flyzone/node vertices: {:?},{:?}", b.lat(), b.lon());
                }
                node.insert_vertex(vertex_a);
                node.insert_vertex(vertex_b);
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

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
