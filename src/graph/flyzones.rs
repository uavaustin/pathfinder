// flyzones.rs
// Contains all functions relating to virtual flyzone

use super::*;

impl<T> Pathfinder<T> {
    // Convert flyzone into virtual nodes
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
                    //println!("center: {:?}", center);
                    let virt_ob = Node::new(center, TURNING_RADIUS, 0f32);
                    self.nodes.push(Rc::new(RefCell::new(virt_ob)));
                }
            }
            iter += direction;
        }
    }

    // determines vertices of node and flyzone intersection
    pub fn insert_flyzone_sentinel(&mut self, node: &mut Node) {
        let center: Point = node.origin;
        let r: f32 = node.radius;
        for flyzone in self.flyzones.iter() {
            let size = flyzone.len();
            // iterate node over all vertices
            for i in 0..size {
                let mut v1 = flyzone[i];
                let mut v2 = flyzone[(i + 1) % size];
                let (x, y, dist_squared, end) = intersect_distance(
                    &Point::from_location(&v1, &self.origin),
                    &Point::from_location(&v2, &self.origin),
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
                let vertex_a = Rc::new(RefCell::new(Vertex::new_sentinel(
                    &mut self.num_vertices,
                    node,
                    a,
                )));
                let vertex_b = Rc::new(RefCell::new(Vertex::new_sentinel(
                    &mut self.num_vertices,
                    node,
                    b,
                )));

                let a_lat = vertex_a
                    .borrow()
                    .location
                    .to_location(&self.origin)
                    .lat_degree();
                let a_lon = vertex_a
                    .borrow()
                    .location
                    .to_location(&self.origin)
                    .lon_degree();
                let b_lat = vertex_b
                    .borrow()
                    .location
                    .to_location(&self.origin)
                    .lat_degree();
                let b_lon = vertex_b
                    .borrow()
                    .location
                    .to_location(&self.origin)
                    .lon_degree();
                println!("flyzone/node vertices: {:?},{:?}", a_lat, a_lon);
                println!("flyzone/node vertices: {:?},{:?}", b_lat, b_lon);
                node.insert_vertex(vertex_a);
                node.insert_vertex(vertex_b);
            }
        }
    }
}
