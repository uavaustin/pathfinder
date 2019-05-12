use super::*;

use std::fmt;

impl fmt::Display for Node {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "loc={:?}, r={} \nleft = [", self.origin, self.radius)
            .expect("error in writing format display");
        let mut current = match self.left_ring.borrow().next {
            Some(ref val) => val.clone(),
            None => panic!("next points to null"),
        };
        while current.borrow().index != HEADER_VERTEX_INDEX {
            let ref mut vertex = current.clone();
            write!(f, " {} ", vertex.borrow()).expect("error in writing format display");
            current = match vertex.borrow().next {
                Some(ref val) => val.clone(),
                None => panic!("next points to null"),
            };
        }
        write!(f, " ] \nright = [").expect("error in writing format display");
        let mut current = match self.right_ring.borrow().next {
            Some(ref val) => val.clone(),
            None => panic!("next points to null"),
        };
        while current.borrow().index != HEADER_VERTEX_INDEX {
            let ref mut vertex = current.clone();
            write!(f, " {} ", vertex.borrow()).expect("error in writing format display");
            current = match vertex.borrow().next {
                Some(ref val) => val.clone(),
                None => panic!("next points to null"),
            };
        }
        write!(f, "]").expect("error in writing format display");
        Ok(())
    }
}

impl Node {
    pub fn new(origin: Point, radius: f32, height: f32) -> Self {
        let left_head = Rc::new(RefCell::new(Vertex::new_head(
            &mut HEADER_VERTEX_INDEX,
            origin,
        )));
        left_head.borrow_mut().next = Some(left_head.clone());
        left_head.borrow_mut().prev = Some(left_head.clone());
        let right_head = Rc::new(RefCell::new(Vertex::new_head(
            &mut HEADER_VERTEX_INDEX,
            origin,
        )));
        right_head.borrow_mut().next = Some(right_head.clone());
        right_head.borrow_mut().prev = Some(right_head.clone());
        Node {
            origin: origin,
            radius: radius,
            height: height,
            left_ring: left_head,
            right_ring: right_head,
        }
    }

    // Generate node from obstacle
    pub fn from_obstacle(obs: &Obstacle, origin: &Location, buffer: f32) -> Self {
        Node::new(
            Point::from_location(&obs.location, origin),
            obs.radius + buffer,
            obs.height,
        )
    }

    // Generate node from point, used for inserting virtual obstacles for flyzones
    pub fn from_location(p: &Location, origin: &Location) -> Self {
        Node::new(Point::from_location(p, origin), TURNING_RADIUS, 0f32)
    }

    // Generate node from plane
    pub fn from_plane(plane: &Plane, origin: &Location) -> Self {
        Node::new(
            Point::from_location(&plane.location, origin),
            TURNING_RADIUS,
            plane.location.alt(),
        )
    }

    // Generate node from waypoint
    pub fn from_waypoint(waypoint: &Waypoint, origin: &Location) -> Self {
        Node::new(
            Point::from_location(&waypoint.location, origin),
            waypoint.radius,
            0f32,
        )
    }

    // Converts a vertex on a node to coordinate
    pub fn to_point(&self, angle: f32) -> Point {
        Point::new(
            self.origin.x + (self.radius * angle.cos()),
            self.origin.y + (self.radius * angle.sin()),
            self.height,
        )
    }

    // Traverse ring to find current pointer and next pointer for angle
    fn traverse_rings(&self, angle: f32) -> (Rc<RefCell<Vertex>>, Rc<RefCell<Vertex>>) {
        let (is_left, mut current) = if angle >= 0f32 {
            // Left ring
            (true, self.left_ring.clone())
        } else {
            // Right ring
            (false, self.right_ring.clone())
        };

        loop {
            // Get index, angle, and pointer to the next vertex
            let (next_index, next_angle, next) = match current.borrow().next {
                Some(ref next) => (next.borrow().index, next.borrow().angle, next.clone()),
                None => panic!("Next points to null"),
            };

            if (next_index == HEADER_VERTEX_INDEX)       // Reached end of chain
                || (is_left && angle < next_angle)   // Found furthest possible vertex on left
                || (!is_left && angle > next_angle)
            // Found furthest possible vertex on right
            {
                return (current, next);
            }
            current = next;
        }
    }

    // Find a vertex within threshold to the angle and return it
    // if not found, create new one
    pub fn get_vertex(&self, num_vertices: &mut i32, angle: f32) -> Rc<RefCell<Vertex>> {
        println!("Looking for vertex with angle {}", angle);
        let (current, next) = self.traverse_rings(angle);

        let cur_index = current.borrow().index;
        let next_index = next.borrow().index;
        // Ensure current and next are different vertices
        if cur_index != next_index {
            let mut temp_current = current.clone();
            let mut temp_next = next.clone();
            if cur_index == HEADER_VERTEX_INDEX || next_index == HEADER_VERTEX_INDEX {
                if cur_index == HEADER_VERTEX_INDEX {
                    temp_current = match current.borrow().prev {
                        Some(ref prev) => prev.clone(),
                        None => panic!("broken chain"),
                    };
                } else {
                    temp_next = match next.borrow().next {
                        Some(ref next) => next.clone(),
                        None => panic!("broken chain"),
                    };
                }
            }
            let arc_a = arc_length(temp_current.borrow().angle, angle, self.radius);
            let arc_b = arc_length(angle, temp_next.borrow().angle, self.radius);
            let min_arc = if arc_a < arc_b { arc_a } else { arc_b };
            if min_arc < VERTEX_MERGE_THRESHOLD {
                println!(
                    "found existing vertex {} and {} with dist {}",
                    current.borrow(),
                    next.borrow(),
                    min_arc
                );

                return if arc_a < arc_b { temp_current } else { temp_next };
            }
        }

        println!(
            "insert new vertex {} between {} and {}",
            *num_vertices + 1,
            current.borrow().index,
            next.borrow().index
        );
        let v = Rc::new(RefCell::new(Vertex::new(num_vertices, self, angle, vec![])));
        next.borrow_mut().prev = Some(v.clone());
        v.borrow_mut().next = Some(next);
        v.borrow_mut().prev = Some(current.clone());
        current.borrow_mut().next = Some(v.clone());

        v
    }

    // Insert an existing vertex into a node
    pub fn insert_vertex(&mut self, v: Rc<RefCell<Vertex>>) {
        let (current, next) = self.traverse_rings(v.borrow().angle);
        next.borrow_mut().prev = Some(v.clone());
        v.borrow_mut().next = Some(next);
        v.borrow_mut().prev = Some(current.clone());
        current.borrow_mut().next = Some(v.clone());
    }

    pub fn prune_vertices(to_remove: LinkedList<Rc<RefCell<Vertex>>>) {
        for v in to_remove.iter() {
            let prev = match v.borrow_mut().prev {
                Some(ref vert) => vert.clone(),
                None => panic!("Next points to null"),
            };
            let next = match v.borrow_mut().next {
                Some(ref vert) => vert.clone(),
                None => panic!("Next points to null"),
            };
            prev.borrow_mut().next = Some(next.clone());
            next.borrow_mut().prev = Some(prev.clone());
        }
    }
}
