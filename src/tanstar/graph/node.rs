use super::*;

use std::fmt;

#[derive(Debug)]
pub struct Node {
    pub origin: Point,
    pub radius: f32,
    pub height: f32,                 // make private later
    pub left_ring: Wrapper<Vertex>,  // make private later
    pub right_ring: Wrapper<Vertex>, // make private later
}

impl fmt::Display for Node {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "loc={:?}, r={} \nleft = [", self.origin, self.radius)
            .expect("error in writing format display");
        let left_ring = self.left_ring.lock();
        let mut current = match left_ring.borrow().next {
            Some(ref val) => val.clone(),
            None => panic!("next points to null"),
        };
        loop {
            let _current;
            {
                let vertex = current.lock();
                if vertex.borrow().index != HEADER_VERTEX_INDEX {
                    break;
                }
                write!(f, " {} ", vertex.borrow()).expect("error in writing format display");
                _current = match vertex.borrow().next {
                    Some(ref val) => val.clone(),
                    None => panic!("next points to null"),
                };
            }
            current = _current;
        }
        write!(f, " ] \nright = [").expect("error in writing format display");
        let right_ring = self.right_ring.lock();
        let mut current = match right_ring.borrow().next {
            Some(ref val) => val.clone(),
            None => panic!("next points to null"),
        };
        loop {
            let _current;
            {
                let vertex = current.lock();
                if vertex.borrow().index != HEADER_VERTEX_INDEX {
                    break;
                }
                write!(f, " {} ", vertex.borrow()).expect("error in writing format display");
                _current = match vertex.borrow().next {
                    Some(ref val) => val.clone(),
                    None => panic!("next points to null"),
                };
            }
            current = _current;
        }
        write!(f, "]").expect("error in writing format display");
        Ok(())
    }
}

impl From<(&Obstacle, &Location, f32)> for Node {
    // Generate node from obstacle
    fn from((obs, origin, buffer): (&Obstacle, &Location, f32)) -> Self {
        Self::new(
            Point::from((&obs.location, origin)),
            obs.radius + buffer,
            obs.height,
        )
    }
}

impl From<(&Location, &Location, f32)> for Node {
    // Generate node from point, used for inserting virtual obstacles for flyzones
    fn from((p, origin, turning_radius): (&Location, &Location, f32)) -> Self {
        Self::new(Point::from((p, origin)), turning_radius, 0f32)
    }
}

impl From<(&Plane, &Location, f32)> for Node {
    // Generate node from plane
    fn from((plane, origin, turning_radius): (&Plane, &Location, f32)) -> Self {
        Self::new(
            Point::from((&plane.location, origin)),
            turning_radius,
            plane.location.alt(),
        )
    }
}

impl<T> From<(&Waypoint<T>, &Location)> for Node {
    // Generate node from waypoint
    fn from((waypoint, origin): (&Waypoint<T>, &Location)) -> Self {
        Self::new(
            Point::from((&waypoint.location, origin)),
            waypoint.radius,
            0f32,
        )
    }
}

impl Node {
    pub fn new(origin: Point, radius: f32, height: f32) -> Self {
        let left_head = Wrapper::new(Vertex::new_head(&mut HEADER_VERTEX_INDEX, origin));
        {
            let head = left_head.lock();
            head.borrow_mut().next = Some(left_head.clone());
            head.borrow_mut().prev = Some(left_head.clone());
        }
        let right_head = Wrapper::new(Vertex::new_head(&mut HEADER_VERTEX_INDEX, origin));
        {
            let head = right_head.lock();
            head.borrow_mut().next = Some(right_head.clone());
            head.borrow_mut().prev = Some(right_head.clone());
        }
        Self {
            origin,
            radius,
            height,
            left_ring: left_head,
            right_ring: right_head,
        }
    }

    // Traverse ring to find current pointer and next pointer for angle
    fn traverse_rings(&self, angle: f32) -> (Wrapper<Vertex>, Wrapper<Vertex>) {
        let (is_left, mut current) = if angle >= 0f32 {
            // Left ring
            (true, self.left_ring.clone())
        } else {
            // Right ring
            (false, self.right_ring.clone())
        };

        loop {
            let next_index;
            let next_angle;
            let next;
            {
                let _current = current.lock();
                // Get index, angle, and pointer to the next vertex
                let (_next_index, _next_angle, _next) = match _current.borrow().next {
                    Some(ref next) => {
                        let _next = next.lock();
                        let (index, angle) = (_next.borrow().index, _next.borrow().angle);
                        (index, angle, next.clone())
                    }
                    None => panic!("Next points to null"),
                };
                next_index = _next_index;
                next_angle = _next_angle;
                next = _next;
            }

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
    pub fn get_vertex(
        &self,
        num_vertices: &mut i32,
        angle: f32,
        threshold: f32,
    ) -> Wrapper<Vertex> {
        println!("Looking for vertex with angle {}", angle);
        let (current, next) = self.traverse_rings(angle);

        let _current = current.lock();
        let _next = next.lock();
        let cur_index = _current.borrow().index;
        let next_index = _next.borrow().index;
        // Ensure current and next are different vertices
        if cur_index != next_index {
            let mut temp_current = current.clone();
            let mut temp_next = next.clone();
            if cur_index == HEADER_VERTEX_INDEX || next_index == HEADER_VERTEX_INDEX {
                if cur_index == HEADER_VERTEX_INDEX {
                    temp_current = match _current.borrow().prev {
                        Some(ref prev) => prev.clone(),
                        None => panic!("broken chain"),
                    };
                } else {
                    temp_next = match _next.borrow().next {
                        Some(ref next) => next.clone(),
                        None => panic!("broken chain"),
                    };
                }
            }
            let arc_a;
            let arc_b;
            {
                let _temp_current = temp_current.lock();
                let _temp_next = temp_next.lock();
                let _arc_a = arc_length(_temp_current.borrow().angle, angle, self.radius);
                let _arc_b = arc_length(angle, _temp_next.borrow().angle, self.radius);
                arc_a = _arc_a;
                arc_b = _arc_b;
            }
            let min_arc = if arc_a < arc_b { arc_a } else { arc_b };
            if min_arc < threshold {
                println!(
                    "found existing vertex {} and {} with dist {}",
                    _current.borrow(),
                    _next.borrow(),
                    min_arc
                );

                return if arc_a < arc_b {
                    temp_current
                } else {
                    temp_next
                };
            }
        }

        println!(
            "insert new vertex {} between {} and {}",
            *num_vertices + 1,
            _current.borrow().index,
            _next.borrow().index
        );
        let v = Wrapper::new(Vertex::new(num_vertices, self, angle, vec![]));
        {
            let _v = v.lock();
            _next.borrow_mut().prev = Some(v.clone());
            _v.borrow_mut().next = Some(next.clone());
            _v.borrow_mut().prev = Some(current.clone());
            _current.borrow_mut().next = Some(v.clone());
        }

        v
    }

    // Insert an existing vertex into a node
    pub fn insert_vertex(&mut self, v: Wrapper<Vertex>) {
        let _v = v.lock();
        let (current, next) = self.traverse_rings(_v.borrow().angle);
        let _current = current.lock();
        let _next = next.lock();
        _next.borrow_mut().prev = Some(v.clone());
        _v.borrow_mut().next = Some(next.clone());
        _v.borrow_mut().prev = Some(current.clone());
        _current.borrow_mut().next = Some(v.clone());
    }

    pub fn prune_vertices(target: LinkedList<Wrapper<Vertex>>) {
        for v in target {
            let _v = v.lock();
            let prev = match _v.borrow_mut().prev {
                Some(ref vert) => vert.clone(),
                None => panic!("Next points to null"),
            };
            let next = match _v.borrow_mut().next {
                Some(ref vert) => vert.clone(),
                None => panic!("Next points to null"),
            };
            let _prev = prev.lock();
            let _next = next.lock();
            _prev.borrow_mut().next = Some(next.clone());
            _next.borrow_mut().prev = Some(prev.clone());
        }
    }
}
