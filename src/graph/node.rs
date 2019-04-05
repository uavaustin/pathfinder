use super::*;

use std::fmt;

impl fmt::Display for Node {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "loc={:?}, r={} \nleft = [", self.origin, self.radius);
        let mut current = match self.left_ring.borrow().next {
            Some(ref val) => val.clone(),
            None => panic!("next points to null"),
        };
        while current.borrow().index != HEADER_VERTEX_INDEX {
            let ref mut vertex = current.clone();
            write!(f, " {} ", vertex.borrow());
            current = match vertex.borrow().next {
                Some(ref val) => val.clone(),
                None => panic!("next points to null"),
            };
        }
        write!(f, " ] \nright = [");
        let mut current = match self.right_ring.borrow().next {
            Some(ref val) => val.clone(),
            None => panic!("next points to null"),
        };
        while current.borrow().index != HEADER_VERTEX_INDEX {
            let ref mut vertex = current.clone();
            write!(f, " {} ", vertex.borrow());
            current = match vertex.borrow().next {
                Some(ref val) => val.clone(),
                None => panic!("next points to null"),
            };
        }
        write!(f, "]");
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
    pub fn from_obstacle(obs: &Obstacle, origin: &Location) -> Self {
        Node::new(
            Point::from_location(&obs.location, origin),
            obs.radius,
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

    pub fn insert_vertex(&mut self, v: Rc<RefCell<Vertex>>) {
        let new_angle: f32 = v.borrow().angle;
        let (is_left, mut current) = if new_angle > 0f32 {
            // Left ring
            (true, self.left_ring.clone())
        } else {
            // Right ring
            (false, self.right_ring.clone())
        };
        print!(
            "inserting {:?} to header {:?}\n",
            v.borrow().index,
            current.borrow().index
        );
        loop {
            let ref mut vertex = current.clone();
            //print!("{:?}\n", current.borrow().index);
            let index = match vertex.borrow().next {
                Some(ref vert) => vert.borrow().index,
                None => panic!("Next points to null"),
            };
            if index != HEADER_VERTEX_INDEX {
                let (angle, next_ptr) = match vertex.borrow().next {
                    Some(ref next) => (next.borrow().angle, next.clone()),
                    None => panic!("Next points to null"),
                };
                if (is_left && new_angle < angle) || (!is_left && new_angle > angle) {
                    next_ptr.borrow_mut().prev = Some(v.clone());
                    v.borrow_mut().next = Some(next_ptr);
                    v.borrow_mut().prev = Some(vertex.clone());
                    vertex.borrow_mut().next = Some(v.clone());
                    return;
                }
            } else {
                v.borrow_mut().prev = Some(vertex.clone());
                v.borrow_mut().next = vertex.borrow().next.clone();
                vertex.borrow_mut().next = Some(v.clone());
                let next = match v.borrow().next {
                    Some(ref vert) => vert.clone(),
                    None => panic!("Next points to null"),
                };
                next.borrow_mut().prev = Some(v.clone());
                return;
            }
            current = match vertex.borrow().next {
                Some(ref v) => v.clone(),
                None => panic!("Next points to null"),
            };
        }
    }

    pub fn remove_extra_vertices(to_remove: LinkedList<Rc<RefCell<Vertex>>>) {
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
