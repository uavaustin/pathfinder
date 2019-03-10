use super::*;

impl Connection {
    pub fn new(neighbor: Rc<RefCell<Vertex>>, distance: f32, threshold: f32) -> Self {
        Connection {
            neighbor: neighbor,
            distance: distance,
            threshold: threshold,
        }
    }
}
