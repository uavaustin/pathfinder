use super::*;

use std::fmt;

impl Vertex {
    pub fn new(angle: f32, connection: Option<Connection>) -> Vertex {
        Vertex {
            angle: angle,
            connection: connection,
            next: None,
        }
    }
}

impl fmt::Display for Vertex {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "(angle={}, connection={} next={})",
            self.angle,
            self.connection.is_some(),
            self.next.is_some()
        )
    }
}
