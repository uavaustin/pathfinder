//! A set of structs for handling general pathfinding data

mod location;
mod obstacle;
mod plane;
mod waypoint;

pub use self::location::Location;
pub use self::obstacle::Obstacle;
pub use self::plane::Plane;
pub use self::waypoint::Waypoint;
