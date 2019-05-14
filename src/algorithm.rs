// algorithm.rs
// trait that defines minimum requirement for a pathfinding algorithm

use super::*;

pub trait Algorithm {
    type Config;

    fn init(
        &mut self,
        config: Self::Config,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    );
    fn adjust_path<T>(&mut self, start: Location, end: Location)
        -> Option<LinkedList<Waypoint<T>>>;

    // Getters
    fn get_config(&self) -> Self::Config;
    fn get_flyzone(&mut self) -> &Vec<Vec<Location>>;
    fn get_obstacles(&self) -> &Vec<Obstacle>;

    // Setters
    fn set_config(&mut self, config: Self::Config);
    fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>);
    fn set_obstacles(&mut self, obstacles: Vec<Obstacle>);
}
