// lib.rs
// contains exposed functionality of the library
#![allow(dead_code)]
#![allow(unused_variables)]

pub mod obj;
pub mod tanstar;

mod algorithm;

pub use obj::*;
pub use tanstar::{TConfig, Tanstar};

use algorithm::Algorithm;
use std::collections::LinkedList;

pub struct Pathfinder<A: Algorithm> {
    algo: A,
}

impl<A: Algorithm> Pathfinder<A> {
    pub fn new(
        mut algo: A,
        config: A::Config,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        algo.init(config, flyzones, obstacles);
        Self { algo: algo }
    }

    pub fn get_adjust_path<T>(
        &mut self,
        plane: Plane,
        mut wp_list: LinkedList<Waypoint<T>>,
    ) -> LinkedList<Waypoint<T>> {
        let mut new_wp_list = LinkedList::new();
        let mut current_wp: Waypoint<T>;
        let mut current_loc = plane.location;

        loop {
            match wp_list.pop_front() {
                Some(wp) => current_wp = wp,
                None => break,
            }

            let next_loc = current_wp.location.clone();

            if let Some(mut path) = self.algo.adjust_path::<T>(current_loc, next_loc) {
                println!("appending");
                new_wp_list.append(&mut path);
            } else {
                println!("no path");
                break;
            }

            current_loc = current_wp.location;
            new_wp_list.push_back(current_wp);
            // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint
            // self.wp_list.push_back(Waypoint::from_degrees(0, 30.69, -97.69, 100f32, 10f32));
        }

        new_wp_list
    }

    pub fn set_config(&mut self, config: A::Config) {
        self.algo.set_config(config);
    }

    pub fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>) {
        self.algo.set_flyzone(flyzone);
    }

    pub fn set_obstacles(&mut self, obstacles: Vec<Obstacle>) {
        self.algo.set_obstacles(obstacles);
    }

    pub fn get_config(&self) -> &A::Config {
        self.algo.get_config()
    }

    pub fn get_flyzone(&mut self) -> &Vec<Vec<Location>> {
        self.algo.get_flyzone()
    }

    pub fn get_obstacle(&self) -> &Vec<Obstacle> {
        self.algo.get_obstacles()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn tanstar_invalid_flyzones_test() {
        Pathfinder::new(Tanstar::new(), TConfig::default(), vec![], Vec::new());
    }

    #[test]
    #[should_panic]
    fn tanstar_invalid_flyzone_test() {
        Pathfinder::new(Tanstar::new(), TConfig::default(), vec![vec![]], Vec::new());
    }
}
