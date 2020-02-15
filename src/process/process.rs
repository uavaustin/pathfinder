use super::*;
use std::collections::linked_list::LinkedList;

pub struct Process {
    pub flyzones: Vec<Vec<Location>>,
    pub obstacles: Vec<Obstacle>,
    pub waypoints: LinkedList<Waypoint<()>>,
    pub plane_location: Location,
    pub config: TConfig,
}

impl Process {
    pub fn new(
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
        waypoints: LinkedList<Waypoint<()>>,
        plane_location: Location,
        config: TConfig,
    ) -> Self {
        Self {
            flyzones,
            obstacles,
            waypoints,
            plane_location,
            config,
        }
    }

    pub fn run(&self) -> LinkedList<Waypoint<()>> {
        let mut pathfinder = Pathfinder::new(
            Tanstar::new(),
            self.config.clone(),
            self.flyzones.clone(),
            self.obstacles.clone(),
        );
        let plane = Plane::new(self.plane_location.clone());
        pathfinder.get_adjust_path(plane.clone(), self.waypoints.clone())
    }

    pub fn parse(input: &String) -> Self {
        // parse into data

        // let mut iter = input.split_whitespace();

        // return placeholder
        Self::new(
            vec![vec![Location::from_degrees(0f64, 0f64, 0f32)]],
            vec![Obstacle::new(
                Location::from_degrees(0f64, 0f64, 0f32),
                0f32,
                0f32,
            )],
            LinkedList::new(),
            Location::from_degrees(0f64, 0f64, 0f32),
            TConfig::default(),
        )
    }
}
