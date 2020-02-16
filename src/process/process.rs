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

    pub fn parse(input: String) -> Option<Self> {
        let mut config: Option<TConfig> = None;
        let mut flyzones: Option<Vec<Vec<Location>>> = None;
        let mut plane_location: Option<Location> = None;
        let mut obstacles: Option<Vec<Obstacle>> = None;
        let mut waypoints: Option<LinkedList<Waypoint<()>>> = None;

        // parse into data
        let lines = input.split("\n");
        for line in lines {
            let data_type = &line[..line.rfind(": ")?];
            // check what kind of data is contained in this line
            match data_type {
                "Config:" => config = Self::parse_config(line.to_string()),
                "Flyzones:" => flyzones = Self::parse_flyzones(line.to_string()),
                "Plane_Location:" => plane_location = Self::parse_plane_location(line.to_string()),
                "Obstacles:" => obstacles = Self::parse_obstacles(line.to_string()),
                "Waypoints:" => waypoints = Self::parse_waypoints(line.to_string()),
                _ => (),
            }
        }

        // return Process
        Some(Self::new(
            flyzones?,
            obstacles?,
            waypoints?,
            plane_location?,
            config?,
        ))
    }

    fn parse_config(input: String) -> Option<TConfig> {
        // return placeholder
        Some(TConfig::default())
    }

    fn parse_flyzones(input: String) -> Option<Vec<Vec<Location>>> {
        // return placeholder
        Some(vec![vec![Location::from_degrees(0f64, 0f64, 0f32)]])
    }

    fn parse_plane_location(input: String) -> Option<Location> {
        // return placeholder
        Some(Location::from_degrees(0f64, 0f64, 0f32))
    }

    fn parse_obstacles(input: String) -> Option<Vec<Obstacle>> {
        // return placeholder
        Some(vec![Obstacle::new(
            Location::from_degrees(0f64, 0f64, 0f32),
            0f32,
            0f32,
        )])
    }

    fn parse_waypoints(input: String) -> Option<LinkedList<Waypoint<()>>> {
        // return placeholder
        Some(LinkedList::new())
    }
}

#[cfg(test)]
mod tests {
    /*
    #[test]
    fn test_parse_config() {

    }

    #[test]
    fn test_parse_flyzones() {

    }

    #[test]
    fn test_parse_plane_location() {

    }

    #[test]
    fn test_parse_obstacles() {

    }

    #[test]
    fn test_parse_waypoints() {

    }

    #[test]
    fn test_parse() {

    }
    */
}
