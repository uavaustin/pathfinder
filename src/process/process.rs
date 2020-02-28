use super::*;
use std::collections::linked_list::LinkedList;
use std::time::Duration;

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

    pub fn parse(&mut self, input: String) {
        // **Look into Thruster**

        // parse into data
        let lines = input.split("\n");
        for mut line in lines {
            // get
            let reverse_index: usize;
            match line.rfind(": ") {
                Some(x) => reverse_index = x,
                None => continue,
            }
            let data_type = &line[..reverse_index];
            line = &line[reverse_index..];
            // check what kind of data is contained in this line
            match data_type {
                "Config:" => self.parse_config(line.to_string()),
                "Flyzones:" => self.parse_flyzones(line.to_string()),
                "Plane_Location:" => self.parse_plane_location(line.to_string()),
                "Obstacles:" => self.parse_obstacles(line.to_string()),
                "Waypoints:" => self.parse_waypoints(line.to_string()),
                _ => (),
            }
        }
    }

    fn parse_config(&mut self, input: String) {
        /*
        let data = input.split(',');
        //f32, Duration, f32, f32, bool
        let buffer_size = parse::f32(data[0]);
        let max_process_time = parse(data[1], );
        let turning_radius = parse::f32(data[2]);
        let vertex_merge_threshold = parse::f32(data[3]);
        let virtualize_flyzone = parse::bool(data[4]);

        // return placeholder
        self.config = TConfig::new(buffer_size, /*max_process_time*/, turning_radius, vertex_merge_threshold, virtualize_flyzone);
        */
        self.config = TConfig::default();
    }

    fn parse_flyzones(&mut self, input: String) {
        // return placeholder
        self.flyzones = vec![vec![Location::from_degrees(0f64, 0f64, 0f32)]];
    }

    fn parse_plane_location(&mut self, input: String) {
        // return placeholder
        self.plane_location = Location::from_degrees(0f64, 0f64, 0f32)
    }

    fn parse_obstacles(&mut self, input: String) {
        // return placeholder
        self.obstacles = vec![Obstacle::new(
            Location::from_degrees(0f64, 0f64, 0f32),
            0f32,
            0f32,
        )];
    }

    fn parse_waypoints(&mut self, input: String) {
        // return placeholder
        self.waypoints = LinkedList::new();
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
