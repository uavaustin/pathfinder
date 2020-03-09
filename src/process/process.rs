use super::*;
use protobuf::*;
use std::collections::linked_list::LinkedList;

#[derive(Eq)]
pub struct Process {
    pub flyzones: Vec<Vec<Location>>,
    pub obstacles: Vec<Obstacle>,
    pub waypoints: LinkedList<Waypoint<()>>,
    pub plane: Plane,
    pub config: TConfig,
}

impl Process {
    pub fn new(
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
        waypoints: LinkedList<Waypoint<()>>,
        plane: Plane,
        config: TConfig,
    ) -> Self {
        Self {
            flyzones,
            obstacles,
            waypoints,
            plane,
            config,
        }
    }

    pub fn parse(cis: &mut CodedInputStream) -> ProtobufResult<Self> {
        // parse request
        let request: PathRequest = cis.read_message()?;

        // get flyzones
        let mut flyzones = Vec::new();
        // go through each request Flyzone
        for fz in request.get_flyzoneList() {
            // create a flyzone from the points (locations) of the boundary
            let mut flyzone = Vec::new();
            for loc in fz.get_polygon() {
                flyzone.push(Location::from_degrees(
                    loc.get_lat(),
                    loc.get_lon(),
                    loc.get_alt() as f32,
                ));
            }
            flyzones.push(flyzone);
        }

        // get obstacles
        let mut obstacles = Vec::new();
        // go through each request Obstacle
        for obs in request.get_obstacleList() {
            // create an obj Obstacle
            let pos = obs.get_center();
            let obstacle = Obstacle::from_degrees(
                pos.get_lon(),
                pos.get_lat(),
                obs.get_radius() as f32,
                pos.get_alt() as f32,
            );
            obstacles.push(obstacle);
        }

        // get waypoints
        let mut waypoints = LinkedList::new();
        // go through each request waypoint
        for wp in request.get_oldWaypoints() {
            // create a Waypoint with arbitrary radius (small to mimic a mathematical point)
            waypoints.push_back(Waypoint::from_degrees(
                wp.get_lon(),
                wp.get_lon(),
                wp.get_alt() as f32,
                0.01f32,
            ));
        }

        // get plane location
        let plane_loc = request.get_planeLocation();
        let plane = Plane::from_degrees(
            plane_loc.get_lon(),
            plane_loc.get_lat(),
            plane_loc.get_alt() as f32,
        );

        // return process
        Ok(Process::new(
            flyzones,
            obstacles,
            waypoints,
            plane,
            TConfig::default(),
        ))
    }

    pub fn run(&self) -> LinkedList<Waypoint<()>> {
        let mut pathfinder = Pathfinder::new(
            Tanstar::new(),
            self.config.clone(),
            self.flyzones.clone(),
            self.obstacles.clone(),
        );
        pathfinder.get_adjust_path(self.plane.clone(), self.waypoints.clone())
    }
}

impl Default for Process {
    // return a bunch of empty stuff
    fn default() -> Self {
        Self::new(
            vec![vec![]],
            vec![],
            LinkedList::new(),
            Plane::new(Location::from_degrees(0f64, 0f64, 0f32)),
            TConfig::default(),
        )
    }
}

impl PartialEq for Process {
    fn eq(&self, other: &Self) -> bool {
        self.flyzones.eq(&other.flyzones)
            && self.obstacles.eq(&other.obstacles)
            && self.waypoints.eq(&other.waypoints)
            && self.plane.eq(&other.plane)
            && self.config.eq(&other.config)
    }

    fn ne(&self, other: &Self) -> bool {
        self.flyzones.ne(&other.flyzones)
            && self.obstacles.ne(&other.obstacles)
            && self.waypoints.ne(&other.waypoints)
            && self.plane.ne(&other.plane)
            && self.config.ne(&other.config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        // create some process
        let flyzone = vec![vec![Location::from_degrees(30.32469, -97.60466, 0f32)]];
        let obstacles = vec![Obstacle::from_degrees(30.32228, -97.60198, 50f32, 200f32)];
        let mut waypoints = LinkedList::new();
        waypoints.push_front(Waypoint::from_degrees(
            30.322280883789063,
            -97.60098266601564,
            100f32,
            10f32,
        ));
        let plane = Plane::from_degrees(30.322280883789063, -97.60298156738281, 100f32).yaw(170f32);
        let process = Process::new(flyzone, obstacles, waypoints, plane, TConfig::default());

        // if all has gone well so far test is successful
        assert!(true);
    }

    #[test]
    fn test_equality() {
        // create some process
        let flyzone1 = vec![vec![Location::from_degrees(30.32469, -97.60466, 0f32)]];
        let obstacles1 = vec![Obstacle::from_degrees(30.32228, -97.60198, 50f32, 200f32)];
        let mut waypoints1 = LinkedList::new();
        waypoints1.push_front(Waypoint::from_degrees(
            30.322280883789063,
            -97.60098266601564,
            100f32,
            10f32,
        ));
        let plane1 =
            Plane::from_degrees(30.322280883789063, -97.60298156738281, 100f32).yaw(170f32);
        let process1 = Process::new(flyzone1, obstacles1, waypoints1, plane1, TConfig::default());

        // create another process
        let flyzone2 = vec![vec![Location::from_degrees(30.32469, -97.60466, 0f32)]];
        let obstacles2 = vec![Obstacle::from_degrees(30.32228, -97.60198, 50f32, 200f32)];
        let mut waypoints2 = LinkedList::new();
        waypoints2.push_front(Waypoint::from_degrees(
            30.322280883789063,
            -97.60098266601564,
            100f32,
            10f32,
        ));
        let plane2 =
            Plane::from_degrees(30.322280883789063, -97.60298156738281, 100f32).yaw(170f32);
        let process2 = Process::new(flyzone2, obstacles2, waypoints2, plane2, TConfig::default());

        assert!(process1.eq(&process2));
    }

    #[test]
    fn test_run() {
        // create some process
        let flyzone = vec![vec![
            Location::from_degrees(30.32469, -97.60466, 0f32),
            Location::from_degrees(30.32437, -97.60367, 0f32),
            Location::from_degrees(30.32356, -97.60333, 0f32),
            Location::from_degrees(30.32276, -97.60398, 0f32),
            Location::from_degrees(30.32082, -97.60368, 0f32),
            Location::from_degrees(30.32173, -97.60008, 0f32),
            Location::from_degrees(30.32329, -97.59958, 0f32),
            Location::from_degrees(30.32545, -97.60066, 0f32),
            Location::from_degrees(30.32608, -97.60201, 0f32),
            Location::from_degrees(30.32613, -97.60339, 0f32),
            Location::from_degrees(30.32537, -97.60453, 0f32),
        ]];
        let obstacles = vec![
            Obstacle::from_degrees(30.32228, -97.60198, 50f32, 200f32),
            Obstacle::from_degrees(30.32332, -97.60183, 30f32, 200f32),
            Obstacle::from_degrees(30.32393, -97.60172, 20f32, 200f32),
        ];
        let mut waypoints = LinkedList::new();
        waypoints.push_front(Waypoint::from_degrees(
            30.322280883789063,
            -97.60098266601564,
            100f32,
            10f32,
        ));
        let plane = Plane::from_degrees(30.322280883789063, -97.60298156738281, 100f32).yaw(170f32);
        let process = Process::new(flyzone, obstacles, waypoints, plane, TConfig::default());
        let path = process.run();

        // check that path goes somewhere (and has more than one waypoint)
        assert!(path.len() >= 2);
        // assert!(distance(path) > 0f32); TODO: implement at some point
    }
}
