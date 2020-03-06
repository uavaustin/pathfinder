use super::*;
use protobuf::*;
use std::collections::linked_list::LinkedList;

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
