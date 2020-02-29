use super::*;
use protobuf::*;
use std::borrow::BorrowMut;
use std::collections::linked_list::LinkedList;
use std::io::Read;
use std::time::Duration;

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

    pub fn run(&self) -> LinkedList<Waypoint<()>> {
        let mut pathfinder = Pathfinder::new(
            Tanstar::new(),
            self.config.clone(),
            self.flyzones.clone(),
            self.obstacles.clone(),
        );
        pathfinder.get_adjust_path(self.plane.clone(), self.waypoints.clone())
    }

    pub fn parse(reader: &mut dyn Read) -> ProtobufResult<Self> {
        // parse into data
        let mut is = protobuf::CodedInputStream::new(reader);

        // parse request
        let mut request = process::pathfinder::Request::new();
        match request.merge_from(is.borrow_mut()) {
            Err(e) => return Err(e),
            _ => (),
        }

        // parse plane
        let mut proto_plane = process::pathfinder::Plane::new();
        match proto_plane.merge_from(is.borrow_mut()) {
            Err(e) => return Err(e),
            _ => (),
        }
        // should altitude be msl or agl?
        let mut plane = Plane::new(Location::from_degrees(
            request.get_overview().get_pos().get_lat(),
            request.get_overview().get_pos().get_lon(),
            request.get_overview().get_alt().get_msl() as f32,
        ));
        plane.yaw = proto_plane.get_rot().get_yaw() as f32;
        plane.pitch = proto_plane.get_rot().get_pitch() as f32;
        plane.roll = proto_plane.get_rot().get_roll() as f32;
        plane.airspeed = proto_plane.get_speed().get_airspeed() as f32;
        plane.groundspeed = proto_plane.get_speed().get_ground_speed() as f32;
        // plane.wind_dir = // no source so ignore TODO: get source and don't ignore

        // parse flyzones
        let mut flyzones = Vec::new();
        // go through each InteropMission_FlyZone
        for im_fz in request.get_flyzones() {
            // create a flyzone from the locations of the boundary
            let mut fz = Vec::new();
            for loc in im_fz.get_boundary() {
                fz.push(Location::from_degrees(loc.get_lat(), loc.get_lon(), 0f32));
            }
            flyzones.push(fz);
        }

        // parse obstacles
        let mut obstacles = Vec::new();
        // go through each Obstacles_StationaryObstacle
        for s_o in request.get_obstacles().get_stationary() {
            // create an obstacle
            let pos = s_o.get_pos();
            let obs = Obstacle::from_degrees(
                pos.get_lon(),
                pos.get_lat(),
                s_o.get_radius() as f32,
                s_o.get_height() as f32,
            );
            obstacles.push(obs);
        }

        // parse waypoints
        let mut waypoints = LinkedList::new();
        // for mi in request.get_mission().get_mission_items() {
        //     // TODO: add a bunch of waypoints from mi.get_x(), mi.get_y(), mi.get_z() (probably)
        //     waypoints.push_back()
        // }

        // parse config (assuming overview.algorithm is Tanstar) TODO: don't assume
        let mut config = TConfig::default();
        config.buffer_size = request.get_buffer_size() as f32;
        config.max_process_time = Duration::from_secs(request.get_process_time().into());

        // return process
        Ok(Process::new(flyzones, obstacles, waypoints, plane, config))
    }
}

impl Default for Process {
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

#[cfg(test)]
mod tests {
    /*
    #[test]
    fn test_parse() {

    }
    */
}
