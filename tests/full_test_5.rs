extern crate pathfinder;

use pathfinder::obj::*;
use pathfinder::Pathfinder;

mod util;
use util::*;

/*

*/

#[test]
fn test5() {
    let waypoints = vec_to_list(vec![
        Waypoint::from_degrees(0, 30.32551, -97.60331, 150f32, 10f32),
        Waypoint::from_degrees(1, 30.32222, -97.60060, 70f32, 10f32),
    ]);
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
    Obstacle::from_degrees(30.32497,-97.60275,36f32,200f32),
    Obstacle::from_degrees(30.32308,-97.60104,54f32,200f32),
    ];

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(5.0, flyzone, obstacles);
    let plane = Plane::from_degrees(30.32298, -97.60310, 100.0).yaw(170f32);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}
