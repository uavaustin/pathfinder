extern crate pathfinder;

use pathfinder::obj::*;
use pathfinder::Pathfinder;

mod util;
use util::*;

#[test]
fn test1() {
    let flyzone = vec![
        Location::from_degrees(30.32521, -97.6023, 0f32),
        Location::from_degrees(30.32466, -97.59856, 0f32),
        Location::from_degrees(30.32107, -97.60032, 0f32),
        Location::from_degrees(30.32247, -97.60325, 0f32),
        Location::from_degrees(30.32473, -97.6041, 0f32),
    ];
    let obstacles = vec![
        Obstacle::from_degrees(30.32457, -97.60254, 50f32, 50.0),
        Obstacle::from_degrees(30.32429, -97.60166, 50f32, 50.0),
        Obstacle::from_degrees(30.32405, -97.60015, 50f32, 50.0),
        Obstacle::from_degrees(30.32344, -97.60077, 50f32, 50.0),
        Obstacle::from_degrees(30.32466, -97.60327, 50f32, 50.0),
    ];
    let waypoints = vec_to_list::<()>(vec![
        Waypoint::from_degrees(30.32271, -97.60035, 100f32, 10f32),
        Waypoint::from_degrees(30.32457, -97.59972, 150f32, 10f32),
        //Waypoint::from_degrees(2, 30.32271, -97.60035, 100f32, 10f32)
    ]);

    let flyzone = vec![flyzone];
    let mut pathfinder = Pathfinder::new();
    pathfinder.init(5.0, flyzone, obstacles);
    let plane = Plane::from_degrees(30.32491, -97.60159, 10.0);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}
