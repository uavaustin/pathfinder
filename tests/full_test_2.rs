extern crate pathfinder;

use pathfinder::*;

mod util;
use util::*;

#[test]
fn test2() {
    let flyzone = vec![vec![
        Location::from_degrees(30.276450732764616, -97.74291515350342, 0f32),
        Location::from_degrees(30.276450732764616, -97.7239465713501, 0f32),
        Location::from_degrees(30.29294185380876, -97.7239465713501, 0f32),
        Location::from_degrees(30.29294185380876, -97.74291515350342, 0f32),
    ]];
    let obstacles = vec![Obstacle::from_degrees(
        30.286975723301133,
        -97.7305555343628,
        50f32,
        50f32,
    )];
    let waypoints = vec_to_list::<()>(vec![Waypoint::from_degrees(
        30.287401, -97.726685, 100f32, 10f32,
    )]);

    let mut pathfinder = Pathfinder::new(Tanstar::new(), TConfig::default(), flyzone, obstacles);
    let plane = Plane::from_degrees(30.288105, -97.73533, 10.0);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}
