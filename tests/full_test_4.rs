extern crate pathfinder;

use pathfinder::*;

mod util;
use util::*;

#[test]
fn test4() {
    let flyzone = vec![vec![
        Location::from_degrees(38.14627, -76.42816, 0f32),
        Location::from_degrees(38.15162, -76.42868, 0f32),
        Location::from_degrees(38.15189, -76.43147, 0f32),
        Location::from_degrees(38.15059, -76.43536, 0f32),
        Location::from_degrees(38.14757, -76.43234, 0f32),
        Location::from_degrees(38.14467, -76.43295, 0f32),
        Location::from_degrees(38.14326, -76.43477, 0f32),
        Location::from_degrees(38.14046, -76.43264, 0f32),
        Location::from_degrees(38.14072, -76.42601, 0f32),
        Location::from_degrees(38.14376, -76.42121, 0f32),
        Location::from_degrees(38.14735, -76.42321, 0f32),
        Location::from_degrees(38.14613, -76.42665, 0f32),
    ]];
    let obstacles = vec![Obstacle::from_degrees(38.14376, -76.42816, 50f32, 250f32)];
    let waypoints = vec_to_list::<()>(vec![Waypoint::from_degrees(
        38.14376, -76.42321, 76.1, 10f32,
    )]);

    let mut pathfinder = Pathfinder::new(Tanstar::new(), TConfig::default(), flyzone, obstacles);
    let plane = Plane::from_degrees(38.15059, -76.43147, 10.0);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}
