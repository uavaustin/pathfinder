extern crate pathfinder;

use pathfinder::obj::*;
use pathfinder::Pathfinder;
use std::collections::LinkedList;

mod util;
use util::*;

#[test]
fn test3() {
    let flyzone = vec![vec![
        Location::from_degrees(30.276450732764616, -97.74291515350342, 0f32),
        Location::from_degrees(30.276450732764616, -97.7239465713501, 0f32),
        Location::from_degrees(30.29294185380876, -97.7239465713501, 0f32),
        Location::from_degrees(30.29294185380876, -97.74291515350342, 0f32),
    ]];
    let obstacles = vec![Obstacle::from_degrees(
        30.286975723301133,
        -97.7305555343628,
        150f32,
        250f32,
    )];
    let waypoints = vec_to_list(vec![
        Waypoint::from_degrees(0, 30.28718185424805, -97.72671508789063, 76.1, 10f32),
        Waypoint::from_degrees(1, 30.283584594726563, -97.731201171875, 76.1, 10f32),
        Waypoint::from_degrees(2, 30.289718627929688, -97.73104858398439, 76.1, 10f32),
    ]);

    let mut pathfinder = Pathfinder::<()>::new();
    let plane = Plane::from_degrees(30.2881757, -97.7354343, 10.0);
    pathfinder.init(5.0, flyzone, obstacles);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}

#[test]
fn test3_with_data() {
    #[derive(Clone, Debug)]
    enum Data<A, B, C> {
        Uno(A),
        Dos(B),
        Tres(C),
    };

    let flyzone = vec![vec![
        Location::from_degrees(30.276450732764616, -97.74291515350342, 0f32),
        Location::from_degrees(30.276450732764616, -97.7239465713501, 0f32),
        Location::from_degrees(30.29294185380876, -97.7239465713501, 0f32),
        Location::from_degrees(30.29294185380876, -97.74291515350342, 0f32),
    ]];
    let obstacles = vec![Obstacle::from_degrees(
        30.286975723301133,
        -97.7305555343628,
        150f32,
        250f32,
    )];
    let waypoints: LinkedList<Waypoint<Data<String, u8, [u8; 6]>>> = vec_to_list(vec![
        Waypoint::<()>::from_degrees(0, 30.28718185424805, -97.72671508789063, 76.1, 10f32)
            .add_data(Data::Uno(String::from("watch your toes!"))),
        Waypoint::<()>::from_degrees(1, 30.283584594726563, -97.731201171875, 76.1, 10f32)
            .add_data(Data::Dos(42)),
        Waypoint::<()>::from_degrees(2, 30.289718627929688, -97.73104858398439, 76.1, 10f32)
            .add_data(Data::Tres([4, 8, 15, 16, 23, 42])),
    ]);

    let mut pathfinder = Pathfinder::<_>::new();
    let plane = Plane::from_degrees(30.2881757, -97.7354343, 10.0);
    pathfinder.init(5.0, flyzone, obstacles);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}
