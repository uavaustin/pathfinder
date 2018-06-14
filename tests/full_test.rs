extern crate pathfinder;
use std::collections::LinkedList;

use pathfinder::*;

#[test]
fn test0() {
    let waypoints = vec_to_list(vec![
        Waypoint::from_degrees(0, 30.322280883789063, -97.60298156738281, 100f32, 10f32),
        Waypoint::from_degrees(1, 30.322280883789063, -97.60098266601564, 150f32, 10f32),
    ]);
    let flyzone = vec![vec![
        Point::from_degrees(30.32469, -97.60466, 0f32),
        Point::from_degrees(30.32437, -97.60367, 0f32),
        Point::from_degrees(30.32356, -97.60333, 0f32),
        Point::from_degrees(30.32276, -97.60398, 0f32),
        Point::from_degrees(30.32082, -97.60368, 0f32),
        Point::from_degrees(30.32173, -97.60008, 0f32),
        Point::from_degrees(30.32329, -97.59958, 0f32),
        Point::from_degrees(30.32545, -97.60066, 0f32),
        Point::from_degrees(30.32608, -97.60201, 0f32),
        Point::from_degrees(30.32613, -97.60339, 0f32),
        Point::from_degrees(30.32537, -97.60453, 0f32),
    ]];
    let obstacles = vec![Obstacle::from_degrees(30.32228, -97.60198, 50f32, 10f32)];

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(5.0, flyzone, obstacles);
    let plane = Plane::from_degrees(30.32298, -97.60310, 100.0).yaw(170f32);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}

#[test]
fn test1() {
    let flyzone = vec![
        Point::from_degrees(30.32521, -97.6023, 0f32),
        Point::from_degrees(30.32466, -97.59856, 0f32),
        Point::from_degrees(30.32107, -97.60032, 0f32),
        Point::from_degrees(30.32247, -97.60325, 0f32),
        Point::from_degrees(30.32473, -97.6041, 0f32),
    ];
    let obstacles = vec![
        Obstacle::from_degrees(30.32457, -97.60254, 50f32, 50.0),
        Obstacle::from_degrees(30.32429, -97.60166, 50f32, 50.0),
        Obstacle::from_degrees(30.32405, -97.60015, 50f32, 50.0),
        Obstacle::from_degrees(30.32344, -97.60077, 50f32, 50.0),
        Obstacle::from_degrees(30.32466, -97.60327, 50f32, 50.0),
    ];
    let waypoints = vec_to_list(vec![
        Waypoint::from_degrees(0, 30.32271, -97.60035, 100f32, 10f32),
        Waypoint::from_degrees(1, 30.32457, -97.59972, 150f32, 10f32),
        //Waypoint::from_degrees(2, 30.32271, -97.60035, 100f32, 10f32)
    ]);

    let flyzone = vec![flyzone];
    let mut pathfinder = Pathfinder::new();
    pathfinder.init(5.0, flyzone, obstacles);
    let plane = Plane::from_degrees(30.32491, -97.60159, 10.0);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}

#[test]
fn test2() {
    let flyzone = vec![vec![
        Point::from_degrees(30.276450732764616, -97.74291515350342, 0f32),
        Point::from_degrees(30.276450732764616, -97.7239465713501, 0f32),
        Point::from_degrees(30.29294185380876, -97.7239465713501, 0f32),
        Point::from_degrees(30.29294185380876, -97.74291515350342, 0f32),
    ]];
    let obstacles = vec![Obstacle::from_degrees(
        30.286975723301133,
        -97.7305555343628,
        50f32,
        50f32,
    )];
    let waypoints = vec_to_list(vec![Waypoint::from_degrees(
        0, 30.287401, -97.726685, 100f32, 10f32,
    )]);

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(5.0, flyzone, obstacles);
    let plane = Plane::from_degrees(30.288105, -97.73533, 10.0);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}

#[test]
fn test3() {
    let flyzone = vec![vec![
        Point::from_degrees(30.276450732764616, -97.74291515350342, 0f32),
        Point::from_degrees(30.276450732764616, -97.7239465713501, 0f32),
        Point::from_degrees(30.29294185380876, -97.7239465713501, 0f32),
        Point::from_degrees(30.29294185380876, -97.74291515350342, 0f32),
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

    let mut pathfinder = Pathfinder::new();
    let plane = Plane::from_degrees(30.2881757, -97.7354343, 10.0);
    pathfinder.init(5.0, flyzone, obstacles);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}

#[test]
fn test4() {
    let flyzone = vec![vec![
        Point::from_degrees(38.14627, -76.42816, 0f32),
        Point::from_degrees(38.15162, -76.42868, 0f32),
        Point::from_degrees(38.15189, -76.43147, 0f32),
        Point::from_degrees(38.15059, -76.43536, 0f32),
        Point::from_degrees(38.14757, -76.43234, 0f32),
        Point::from_degrees(38.14467, -76.43295, 0f32),
        Point::from_degrees(38.14326, -76.43477, 0f32),
        Point::from_degrees(38.14046, -76.43264, 0f32),
        Point::from_degrees(38.14072, -76.42601, 0f32),
        Point::from_degrees(38.14376, -76.42121, 0f32),
        Point::from_degrees(38.14735, -76.42321, 0f32),
        Point::from_degrees(38.14613, -76.42665, 0f32),
    ]];
    let obstacles = vec![Obstacle::from_degrees(38.14376, -76.42816, 50f32, 250f32)];
    let waypoints = vec_to_list(vec![Waypoint::from_degrees(
        0, 38.14376, -76.42321, 76.1, 10f32,
    )]);

    let mut pathfinder = Pathfinder::new();
    pathfinder.init(5.0, flyzone, obstacles);
    let plane = Plane::from_degrees(38.15059, -76.43147, 10.0);
    let result = pathfinder.get_adjust_path(plane.clone(), waypoints.clone());
    output_result(waypoints, result, plane);
}

fn vec_to_list(waypoints: Vec<Waypoint>) -> LinkedList<Waypoint> {
    let mut list = LinkedList::new();
    for wp in waypoints {
        list.push_back(wp);
    }
    list
}

fn output_result(waypoints: LinkedList<Waypoint>, result: &LinkedList<Waypoint>, plane: Plane) {
    let mut iter = result.iter();
    eprintln!(
        "{:.5}, {:.5}, {:.5}",
        plane.location.lat_degree(),
        plane.location.lon_degree(),
        plane.location.alt()
    );
    for wp in waypoints {
        while let Some(node) = iter.next() {
            if node.index != wp.index {
                break;
            }
            eprintln!(
                "{:.5}, {:.5}, {:.5} {}",
                node.location.lat_degree(),
                node.location.lon_degree(),
                node.location.alt(),
                node.index
            );
        }
        eprintln!(
            "{:.5}, {:.5}, {:.5} {}",
            wp.location.lat_degree(),
            wp.location.lon_degree(),
            wp.location.alt(),
            wp.index
        );
    }

    println!();
}
