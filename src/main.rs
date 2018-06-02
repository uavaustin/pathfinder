extern crate Obstacle_Path_Finder;

use Obstacle_Path_Finder::*;
use std::collections::LinkedList;

fn main() {
    println!("---------------");
    println!("test1");

    let flight_zone = vec!(
        Point::from_degrees(30.32521, -97.6023, 0f32),
        Point::from_degrees(30.32466, -97.59856, 0f32),
        Point::from_degrees(30.32107, -97.60032, 0f32),
        Point::from_degrees(30.32247, -97.60325, 0f32),
        Point::from_degrees(30.32473, -97.6041, 0f32)
    );
    println!("Flightzone:");
    for point in &flight_zone {
        println!("{:.5}, {:.5}", point.lat_degree(), point.lon_degree());
    }
    println!();
    let obstacles = vec!(
        Obstacle{coords: Point::from_degrees(30.32457, -97.60254, 0f32), radius: 50.0, height: 1.0},
        Obstacle{coords: Point::from_degrees(30.32429, -97.60166, 0f32), radius: 50.0, height: 1.0},
        Obstacle{coords: Point::from_degrees(30.32405, -97.60015, 0f32), radius: 50.0, height: 1.0},
        Obstacle{coords: Point::from_degrees(30.32344, -97.60077, 0f32), radius: 50.0, height: 1.0},
        Obstacle{coords: Point::from_degrees(30.32466, -97.60327, 0f32), radius: 50.0, height: 1.0}
    );

    let mut waypoints = LinkedList::new();
    waypoints.push_back(
        Waypoint::new(0, Point::from_degrees(30.32271, -97.60035, 100f32), 10f32)
    );
    waypoints.push_back(
        Waypoint::new(1, Point::from_degrees(30.32457, -97.59972, 150f32), 10f32)
    );
    // waypoints.push_back(
    //     Waypoint::new(2, Point::from_degrees(30.32271, -97.60035, 100f32), 10f32)
    // );
    let flyzone = vec!(flight_zone);
    let mut path_finder1 = PathFinder::new();
    path_finder1.init(5.0, flyzone, obstacles);
    let result = path_finder1.get_adjust_path(
        Plane::new(30.32491, -97.60159, 10.0),
        waypoints);
    println!("A* Result");
    for node in result {
        println!("{:.5}, {:.5}, {:.5}",
        node.location.lat_degree(), node.location.lon_degree(), node.location.alt());
    }
    println!();

    println!("A* Result w/o alt");
    for node in result {
        println!("{:.5}, {:.5}",
        node.location.lat_degree(), node.location.lon_degree());
    }
    println!();

    // path_finder1.draw(0,100,0,100);
    // result = path_finder1.adjust_path_jump_point(Plane::new(30.32456, -97.60283, 10.0));
    // if let Some(result) = result {
    //     println!("Jump Result");
    //     for node in result {
    //         pr// result = path_finder1.adjust_path_jump_point(Plane::new(30.32456, -97.60283, 10.0));
    // if let Some(result) = result {
    //     println!("Jump Result");
    //     for node in result {
    //         println!("{:.5}, {:.5}", node.location.lat_degree(), node.location.lon_degree());
    //     }
    // }intln!("{:.5}, {:.5}", node.location.lat_degree(), node.location.lon_degree());
    //     }
    // }
}
