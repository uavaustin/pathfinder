extern crate Obstacle_Path_Finder;

use Obstacle_Path_Finder::*;

fn main() {
    println!("---------------");
    println!("test1");

    let flight_zone = vec!(
        Point::from_degrees(30.32521, -97.6023),
        Point::from_degrees(30.32466, -97.59856),
        Point::from_degrees(30.32107, -97.60032),
        Point::from_degrees(30.32247, -97.60325),
        Point::from_degrees(30.32473, -97.6041)
    );
    let obstacles = vec!(
        // Obstacle{coords: Point::from_degrees(30.32456, -97.60283), radius: 120.0, height: 1.0},
        Obstacle{coords: Point::from_degrees(30.32447, -97.60157), radius: 120.0, height: 1.0},
        Obstacle{coords: Point::from_degrees(30.32374, -97.60232), radius: 120.0, height: 1.0}
    );
    let waypoints = vec!(
        Waypoint::new(Point::from_degrees(30.32392, -97.60157))
    );
    let mut path_finder1 = PathFinder::new(10.0, flight_zone);
    path_finder1.set_obstacle_list(obstacles);
    path_finder1.set_waypoint_list(waypoints);
    let mut result = path_finder1.adjust_path(Plane::new(30.32456, -97.60283, 10.0));
    if let Some(result) = result {
        for node in result {
            println!("{:.5}, {:.5}", node.location.lat_degree(), node.location.lon_degree());
        }
    }
    // result = path_finder1.adjust_path_jump_point(Plane::new(30.32456, -97.60283, 10.0));
    // if let Some(result) = result {
    //     for node in result {
    //         println!("{:.5}, {:.5}", node.location.lat_degree(), node.location.lon_degree());
    //     }
    // }
    // for node in path_finder1.obstacle_list {
    //     println!("x: {} y: {}", node.x, node.y);
    // }
}
