extern crate pathfinder;
use std::collections::LinkedList;

use pathfinder::obj::*;

pub fn vec_to_list(waypoints: Vec<Waypoint>) -> LinkedList<Waypoint> {
    let mut list = LinkedList::new();
    for wp in waypoints {
        list.push_back(wp);
    }
    list
}

pub fn output_result(waypoints: LinkedList<Waypoint>, result: &LinkedList<Waypoint>, plane: Plane) {
    let mut iter = result.iter();
    eprintln!(
        "{:.5}, {:.5}",
        plane.location.lon_degree(),
        plane.location.lat_degree(),
        // plane.location.alt()
    );
    for wp in waypoints {
        while let Some(node) = iter.next() {
            if node.index != wp.index {
                break;
            }
            eprintln!(
                "{:.5}, {:.5}",
                node.location.lon_degree(),
                node.location.lat_degree(),
                // node.location.alt(),
                // node.index
            );
        }
        eprintln!(
            "{:.5}, {:.5}",
            wp.location.lon_degree(),
            wp.location.lat_degree(),
            // wp.location.alt(),
            // wp.index
        );
    }

    println!();
}
