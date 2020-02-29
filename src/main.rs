extern crate pathfinder;
extern crate protobuf;

use pathfinder::*;
use process::process::*;
use protobuf::ProtobufError;
use std::borrow::BorrowMut;
use std::collections::linked_list::LinkedList;
use std::io;
use std::io::BufReader;

pub fn main() -> Result<(), ProtobufError> {
    // TODO: Look into Thruster

    // read to end of incoming data (should include everything for Process)
    let mut reader = BufReader::new(io::stdin());
    // create a Process and insure it is valid (None means it was invalid)
    let process: Process;
    match Process::parse(reader.borrow_mut()) {
        Ok(p) => process = p,
        Err(e) => return Err(e),
    }

    let plane = process.plane;
    let mut pathfinder = Pathfinder::new(
        Tanstar::new(),
        process.config,
        process.flyzones,
        process.obstacles,
    );
    let result = pathfinder.get_adjust_path(plane, process.waypoints);
    output_result(result, plane);

    Ok(())
}

fn output_result<T>(result: LinkedList<Waypoint<T>>, plane: Plane) {
    // send plane position
    println!(
        "\nPlane: {},{},{}",
        plane.location.lat_degree(),
        plane.location.lon_degree(),
        plane.location.alt()
    );

    // send waypoint positions (waypoints separated by spaces)
    print!("Waypoints: ");
    let mut iter = result.iter();
    while let Some(node) = iter.next() {
        print!(
            " {},{},{}",
            node.location.lat_degree(),
            node.location.lon_degree(),
            node.location.alt
        );
    }

    println!();
}
