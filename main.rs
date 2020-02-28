extern crate pathfinder;

use pathfinder::*;
use std::collections::LinkedList;
use std::io;
use std::io::{BufReader, Read};

pub fn main() -> io::Result<()> {
    // read to end of incoming data (should include everything for Process)
    let mut reader = BufReader::new(io::stdin());
    let mut buffer = String::new();
    reader.read_to_string(&mut buffer)?;
    // create a Process and insure it is valid (None means it was invalid)
    let process: Process;
    match Process::parse(buffer) {
        Some(p) => process = p,
        None => return Err(io::Error::from(io::ErrorKind::InvalidData)),
    }

    let plane = Plane::new(process.plane_location);
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
