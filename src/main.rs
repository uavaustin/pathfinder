extern crate pathfinder;
extern crate protobuf;

use pathfinder::*;
use process::process::Process;
use protobuf::*;
use std::collections::linked_list::LinkedList;
use std::io;
use std::io::{BufReader, BufWriter};

pub fn main() {
    // handle request
    let process;
    // scream and exit if there is an error, otherwise continue
    match retrieve_request() {
        Ok(p) => process = p,
        Err(e) => {
            eprintln!("{}", e);
            return;
        }
    }

    // get adjusted path
    let mut pathfinder = Pathfinder::new(
        Tanstar::new(),
        process.config,
        process.flyzones,
        process.obstacles,
    );
    let result = pathfinder.get_adjust_path(process.plane.clone(), process.waypoints);

    // scream if there is an error
    match send_response(result) {
        Err(e) => eprintln!("{}", e),
        _ => (),
    }
}

fn retrieve_request() -> ProtobufResult<Process> {
    let mut reader = BufReader::new(io::stdin());
    let cis = &mut CodedInputStream::new(reader.get_mut());
    // return process results (Ok(Process) if successful, Err(ProtobufError) otherwise)
    Process::parse(cis)
}

fn send_response(result: LinkedList<Waypoint<()>>) -> ProtobufResult<()> {
    let response = create_response(&result)?;

    let mut writer = BufWriter::new(io::stdout());
    let mut cos = CodedOutputStream::new(writer.get_mut());
    // return the result of sending the message
    cos.write_message_no_tag(&response)
}

fn create_response(waypoints: &LinkedList<Waypoint<()>>) -> ProtobufResult<PathResponse> {
    let mut response = PathResponse::new();

    let mut response_waypoints = Vec::new();
    for wp in waypoints {
        // create response Point from waypoint position
        let mut point = NSFW_Point::new();
        point.set_lon(wp.location.lon_degree());
        point.set_lat(wp.location.lat_degree());
        point.set_alt(wp.location.alt().into());

        response_waypoints.push(point);
    }

    response.set_newWaypoints(RepeatedField::from_vec(response_waypoints));

    Ok(response)
}
