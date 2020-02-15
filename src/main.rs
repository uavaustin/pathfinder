extern crate pathfinder;

use pathfinder::*;
use std::io;
use std::io::Read;

pub fn main() {
    let mut buffer = String::new();
    io::stdin().read_to_string(&mut buffer);
    let process = Process::parse(&buffer);
}
