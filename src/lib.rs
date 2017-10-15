#![allow(dead_code)]

mod obj;
use obj::*;

impl PathFinder{
    pub fn new() -> PathFinder {
        PathFinder {
            delta_x: 1,
            buffer: 1,
            max_process_time: 10,
            plane: Plane::new(0.0,0.0,0.0),
            obstacle_list: Vec::new(),
            wp_list: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
	#[test]
	fn test() {

	}
}
