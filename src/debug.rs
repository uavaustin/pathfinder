use super::*;

use std::error::Error;
use std::io::prelude::*;
use std::fs::File;
use std::path::Path;

impl PathFinder {
    pub fn export_obstacle_list_to_file(&mut self)
    {
        let path = Path::new("obstacle_list_export.txt");
        let display = path.display();

        let mut file = match File::create(&path) {
            Err(why) => panic!("couldn't create {}: {}",
                               display,
                               why.description()),
            Ok(file) => file,
        };

        for Node in &self.obstacle_list {
            let comma = ",";
            let dash = "-";
            let node_rep = format!("{}{}{}{}", Node.x.to_string(), comma, Node.y.to_string(), dash);

            match file.write_all(node_rep.as_bytes()) {
                Err(why) => {
                    panic!("couldn't write to {}: {}", display,
                                                       why.description())
                },
                Ok(_) => println!("successfully wrote to {}", display),
            }
        }
    }

    /*
    Takes in the paramters of the max X and max Y you want to see, and displays obstacles
    and non-obstacle nodes. Obstacle nodes are labeled "X" and non-obstacle nodes are
    labeled as ".".
    */
    pub fn draw(&self, x_min : i32, x_max : i32, y_min : i32, y_max : i32)
    {
        for y in y_min..y_max
        {
            for x in x_min..x_max
            {
                let current : Node = Node::new(x, y_max - y);
                //print!("XY {:?} {:?}", x, y);
                //println!("", );
                if self.obstacle_list.contains(&current)
                {
                    print!("X");
                }
                else {
                    print!(".");
                }
            }
            println!("",);
        }
    }
}
