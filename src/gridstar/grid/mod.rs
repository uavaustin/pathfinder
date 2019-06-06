impl Gridstar {
    fn populate_map(&mut self) {
        self.obstacle_list.clear();
        self.generate_fly_zone();
        self.generate_obstacles();
    }

    fn draw_line(
        &mut self,
        mut indep: f32,
        mut dep: f32,
        indep_goal: f32,
        dep_goal: f32,
        slope: f32,
        invert: bool,
    ) {
        const INCREMENT: f32 = 0.1;
        if indep > indep_goal {
            while indep > indep_goal && !(indep == indep_goal && dep == dep_goal) {
                indep -= INCREMENT;
                dep -= INCREMENT * slope;
                let buffer;
                if invert {
                    self.obstacle_list
                        .insert(Node::new(dep.floor() as i32, indep.floor() as i32));
                    buffer = Node::new(dep as i32, indep as i32 + 1);
                } else {
                    self.obstacle_list
                        .insert(Node::new(indep.floor() as i32, dep.floor() as i32));
                    buffer = Node::new(indep as i32 + 1, dep as i32);
                }
                self.obstacle_list.insert(buffer);
            }
        } else {
            while indep < indep_goal && !(indep == indep_goal && dep == dep_goal) {
                indep += INCREMENT;
                dep += INCREMENT * slope;
                let buffer;
                if invert {
                    self.obstacle_list
                        .insert(Node::new(dep.floor() as i32, indep.floor() as i32));
                    buffer = Node::new(dep as i32, indep as i32 - 1);
                } else {
                    self.obstacle_list
                        .insert(Node::new(indep.floor() as i32, dep.floor() as i32));
                    buffer = Node::new(indep as i32 - 1, dep as i32);
                }
                self.obstacle_list.insert(buffer);
            }
        }
    }

    fn generate_fly_zone(&mut self) {
        for i in 0..self.flyzones.len() {
            let flyzone_points = self.flyzones[i].clone();
            let mut pre_node: Node = flyzone_points[flyzone_points.len() - 1].to_node(&self);
            let mut end_node;

            for end_point in flyzone_points {
                end_node = end_point.to_node(&self);

                let slope = (end_node.y - pre_node.y) as f32 / (end_node.x - pre_node.x) as f32;
                if slope.abs() <= 1f32 {
                    self.draw_line(
                        pre_node.x as f32,
                        pre_node.y as f32,
                        end_node.x as f32,
                        end_node.y as f32,
                        slope,
                        false,
                    );
                } else {
                    self.draw_line(
                        pre_node.y as f32,
                        pre_node.x as f32,
                        end_node.y as f32,
                        end_node.x as f32,
                        1f32 / slope,
                        true,
                    );
                }
                pre_node = end_node;
            }
        }
    }

    fn generate_obstacles(&mut self) {
        for obst in &self.obstacles {
            let radius = ((obst.radius + self.buffer) / (self.grid_size)) as i32;
            let n = obst.coords.to_node(&self);

            for x in n.x - radius..n.x + radius {
                let dy = ((radius.pow(2) - (x - n.x).pow(2)) as f32).sqrt() as i32;
                self.obstacle_list.insert(Node::new(x, n.y + dy));
                self.obstacle_list.insert(Node::new(x, n.y + dy - 1));
                self.obstacle_list.insert(Node::new(x, n.y - dy));
                self.obstacle_list.insert(Node::new(x, n.y - dy - 1));
            }
            for y in n.y - radius..n.y + radius {
                let dx = ((radius.pow(2) - (y - n.y).pow(2)) as f32).sqrt() as i32;
                self.obstacle_list.insert(Node::new(n.x + dx, y));
                self.obstacle_list.insert(Node::new(n.x + dx - 1, y));
                self.obstacle_list.insert(Node::new(n.x - dx, y));
                self.obstacle_list.insert(Node::new(n.x - dx - 1, y));
            }
        }
    }
}
