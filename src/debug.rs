// Adapted from conrod/example/canvas.rs
// https://github.com/PistonDevelopers/conrod/blob/master/examples/canvas.rs

use std::collections::HashSet;

use node::Node;

use super::conrod::backend::glium::glium;
use super::conrod::backend::glium::glium::Surface;
use super::conrod::{self, color, widget, Positionable, Sizeable, Widget};

pub struct Debugger {
    width: u32,
    height: u32,
    obstacles: HashSet<Node>,
    path: HashSet<Node>,
    explored: HashSet<Node>,
    wps: HashSet<Node>,
}

impl Debugger {
    pub fn new() -> Debugger {
        Debugger {
            width: 0,
            height: 0,
            obstacles: HashSet::new(),
            path: HashSet::new(),
            explored: HashSet::new(),
            wps: HashSet::new(),
        }
    }

    pub fn set_size(&mut self, w: u32, h: u32) {
        self.width = w;
        self.height = h;
    }

    pub fn set_obstacles(&mut self, obstacles: HashSet<Node>) {
        self.obstacles = obstacles;
    }

    pub fn add_to_path(&mut self, node: Node) {
        self.path.insert(node);
    }

    pub fn add_to_explored(&mut self, node: Node) {
        self.explored.insert(node);
    }

    pub fn add_to_wp(&mut self, node: Node) {
        self.wps.insert(node);
    }

    pub fn remove_from_wp(&mut self, node: &Node) {
        self.wps.remove(node);
    }

    pub fn draw(&self) {
        const WIDTH: u32 = 1600;
        const HEIGHT: u32 = 1000;

        // Build the window.
        let events_loop = glium::glutin::EventsLoop::new();
        let window = glium::glutin::WindowBuilder::new()
            .with_title("Canvas")
            .with_dimensions(WIDTH, HEIGHT);
        let context = glium::glutin::ContextBuilder::new()
            .with_vsync(true)
            .with_multisampling(4);
        let display = glium::Display::new(window, context, &events_loop).unwrap();

        // construct our `Ui`.
        let mut ui = conrod::UiBuilder::new([WIDTH as f64, HEIGHT as f64]).build();

        // A type used for converting `conrod::render::Primitives` into `Command`s that can be used
        // for drawing to the glium `Surface`.
        let mut renderer = conrod::backend::glium::Renderer::new(&display).unwrap();

        // The image map describing each of our widget->image mappings (in our case, none).
        let image_map = conrod::image::Map::<glium::texture::Texture2d>::new();

        // Generate a unique `WidgetId` for each widget.
        widget_ids! {
            struct Ids {
                canvas,
                matrix
            }
        }

        // Instantiate the generated list of widget identifiers.
        let ids = &mut Ids::new(ui.widget_id_generator());

        'main: loop {
            {
                let ui_cell = &mut ui.set_widgets();
                // Instantiate all widgets in the GUI.
                widget::Canvas::new().set(ids.canvas, ui_cell);

                let canvas_wh = ui_cell.wh_of(ids.canvas).unwrap();
                let mut elements = widget::Matrix::new(self.width as usize, self.height as usize)
                    .w_h(canvas_wh[0], canvas_wh[1])
                    .cell_padding(0.2f64, 0.2f64)
                    .mid_top_of(ids.canvas)
                    .set(ids.matrix, ui_cell);
                while let Some(elem) = elements.next(ui_cell) {
                    let cell;
                    let node = Node::new(elem.row as i32, elem.col as i32);

                    if self.obstacles.contains(&node) {
                        cell = widget::Rectangle::fill_with([elem.w, elem.h], color::BLACK);
                    } else if self.wps.contains(&node) {
                        cell = widget::Rectangle::fill_with([elem.w, elem.h], color::RED);
                    } else if self.path.contains(&node) {
                        cell = widget::Rectangle::fill_with([elem.w, elem.h], color::GREEN);
                    } else if self.explored.contains(&node) {
                        cell = widget::Rectangle::fill_with([elem.w, elem.h], color::BLUE);
                    } else {
                        cell = widget::Rectangle::fill_with([elem.w, elem.h], color::WHITE);
                    }

                    elem.set(cell, ui_cell);
                }
            }

            // Render the `Ui` and then display it on the screen.
            if let Some(primitives) = ui.draw_if_changed() {
                renderer.fill(&display, primitives, &image_map);
                let mut target = display.draw();
                target.clear_color(0.0, 0.0, 0.0, 1.0);
                renderer.draw(&display, &mut target, &image_map).unwrap();
                target.finish().unwrap();
            }
        }
    }
}
