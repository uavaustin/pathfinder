// Adapted from conrod/example/canvas.rs
// https://github.com/PistonDevelopers/conrod/blob/master/examples/canvas.rs
use std::collections::HashSet;
use std::io::{self, Read, Write};
use std::process::{Command, Stdio};
use std::str;

use node::Node;

use super::conrod::backend::glium::glium;
use super::conrod::backend::glium::glium::Surface;
use super::conrod::{self, color, widget, Positionable, Widget};

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
        const DEFAULT_WIDTH: u32 = 1920;
        const DEFAULT_HEIGHT: u32 = 1080;
        let resolution = Debugger::get_resolution().unwrap_or((DEFAULT_WIDTH, DEFAULT_HEIGHT));

        let width = (self.width * 10).min(resolution.0);
        let height = (self.height * 10).min(resolution.1);

        // Build the window.
        let mut events_loop = glium::glutin::EventsLoop::new();
        let window = glium::glutin::WindowBuilder::new()
            .with_title("Canvas")
            .with_dimensions(width, height);
        let context = glium::glutin::ContextBuilder::new()
            .with_vsync(true)
            .with_multisampling(4);
        let display = glium::Display::new(window, context, &events_loop).unwrap();

        // construct our `Ui`.
        let mut ui = conrod::UiBuilder::new([width as f64, height as f64]).build();

        // A type used for converting `conrod::render::Primitives` into `Command`s that can be used
        // for drawing to the glium `Surface`.
        let mut renderer = conrod::backend::glium::Renderer::new(&display).unwrap();

        // The image map describing each of our widget->image mappings (in our case, none).
        let image_map = conrod::image::Map::<glium::texture::Texture2d>::new();

        // Generate a unique `WidgetId` for each widget.
        widget_ids! {
            struct Ids {
                matrix
            }
        }

        // Instantiate the generated list of widget identifiers.
        let ids = &mut Ids::new(ui.widget_id_generator());
        let mut events = Vec::new();

        'main: loop {
            events.clear();
            // Event handler
            events_loop.poll_events(|event| {
                events.push(event);
            });

            // If there are no new events, wait for one.
            if events.is_empty() {
                events_loop.run_forever(|event| {
                    events.push(event);
                    glium::glutin::ControlFlow::Break
                });
            }

            for event in events.drain(..) {
                // Break from the loop upon `Escape` or closed window.
                match event.clone() {
                    glium::glutin::Event::WindowEvent { event, .. } => match event {
                        glium::glutin::WindowEvent::Closed
                        | glium::glutin::WindowEvent::KeyboardInput {
                            input:
                                glium::glutin::KeyboardInput {
                                    virtual_keycode: Some(glium::glutin::VirtualKeyCode::Escape),
                                    ..
                                },
                            ..
                        } => break 'main,
                        _ => (),
                    },
                    _ => (),
                };
                // Use the `winit` backend feature to convert the winit event to a conrod input.
                let input = match conrod::backend::winit::convert_event(event, &display) {
                    None => continue,
                    Some(input) => input,
                };

                // Handle the input with the `Ui`.
                ui.handle_event(input);

                let ui = &mut ui.set_widgets();
                // Instantiate all widgets in the GUI.
                let mut elements = widget::Matrix::new(self.width as usize, self.height as usize)
                    .cell_padding(0.2f64, 0.2f64)
                    .middle_of(ui.window)
                    .set(ids.matrix, ui);
                while let Some(elem) = elements.next(ui) {
                    let cell;
                    let node = Node::new(elem.col as i32, self.height as i32 - elem.row as i32);

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

                    elem.set(cell, ui);
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

    // #TODO use custom error wrapper for cleaner error handling
    fn get_resolution() -> Result<(u32, u32), io::Error> {
        let mut xdpyinfo_cmd = Command::new("xdpyinfo").stdout(Stdio::piped()).spawn()?;

        let mut awk_cmd = Command::new("awk")
            .arg("/dimensions/{print $2}")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;

        if let (Some(stdout), Some(stdin)) = (&mut xdpyinfo_cmd.stdout, &mut awk_cmd.stdin) {
            let mut buf: Vec<u8> = Vec::new();
            stdout.read_to_end(&mut buf)?;
            stdin.write_all(&buf)?;
        } else {
            return Err(io::Error::last_os_error());
        }

        let res = awk_cmd.wait_with_output()?.stdout;
        let res_str = match str::from_utf8(&res) {
            Ok(res) => res,
            Err(_) => return Err(io::Error::last_os_error()),
        };
        let resolutions: Vec<&str> = res_str.trim().split('x').collect();
        match (
            str::parse::<u32>(resolutions[0]),
            str::parse::<u32>(resolutions[1]),
        ) {
            (Ok(w), Ok(h)) => Ok((w, h)),
            _ => Err(io::Error::last_os_error()),
        }
    }
}
