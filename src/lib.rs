// lib.rs
// contains exposed functionality of the library
#![allow(dead_code)]
#![allow(unused_variables)]

extern crate wasm_bindgen;

pub mod obj;
pub mod tanstar;

mod algorithm;

pub use obj::*;
pub use tanstar::{TConfig, Tanstar};

use algorithm::Algorithm;
use std::collections::LinkedList;
use wasm_bindgen::prelude::*;

pub struct Pathfinder<A: Algorithm> {
    algo: A,
}

impl<A: Algorithm> Pathfinder<A> {
    pub fn new(
        mut algo: A,
        config: A::Config,
        flyzones: Vec<Vec<Location>>,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        algo.init(config, flyzones, obstacles);
        Self { algo }
    }

    pub fn get_adjust_path<T>(
        &mut self,
        plane: Plane,
        mut wp_list: LinkedList<Waypoint<T>>,
    ) -> LinkedList<Waypoint<T>> {
        let mut new_wp_list = LinkedList::new();
        let mut current_loc = plane.location;

        while let Some(current_wp) = wp_list.pop_front() {
            let next_loc = current_wp.location;

            if let Some(mut path) = self.algo.adjust_path::<T>(current_loc, next_loc) {
                println!("appending");
                new_wp_list.append(&mut path);
            } else {
                println!("no path");
                break;
            }

            current_loc = current_wp.location;
            new_wp_list.push_back(current_wp);
            // self.wp_list.push_back(self.current_wp.clone()); // Push original waypoint
            // self.wp_list.push_back(Waypoint::from_degrees(0, 30.69, -97.69, 100f32, 10f32));
        }

        new_wp_list
    }

    pub fn set_config(&mut self, config: A::Config) {
        self.algo.set_config(config);
    }

    pub fn set_flyzone(&mut self, flyzone: Vec<Vec<Location>>) {
        self.algo.set_flyzone(flyzone);
    }

    pub fn set_obstacles(&mut self, obstacles: Vec<Obstacle>) {
        self.algo.set_obstacles(obstacles);
    }

    pub fn get_config(&self) -> &A::Config {
        self.algo.get_config()
    }

    pub fn get_flyzone(&mut self) -> &Vec<Vec<Location>> {
        self.algo.get_flyzone()
    }

    pub fn get_obstacle(&self) -> &Vec<Obstacle> {
        self.algo.get_obstacles()
    }
}

/// A struct imported from Javascript for wrapping [`Location`]
#[wasm_bindgen(module = "/src/wrap/pfwrapper.js")]
extern "C" {
    pub type LocationWrapper;

    /// Gets the latitude in degrees
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_lat(this: &LocationWrapper) -> f64;

    /// Gets the longitude in degrees
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_lon(this: &LocationWrapper) -> f64;

    /// Gets the altitude in meters
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_alt(this: &LocationWrapper) -> f32;
}

// Location can't be wasm-bindgenified directly
impl Into<Location> for LocationWrapper {
    fn into(self) -> Location {
        Location::from_degrees(self.get_lat(), self.get_lon(), self.get_alt())
    }
}

/// A struct imported from Javascript for wrapping [`Obstacle`]
#[wasm_bindgen(module = "/src/wrap/pfwrapper.js")]
extern "C" {
    pub type ObstacleWrapper;

    /// Gets the location as a [`LocationWrapper`]
    #[wasm_bindgen(method, getter, js_name = location)]
    // #[wasm_bindgen(method)]
    pub fn get_obstacle_location(this: &ObstacleWrapper) -> LocationWrapper;

    /// Gets the radius in meters
    #[wasm_bindgen(method, getter, js_name = radius)]
    // #[wasm_bindgen(method)]
    pub fn get_obstacle_radius(this: &ObstacleWrapper) -> f32;

    /// Gets the height in meters
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_height(this: &ObstacleWrapper) -> f32;
}

// Obstacle can't be wasm-bindgenified directly
impl Into<Obstacle> for ObstacleWrapper {
    fn into(self) -> Obstacle {
        Obstacle::new(
            self.get_obstacle_location().into(),
            self.get_obstacle_radius(),
            self.get_height()
        )
    }
}

/// A struct imported from Javascript for wrapping [`Plane`]
#[wasm_bindgen(module = "/src/wrap/pfwrapper.js")]
extern "C" {
    pub type PlaneWrapper;

    /// Gets the location as a [`LocationWrapper`]
    #[wasm_bindgen(method, getter, js_name = location)]
    // #[wasm_bindgen(method)]
    pub fn get_plane_location(this: &PlaneWrapper) -> LocationWrapper;

    /// Gets the yaw in degrees, -1 if not provided
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_yaw(this: &PlaneWrapper) -> f32;

    /// Gets the pitch in degrees, -1 if not provided
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_pitch(this: &PlaneWrapper) -> f32;

    /// Gets the roll in degrees, -1 if not provided
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_roll(this: &PlaneWrapper) -> f32;

    /// Gets the airspeed in meters per second, -1 if not provided
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_airspeed(this: &PlaneWrapper) -> f32;

    /// Gets the groundspeed in meters per second, -1 if not provided
    #[wasm_bindgen(method, getter)]
    // #[wasm_bindgen(method)]
    pub fn get_groundspeed(this: &PlaneWrapper) -> f32;

    /// Gets the wind_dir in degrees, -1 if not provided
    #[wasm_bindgen(method, getter, js_name = windDir)]
    // #[wasm_bindgen(method)]
    pub fn get_wind_dir(this: &PlaneWrapper) -> f32;
}

// Plane can't be wasm-bindgenified directly
impl Into<Plane> for PlaneWrapper {
    fn into(self) -> Plane {
        Plane {
            location: self.get_plane_location().into(),
            yaw: self.get_yaw(),
            pitch: self.get_pitch(),
            roll: self.get_roll(),
            airspeed: self.get_airspeed(),
            groundspeed: self.get_groundspeed(),
            wind_dir: self.get_wind_dir()
        }
    }
}

/// A struct imported from Javascript for wrapping [`TConfig`]
#[wasm_bindgen(module = "/src/wrap/pfwrapper.js")]
extern "C" {
    pub type TConfigWrapper;

    /// Get the buffer around obstacles in meters
    #[wasm_bindgen(method, getter, js_name = bufferSize)]
    // #[wasm_bindgen(method)]
    pub fn get_buffer_size(this: &TConfigWrapper) -> f32;

    /// Get the maximum process time allowed in seconds
    #[wasm_bindgen(method, getter, js_name = maxProcessTime)]
    // #[wasm_bindgen(method)]
    pub fn get_max_process_time(this: &TConfigWrapper) -> f32;

    /// Get the turning radius of the plane in meters
    #[wasm_bindgen(method, getter, js_name = turningRadius)]
    // #[wasm_bindgen(method)]
    pub fn get_turning_radius(this: &TConfigWrapper) -> f32;

    /// Get the merge threshold for vertices
    ///
    /// Vertices within this threshold will be merged into one.
    #[wasm_bindgen(method, getter, js_name = vertexMergeThreshold)]
    // #[wasm_bindgen(method)]
    pub fn get_vertex_merge_threshold(this: &TConfigWrapper) -> f32;

    /// Whether to generate virtual nodes for flyzones
    #[wasm_bindgen(method, getter, js_name = virtualizeFlyzone)]
    // #[wasm_bindgen(method)]
    pub fn get_virtualize_flyzone(this: &TConfigWrapper) -> bool;
}

// TConfig can't be wasm-bindgenified directly
impl Into<TConfig> for TConfigWrapper {
    fn into(self) -> TConfig {
        TConfig::new(
            self.get_buffer_size(),
            std::time::Duration::from_secs_f32(self.get_max_process_time()),
            self.get_turning_radius(),
            self.get_vertex_merge_threshold(),
            self.get_virtualize_flyzone()
        )
    }
}

/// A struct imported from Javascript for wrapping [`Waypoint`]
///
/// Since `wasm-bindgen` currently doesn't support generic structures the wrapped `Waypoint` is
/// given a null type.
#[wasm_bindgen(module = "/src/wrap/pfwrapper.js")]
extern "C" {
    pub type WaypointWrapper;

    /// Gets the location as a [`LocationWrapper`]
    #[wasm_bindgen(method, getter, js_name = location)]
    // #[wasm_bindgen(method)]
    pub fn get_waypoint_location(this: &WaypointWrapper) -> LocationWrapper;

    /// Gets the radius in meters
    #[wasm_bindgen(method, getter, js_name = radius)]
    // #[wasm_bindgen(method)]
    pub fn get_waypoint_radius(this: &WaypointWrapper) -> f32;

    // data is omitted due to its lack of use
}

// Waypoint can't be wasm-bindgenified directly
impl Into<Waypoint<()>> for WaypointWrapper {
    fn into(self) -> Waypoint<()> {
        Waypoint::new(
            self.get_waypoint_location().into(),
            self.get_waypoint_radius()
        )
    }
}

/// A wrapper for `wasm-bindgen` to interact with
///
/// Due to `wasm-bindgen`'s current inability to handle generic structures, the `pathfinder`
/// and implementation definitions must be changed when the Pathfinder implementation changes.
#[wasm_bindgen]
pub struct PathfinderWrapper {
    pathfinder: Pathfinder<Tanstar>
}

#[wasm_bindgen]
impl PathfinderWrapper {
    // Signature:
    // mut algo: Tanstar,
    // config: Tanstar::Config,
    // flyzones: Vec<Vec<Location>>,
    // obstacles: Vec<Obstacle>,
    // ) -> Self {
    #[wasm_bindgen(constructor)]
    pub fn new(
        mut algo: Tanstar,
        config: TConfigWrapper,
        flyzones: js_sys::Array,
        obstacles: js_sys::Array,
    ) -> Self {
        unimplemented!();
    }

    // Signature:
    // &mut self,
    // plane: Plane,
    // mut wp_list: LinkedList<Waypoint<()>>,
    // ) -> LinkedList<Waypoint<()>> {
    #[wasm_bindgen(method, js_name = getAdjustPath)]
    pub fn get_adjust_path(
        &mut self,
        plane: PlaneWrapper,
        mut wp_list: js_sys::Array,
    ) -> js_sys::Array {
        unimplemented!();
    }

    #[wasm_bindgen(setter)]
    pub fn set_config(&mut self, config: TConfigWrapper) {
        unimplemented!();
    }

    // Signature:
    // &mut self, flyzone: Vec<Vec<Location>>) {
    #[wasm_bindgen(setter)]
    pub fn set_flyzone(&mut self, flyzone: js_sys::Array) {
        unimplemented!();
    }

    // Signature:
    // &mut self, obstacles: Vec<Obstacle>) {
    #[wasm_bindgen(setter)]
    pub fn set_obstacles(&mut self, obstacles: js_sys::Array) {
        unimplemented!();
    }

    #[wasm_bindgen(getter)]
    pub fn get_config(&self) -> TConfigWrapper {
        unimplemented!();
    }

    // Signature:
    // &mut self) -> &Vec<Vec<Location>> {
    #[wasm_bindgen(getter)]
    pub fn get_flyzone(&mut self) -> js_sys::Array {
        unimplemented!();
    }

    // Signature:
    // &self) -> &Vec<Obstacle> {
    #[wasm_bindgen(getter)]
    pub fn get_obstacle(&self) -> js_sys::Array {
        unimplemented!();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn tanstar_invalid_flyzones_test() {
        Pathfinder::new(Tanstar::new(), TConfig::default(), vec![], Vec::new());
    }

    #[test]
    #[should_panic]
    fn tanstar_invalid_flyzone_test() {
        Pathfinder::new(Tanstar::new(), TConfig::default(), vec![vec![]], Vec::new());
    }
}
