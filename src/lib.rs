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

    /// Creates a [`LocationWrapper`]
    #[wasm_bindgen(constructor)]
    pub fn new_location_wrapper(lat: f64, lon: f64, alt: f32) -> LocationWrapper;

    /// Gets the latitude in degrees
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getLat)]
    pub fn get_lat(this: &LocationWrapper) -> f64;

    /// Gets the longitude in degrees
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getLon)]
    pub fn get_lon(this: &LocationWrapper) -> f64;

    /// Gets the altitude in meters
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getAlt)]
    pub fn get_alt(this: &LocationWrapper) -> f32;
}

// Location can't be directly wrapped
impl From<Location> for LocationWrapper {
    fn from(location: Location) -> Self {
        LocationWrapper::new_location_wrapper(
            location.lat_degree(),
            location.lon_degree(),
            location.alt(),
        )
    }
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

    /// Creates an [`ObstacleWrapper`]
    #[wasm_bindgen(constructor)]
    pub fn new_obstacle_wrapper(
        location: LocationWrapper,
        radius: f32,
        height: f32,
    ) -> ObstacleWrapper;

    /// Gets the location as a [`LocationWrapper`]
    // #[wasm_bindgen(method, getter, js_name = location)]
    #[wasm_bindgen(method, js_name = getLocation)]
    pub fn get_obstacle_location(this: &ObstacleWrapper) -> LocationWrapper;

    /// Gets the radius in meters
    // #[wasm_bindgen(method, getter, js_name = radius)]
    #[wasm_bindgen(method, js_name = getRadius)]
    pub fn get_obstacle_radius(this: &ObstacleWrapper) -> f32;

    /// Gets the height in meters
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getHeight)]
    pub fn get_height(this: &ObstacleWrapper) -> f32;
}

// Obstacle can't be wrapped directly
impl From<Obstacle> for ObstacleWrapper {
    fn from(obstacle: Obstacle) -> Self {
        ObstacleWrapper::new_obstacle_wrapper(
            obstacle.location.into(),
            obstacle.radius,
            obstacle.height,
        )
    }
}

// Obstacle can't be wasm-bindgenified directly
impl Into<Obstacle> for ObstacleWrapper {
    fn into(self) -> Obstacle {
        Obstacle::new(
            self.get_obstacle_location().into(),
            self.get_obstacle_radius(),
            self.get_height(),
        )
    }
}

/// A struct imported from Javascript for wrapping [`Plane`]
#[wasm_bindgen(module = "/src/wrap/pfwrapper.js")]
extern "C" {
    pub type PlaneWrapper;

    /// Gets the location as a [`LocationWrapper`]
    // #[wasm_bindgen(method, getter, js_name = location)]
    #[wasm_bindgen(method, js_name = getLocation)]
    pub fn get_plane_location(this: &PlaneWrapper) -> LocationWrapper;

    /// Gets the yaw in degrees, -1 if not provided
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getYaw)]
    pub fn get_yaw(this: &PlaneWrapper) -> f32;

    /// Gets the pitch in degrees, -1 if not provided
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getPitch)]
    pub fn get_pitch(this: &PlaneWrapper) -> f32;

    /// Gets the roll in degrees, -1 if not provided
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getRoll)]
    pub fn get_roll(this: &PlaneWrapper) -> f32;

    /// Gets the airspeed in meters per second, -1 if not provided
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name= getAirspeed)]
    pub fn get_airspeed(this: &PlaneWrapper) -> f32;

    /// Gets the groundspeed in meters per second, -1 if not provided
    // #[wasm_bindgen(method, getter)]
    #[wasm_bindgen(method, js_name = getGroundspeed)]
    pub fn get_groundspeed(this: &PlaneWrapper) -> f32;

    /// Gets the wind_dir in degrees, -1 if not provided
    // #[wasm_bindgen(method, getter, js_name = windDir)]
    #[wasm_bindgen(method, js_name = getWindDir)]
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
            wind_dir: self.get_wind_dir(),
        }
    }
}

/// A struct imported from Javascript for wrapping [`TConfig`]
#[wasm_bindgen(module = "/src/wrap/pfwrapper.js")]
extern "C" {
    pub type TConfigWrapper;

    /// Create a [`TConfigWrapper`]
    #[wasm_bindgen(constructor)]
    pub fn new_tconfig_wrapper(
        buffer_size: f32,
        max_process_time: f32,
        turning_radius: f32,
        vertex_merge_threshold: f32,
        virtualize_flyzone: bool,
    ) -> TConfigWrapper;

    /// Get the buffer around obstacles in meters
    // #[wasm_bindgen(method, getter, js_name = bufferSize)]
    #[wasm_bindgen(method, js_name = getBufferSize)]
    pub fn get_buffer_size(this: &TConfigWrapper) -> f32;

    /// Get the maximum process time allowed in seconds
    // #[wasm_bindgen(method, getter, js_name = maxProcessTime)]
    #[wasm_bindgen(method, js_name = getMaxProcessTime)]
    pub fn get_max_process_time(this: &TConfigWrapper) -> f32;

    /// Get the turning radius of the plane in meters
    // #[wasm_bindgen(method, getter, js_name = turningRadius)]
    #[wasm_bindgen(method, js_name = getTurningRadius)]
    pub fn get_turning_radius(this: &TConfigWrapper) -> f32;

    /// Get the merge threshold for vertices
    ///
    /// Vertices within this threshold will be merged into one.
    // #[wasm_bindgen(method, getter, js_name = vertexMergeThreshold)]
    #[wasm_bindgen(method, js_name = getVertexMergeThreshold)]
    pub fn get_vertex_merge_threshold(this: &TConfigWrapper) -> f32;

    /// Whether to generate virtual nodes for flyzones
    // #[wasm_bindgen(method, getter, js_name = virtualizeFlyzone)]
    #[wasm_bindgen(method, js_name = getVirtualizeFlyzone)]
    pub fn get_virtualize_flyzone(this: &TConfigWrapper) -> bool;
}

// TConfig can't be directly wrapped
impl From<TConfig> for TConfigWrapper {
    fn from(config: TConfig) -> Self {
        TConfigWrapper::new_tconfig_wrapper(
            config.buffer_size,
            config.max_process_time.as_secs_f32(),
            config.turning_radius,
            config.vertex_merge_threshold,
            config.virtualize_flyzone,
        )
    }
}

// TConfig can't be wasm-bindgenified directly
impl Into<TConfig> for TConfigWrapper {
    fn into(self) -> TConfig {
        TConfig::new(
            self.get_buffer_size(),
            std::time::Duration::from_secs_f32(self.get_max_process_time()),
            self.get_turning_radius(),
            self.get_vertex_merge_threshold(),
            self.get_virtualize_flyzone(),
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

    /// Creates a [`WaypointWrapper`]
    #[wasm_bindgen(constructor)]
    pub fn new_waypoint_wrapper(location: LocationWrapper, radius: f32) -> WaypointWrapper;

    /// Gets the location as a [`LocationWrapper`]
    // #[wasm_bindgen(method, getter, js_name = location)]
    #[wasm_bindgen(method, js_name = getLocation)]
    pub fn get_waypoint_location(this: &WaypointWrapper) -> LocationWrapper;

    /// Gets the radius in meters
    // #[wasm_bindgen(method, getter, js_name = radius)]
    #[wasm_bindgen(method, js_name = getRadius)]
    pub fn get_waypoint_radius(this: &WaypointWrapper) -> f32;

// data is omitted due to its lack of use
}

// Waypoint can't be directly wrapped
impl From<Waypoint<()>> for WaypointWrapper {
    fn from(waypoint: Waypoint<()>) -> Self {
        WaypointWrapper::new_waypoint_wrapper(waypoint.location.into(), waypoint.radius)
    }
}

// Waypoint can't be wasm-bindgenified directly
impl Into<Waypoint<()>> for WaypointWrapper {
    fn into(self) -> Waypoint<()> {
        Waypoint::new(
            self.get_waypoint_location().into(),
            self.get_waypoint_radius(),
        )
    }
}

/// A wrapper for `wasm-bindgen` to interact with
///
/// Due to `wasm-bindgen`'s current inability to handle generic structures, the `pathfinder`
/// and implementation definitions must be changed when the Pathfinder implementation changes.
#[wasm_bindgen]
pub struct PathfinderWrapper {
    pathfinder: Pathfinder<Tanstar>,
}

#[wasm_bindgen]
// TODO: Return Result wrapper to allow for Node.js error handling
impl PathfinderWrapper {
    // Signature:
    // mut algo: Tanstar,
    // config: TConfig,
    // flyzones: Vec<Vec<Location>>,
    // obstacles: Vec<Obstacle>,
    // ) -> Self {
    #[wasm_bindgen(constructor)]
    pub fn new(
        algo: Tanstar,
        config: TConfigWrapper,
        flyzones: js_sys::Array,
        obstacles: js_sys::Array,
    ) -> Self {
        // TODO: Check if unwrapable
        // Unwrap flyzones without checking
        let flyzones_vec = flyzones
            .to_vec()
            .iter()
            .map(|fz_arr| {
                js_sys::Array::from(fz_arr)
                    .to_vec()
                    .into_iter()
                    .map(|lw| LocationWrapper::from(lw).into())
                    .collect()
            })
            .collect();

        // TODO: Check if unwrapable
        // Unwrap obstacles without checking
        let obstacles_vec = obstacles
            .to_vec()
            .into_iter()
            .map(|ow| ObstacleWrapper::from(ow).into())
            .collect();

        PathfinderWrapper {
            pathfinder: Pathfinder::new(algo, config.into(), flyzones_vec, obstacles_vec),
        }
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
        wp_list: js_sys::Array,
    ) -> js_sys::Array {
        // TODO: Check if unwrapable
        // Unwrap wp_list without checking
        let wp_linked_list = wp_list
            .to_vec()
            .into_iter()
            .map(|ww| WaypointWrapper::from(ww).into())
            .collect();

        self.pathfinder
            .get_adjust_path(plane.into(), wp_linked_list)
            .into_iter()
            .map(WaypointWrapper::from)
            .collect()
    }

    // #[wasm_bindgen(setter)]
    #[wasm_bindgen(method, js_name = setConfig)]
    pub fn set_config(&mut self, config: TConfigWrapper) {
        self.pathfinder.set_config(config.into());
    }

    // Signature:
    // &mut self, flyzone: Vec<Vec<Location>>) {
    // #[wasm_bindgen(setter)]
    #[wasm_bindgen(method, js_name = setFlyzone)]
    pub fn set_flyzone(&mut self, flyzone: js_sys::Array) {
        // TODO: Check if unwrapable
        // Unwrap flyzone without checking
        let flyzones_vec = flyzone
            .to_vec()
            .iter()
            .map(|fz_arr| {
                js_sys::Array::from(fz_arr)
                    .to_vec()
                    .into_iter()
                    .map(|lw| LocationWrapper::from(lw).into())
                    .collect()
            })
            .collect();

        self.pathfinder.set_flyzone(flyzones_vec);
    }

    // Signature:
    // &mut self, obstacles: Vec<Obstacle>) {
    // #[wasm_bindgen(setter)]
    #[wasm_bindgen(method, js_name = setObstacles)]
    pub fn set_obstacles(&mut self, obstacles: js_sys::Array) {
        // TODO: Check if unwrapable
        // Unwrap obstacles without checking
        let obstacles_vec = obstacles
            .to_vec()
            .into_iter()
            .map(|ow| ObstacleWrapper::from(ow).into())
            .collect();

        self.pathfinder.set_obstacles(obstacles_vec);
    }

    // #[wasm_bindgen(getter)]
    #[wasm_bindgen(getter, js_name = getConfig)]
    pub fn get_config(&self) -> TConfigWrapper {
        self.pathfinder.get_config().clone().into()
    }

    // Signature:
    // &mut self) -> &Vec<Vec<Location>> {
    // #[wasm_bindgen(getter)]
    #[wasm_bindgen(method, js_name = getFlyzone)]
    pub fn get_flyzone(&mut self) -> js_sys::Array {
        // Convert flyzone into Array of Array of LocationWrappers
        self.pathfinder
            .get_flyzone()
            .iter()
            .map(|flyzone| {
                flyzone
                    .iter()
                    .map(|loc| LocationWrapper::from(loc.clone()))
                    .collect::<js_sys::Array>()
            })
            .collect()
    }

    // Signature:
    // &self) -> &Vec<Obstacle> {
    // #[wasm_bindgen(getter)]
    #[wasm_bindgen(method, js_name = getObstacle)]
    pub fn get_obstacle(&self) -> js_sys::Array {
        self.pathfinder
            .get_obstacle()
            .iter()
            .map(|obs| ObstacleWrapper::from(obs.clone()))
            .collect()
    }
}

// DEBUG
// Allows for panics to be sent to Javascript
// #[wasm_bindgen(method)]
// pub fn init_panic_hook() {
//     extern crate console_error_panic_hook;
//     console_error_panic_hook::set_once();
// }

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

#[cfg(test)]
pub mod wasm_bindgen_tests {
    extern crate wasm_bindgen_test;

    use self::wasm_bindgen_test::*;
    use super::*;

    #[wasm_bindgen_test]
    fn location_wrapper_from_location_test() {
        let location = Location::from_degrees(-1.0, 0.0, 1.0);
        let location_wrapper_into: LocationWrapper = location.into();
        let location_wrapper_from = LocationWrapper::from(location);
        assert_eq!(location.lat_degree(), location_wrapper_into.get_lat());
        assert_eq!(location.lon_degree(), location_wrapper_into.get_lon());
        assert_eq!(location.alt(), location_wrapper_into.get_alt());
        assert_eq!(location.lat_degree(), location_wrapper_from.get_lat());
        assert_eq!(location.lon_degree(), location_wrapper_from.get_lon());
        assert_eq!(location.alt(), location_wrapper_from.get_alt());
    }

    #[wasm_bindgen_test]
    fn location_wrapper_into_location_test() {
        let location_wrapper = LocationWrapper::new_location_wrapper(-1.0, 0.0, 1.0);
        let location_wrapper_to_convert = LocationWrapper::new_location_wrapper(
            location_wrapper.get_lat(),
            location_wrapper.get_lon(),
            location_wrapper.get_alt(),
        ); // into() moves the value and wasm-bindgen JavaScript types don't clone() to the same type
        let location: Location = location_wrapper_to_convert.into();
        assert_eq!(location_wrapper.get_lat(), location.lat_degree());
        assert_eq!(location_wrapper.get_lon(), location.lon_degree());
        assert_eq!(location_wrapper.get_alt(), location.alt());
    }

    #[wasm_bindgen_test]
    fn obstacle_wrapper_into_obstacle_test() {
        let location = LocationWrapper::new_location_wrapper(-1.0, 5.0, 1.0);
        let lat = location.get_lat();
        let lon = location.get_lon();
        let alt = location.get_alt();

        let obstacle_wrapper = ObstacleWrapper::new_obstacle_wrapper(location, 0.1, 10.0);
        let radius = obstacle_wrapper.get_obstacle_radius();
        let height = obstacle_wrapper.get_height();

        let obstacle: Obstacle = obstacle_wrapper.into();
        let obs_location = obstacle.location;
        assert_eq!(lat, obs_location.lat_degree());
        assert_eq!(lon, obs_location.lon_degree());
        assert_eq!(alt, obs_location.alt());
        assert_eq!(radius, obstacle.radius);
        assert_eq!(height, obstacle.height);
    }
}
