// Config struct for tanstar
use std::time::Duration;

pub const DEFAULT_BUFFER_SIZE: f32 = 2f32;
pub const DEFAULT_PROCESS_TIME: u64 = 10u64;
pub const DEFAULT_TURNING_RADIUS: f32 = 5f32;
pub const DEFAULT_V_MERGE_THRESHOLD: f32 = 5f32;

#[derive(Clone, Debug)]
pub struct TConfig {
    // buffer around obstacles, in meters
    pub buffer_size: f32,
    // maximum procssed time allowed
    pub max_process_time: Duration,
    // turning radius of the plane, in meters
    pub turning_radius: f32,
    // vertex within this threshold will be merged into one
    pub vertex_merge_threshold: f32,
    // whether generate virtual nodes for flyzones
    pub virtualize_flyzone: bool,
}

impl Default for TConfig {
    fn default() -> Self {
        Self::new(
            DEFAULT_BUFFER_SIZE,
            Duration::from_secs(DEFAULT_PROCESS_TIME),
            DEFAULT_TURNING_RADIUS,
            DEFAULT_V_MERGE_THRESHOLD,
            true,
        )
    }
}

impl TConfig {
    pub fn new(
        buffer_size: f32,
        max_process_time: Duration,
        turning_radius: f32,
        v_merge_threshold: f32,
        virtualize_flyzone: bool,
    ) -> Self {
        Self {
            buffer_size: buffer_size,
            max_process_time: max_process_time,
            turning_radius: turning_radius,
            vertex_merge_threshold: v_merge_threshold,
            virtualize_flyzone: virtualize_flyzone,
        }
    }
}
