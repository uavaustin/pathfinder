// Config struct for gridstar
use std::time::Duration;

pub const DEFAULT_BUFFER_SIZE: f32 = 5f32;
pub const DEFAULT_PROCESS_TIME: u64 = 10u64;
pub const DEFAULT_GRID_SIZE: f32 = 1f32;
pub const DEFAULT_DIRECT_PATH_MODIFIER_WEIGHT: f64 = 0.001;
pub const DEFAULT_HEADING_MODIFIER_WEIGHT: f64 = 0.15;

#[derive(Clone)]
pub struct GConfig {
    // buffer around obstacles, in meters
    pub buffer_size: f32,
    // maximum procssed time allowed
    pub max_process_time: Duration,
    // size of grid cells, in meters
    pub grid_size: f32,
    // weight that determine how much a straight path is desired
    pub default_direct_weight: f32,
    // weight that determine how much maintaining heighting is desired
    pub default_heading_weight: f32,
}

impl Default for GConfig {
    fn default() -> Self {
        Self::new(
            DEFAULT_BUFFER_SIZE,
            Duration::from_secs(DEFAULT_PROCESS_TIME),
            DEFAULT_GRID_SIZE,
            DEFAULT_DIRECT_PATH_MODIFIER_WEIGHT,
            DEFAULT_HEADING_MODIFIER_WEIGHT,
        )
    }
}

impl GConfig {
    pub fn new(
        buffer_size: f32,
        max_process_time: Duration,
        grid_size: f32,
        default_direct_weight: f32,
        default_heading_weight: f32,
    ) -> Self {
        Self {
            buffer_size: buffer_size,
            max_process_time: max_process_time,
            grid_size: grid_size,
            default_direct_weight: default_direct_weight,
            default_heading_weight: default_heading_weight,
        }
    }
}
