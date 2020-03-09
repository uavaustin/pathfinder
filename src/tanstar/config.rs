// Config struct for tanstar
use std::time::Duration;

pub const DEFAULT_BUFFER_SIZE: f32 = 2f32;
pub const DEFAULT_PROCESS_TIME: u64 = 10u64;
pub const DEFAULT_TURNING_RADIUS: f32 = 5f32;
pub const DEFAULT_V_MERGE_THRESHOLD: f32 = 5f32;

#[derive(Clone)]
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
        vertex_merge_threshold: f32,
        virtualize_flyzone: bool,
    ) -> Self {
        Self {
            buffer_size,
            max_process_time,
            turning_radius,
            vertex_merge_threshold,
            virtualize_flyzone,
        }
    }
}

impl PartialEq for TConfig {
    fn eq(&self, other: &Self) -> bool {
        self.max_process_time.eq(&other.max_process_time)
            && (
                self.buffer_size,
                self.turning_radius,
                self.vertex_merge_threshold,
                self.virtualize_flyzone,
            ) == (
                other.buffer_size,
                other.turning_radius,
                other.vertex_merge_threshold,
                other.virtualize_flyzone,
            )
    }

    fn ne(&self, other: &Self) -> bool {
        self.max_process_time.ne(&other.max_process_time)
            || (
                self.buffer_size,
                self.turning_radius,
                self.vertex_merge_threshold,
                self.virtualize_flyzone,
            ) != (
                other.buffer_size,
                other.turning_radius,
                other.vertex_merge_threshold,
                other.virtualize_flyzone,
            )
    }
}

impl Eq for TConfig {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eq() {
        let config1 = TConfig::new(10f32, Duration::from_secs(1), 20f32, 30f32, false);
        let config2 = TConfig::from(config1.clone());
        let config3 = TConfig::new(10f32, Duration::from_secs(1), 20f32, 30f32, false);
        assert!(config1.eq(&config2) && config2.eq(&config1));
        assert!(config2.eq(&config3) && config3.eq(&config2));
        assert!(config1.eq(&config3) && config3.eq(&config1));
    }

    #[test]
    fn test_ne() {
        let config1 = TConfig::new(10f32, Duration::from_secs(1), 20f32, 30f32, false);
        let config2 = TConfig::new(30f32, Duration::from_secs(1), 20f32, 10f32, false);
        assert!(config1.ne(&config2) && config2.ne(&config1));
    }
}
