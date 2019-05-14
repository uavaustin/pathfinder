// Config struct for tanstar
use std::time::Duration;
use tanstar::MIN_BUFFER;

pub struct TConfig {
    pub buffer_size: f32, // In meters
    pub max_process_time: Duration,
}

impl Default for TConfig {
    fn default() -> Self {
        Self::new(MIN_BUFFER, Duration::from_secs(10u64))
    }
}

impl TConfig {
    pub fn new(buffer_size: f32, max_process_time: Duration) -> Self {
        Self {
            buffer_size: buffer_size,
            max_process_time: max_process_time,
        }
    }
}
