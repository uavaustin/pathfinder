// A wrapper for transferring location data between Rust and Wasm/JS.
module.exports.LocationWrapper = class LocationWrapper {
  /**
   * @param {number} latitude  - In degrees.
   * @param {number} longitude - In degrees.
   * @param {number} altitude  - In meters.
   */
  constructor(latitude, longitude, altitude) {
    this.lat = latitude;
    this.lon = longitude;
    this.alt = altitude;
  }

  function get_lat() {
    return this.lat;
  }

  function get_lon() {
    return this.lon;
  }

  function get_alt() {
    return this.alt;
  }
};

// A wrapper for transferring obstacle data between Rust and Wasm/JS.
module.exports.ObstacleWrapper = class ObstacleWrapper {
  /**
   * @param {Object} location - Location as a LocationWrapper.
   * @param {number} radius   - In meters.
   * @param {number} height   - In meters.
   */
  constructor(location, radius, height) {
    this.location = location;
    this.radius = radius;
    this.height = height;
  }

  function get_obstacle_location() {
    return this.location;
  }

  function get_obstacle_radius() {
    return this.radius;
  }

  function get_height() {
    return this.height;
  }
};

// A wrapper for transferring plane data between Rust and Wasm/JS.
module.exports.PlaneWrapper = class PlaneWrapper {
  /**
   * @param {Object} location         - Location as a LocationWrapper.
   * @param {number} [yaw=-1]         - In degrees, -1 if not provided.
   * @param {number} [pitch=-1]       - In degrees, -1 if not provided.
   * @param {number} [roll=-1]        - In degrees, -1 if not provided.
   * @param {number} [airspeed=-1]    - In meters per second, -1 if not provided.
   * @param {number} [groundspeed=-1] - In meters per second, -1 if not provided.
   * @param {number} [windDir=-1]     - In degrees, -1 if not provided
   */
  constructor(location, yaw, pitch, roll, airspeed, groundspeed, windDir) {
    this.location = location;
    this.yaw = yaw ? yaw : -1;
    this.pitch = pitch ? pitch : -1;
    this.roll = roll ? roll : -1;
    this.airspeed = airspeed ? airspeed : -1;
    this.groundspeed = groundspeed ? groundspeed : -1;
    this.windDir = windDir ? windDir : -1;
  }

  function get_plane_location() {
    return this.location;
  }

  function get_yaw() {
    return this.yaw;
  }

  function get_pitch() {
    return this.pitch;
  }

  function get_roll() {
    return this.roll;
  }

  function get_airspeed() {
    return this.airspeed;
  }

  function get_groundspeed() {
    return this.groundspeed;
  }

  function get_wind_dir() {
    return this.windDir;
  }
};

// A wrapper for transferring TConfig data between Rust and Wasm/JS.
module.exports.TConfigWrapper = class TConfigWrapper {
  /**
   * @param {number}  bufferSize           - Buffer around obstacle in meters.
   * @param {number}  maxProcessTime       - Maximum process time allowed in seconds.
   * @param {number}  turningRadius        - Turning radius of plane in meters.
   * @param {number}  vertexMergeThreshold - In meters. Vertices within threshold will be merged into one.
   * @param {boolean} virtualizeFlyzone    - Whether to generate virtual nodes for flyzones.
   */
  constructor(bufferSize, maxProcessTime, turningRadius, vertexMergeThreshold, virtualizeFlyzone) {
    this.bufferSize = bufferSize;
    this.maxProcessTime = maxProcessTime;
    this.turningRadius = turningRadius;
    this.vertexMergeThreshold = vertexMergeThreshold;
    this.virtualizeFlyzone = virtualizeFlyzone;
  }

  function get_buffer_size() {
    return this.bufferSize;
  }

  function get_max_process_time() {
    return this.maxProcessTime;
  }

  function get_turning_radius() {
    return this.turningRadius;
  }

  function get_vertex_merge_threshold() {
    return this.vertexMergeThreshold;
  }

  function get_virtualize_flyzone() {
    return this.virtualizeFlyzone;
  }
};

// A wrapper for transferring waypoint data between Rust and Wasm/JS.
module.exports.WaypointWrapper = class WaypointWrapper {
  /**
   * @param {Object} location - Location as a LocationWrapper.
   * @param {number} radius   - In meters.
   */
  constructor(location, radius) {
    this.location = location;
    this.radius = radius;
  }

  function get_waypoint_location() {
    return this.location;
  }

  function get_waypoint_radius() {
    return this.radius;
  }
};