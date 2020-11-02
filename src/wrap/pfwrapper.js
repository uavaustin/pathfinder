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

  get lat() {
    return this.lat;
  }

  get lon() {
    return this.lon;
  }

  get alt() {
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

  get location() {
    return this.location;
  }

  get radius() {
    return this.radius;
  }

  get height() {
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

  get location() {
    return this.location;
  }

  get yaw() {
    return this.yaw;
  }

  get pitch() {
    return this.pitch;
  }

  get roll() {
    return this.roll;
  }

  get airspeed() {
    return this.airspeed;
  }

  get groundspeed() {
    return this.groundspeed;
  }

  get windDir() {
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

  get bufferSize() {
    return this.bufferSize;
  }

  get maxProcessTime() {
    return this.maxProcessTime;
  }

  get turningRadius() {
    return this.turningRadius;
  }

  get vertexMergeThreshold() {
    return this.vertexMergeThreshold;
  }

  get virtualizeFlyzone() {
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

  get waypointLocation() {
    return this.location;
  }

  get waypointRadius() {
    return this.radius;
  }
};