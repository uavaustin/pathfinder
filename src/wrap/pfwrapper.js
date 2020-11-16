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

  getLat() {
    return this.lat;
  }

  getLon() {
    return this.lon;
  }

  getAlt() {
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

  getLocation() {
    return this.location;
  }

  getRadius() {
    return this.radius;
  }

  getHeight() {
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
  constructor(location, yaw = -1, pitch = -1, roll = -1, airspeed = -1, groundspeed = -1, windDir = -1) {
    this.location = location;
    this.yaw = yaw;
    this.pitch = pitch;
    this.roll = roll;
    this.airspeed = airspeed;
    this.groundspeed = groundspeed;
    this.windDir = windDir;
  }

  getLocation() {
    return this.location;
  }

  getYaw() {
    return this.yaw;
  }

  getPitch() {
    return this.pitch;
  }

  getRoll() {
    return this.roll;
  }

  getAirspeed() {
    return this.airspeed;
  }

  getGroundspeed() {
    return this.groundspeed;
  }

  getWindDir() {
    return this.windDir;
  }
};

// A wrapper for transferring TConfig data between Rust and Wasm/JS.
// TODO: Centralize defaults (i.e. handle from original Rust class)
module.exports.TConfigWrapper = class TConfigWrapper {
  /**
   * @param {number}  [bufferSize=2]              - Buffer around obstacle in meters.
   * @param {number}  [maxProcessTime=10]         - Maximum process time allowed in seconds.
   * @param {number}  [turningRadius=5]           - Turning radius of plane in meters.
   * @param {number}  [vertexMergeThreshold=5]    - In meters. Vertices within threshold will be merged into one.
   * @param {boolean} [virtualizeFlyzone=true]    - Whether to generate virtual nodes for flyzones.
   */
  constructor(bufferSize = 2, maxProcessTime = 10, turningRadius = 5, vertexMergeThreshold = 5, virtualizeFlyzone = true) {
    this.bufferSize = bufferSize;
    this.maxProcessTime = maxProcessTime;
    this.turningRadius = turningRadius;
    this.vertexMergeThreshold = vertexMergeThreshold;
    this.virtualizeFlyzone = virtualizeFlyzone;
  }

  getBufferSize() {
    return this.bufferSize;
  }

  getMaxProcessTime() {
    return this.maxProcessTime;
  }

  getTurningRadius() {
    return this.turningRadius;
  }

  getVertexMergeThreshold() {
    return this.vertexMergeThreshold;
  }

  getVirtualizeFlyzone() {
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

  getLocation() {
    return this.location;
  }

  getRadius() {
    return this.radius;
  }
};