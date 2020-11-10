// A wrapper for transferring location data between Rust and Wasm/JS.
module.exports.LocationWrapper = class LocationWrapper {
  /**
   * @param {number} latitude  - In degrees.
   * @param {number} longitude - In degrees.
   * @param {number} altitude  - In meters.
   */
  constructor(latitude, longitude, altitude) {
    this._lat = latitude;
    this._lon = longitude;
    this._alt = altitude;
  }

  get lat() {
    return this._lat;
  }

  set lat(value) {
    this._lat = value;
  }

  get lon() {
    return this._lon;
  }

  set lon(value) {
    this._lon = value;
  }

  get alt() {
    return this._alt;
  }

  set alt(value) {
    this._alt = value;
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
    this._location = location;
    this._radius = radius;
    this._height = height;
  }

  get location() {
    return this._location;
  }

  set location(value) {
    this._location = value;
  }

  get radius() {
    return this._radius;
  }

  set radius(value) {
    this._radius = value;
  }

  get height() {
    return this._height;
  }

  set height(value) {
    this._height = value;
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
    this._location = location;
    this._yaw = yaw;
    this._pitch = pitch;
    this._roll = roll;
    this._airspeed = airspeed;
    this._groundspeed = groundspeed;
    this._windDir = windDir;
  }

  get location() {
    return this._location;
  }

  set location(value) {
    this._location = value;
  }

  get yaw() {
    return this._yaw;
  }

  set yaw(value) {
    this._yaw = value;
  }

  get pitch() {
    return this._pitch;
  }

  set pitch(value) {
    this._pitch = value;
  }

  get roll() {
    return this._roll;
  }

  set roll(value) {
    this._roll = value;
  }

  get airspeed() {
    return this._airspeed;
  }

  set airspeed(value) {
    this._airspeed = value;
  }

  get groundspeed() {
    return this._groundspeed;
  }

  set groundspeed(value) {
    this._groundspeed = value;
  }

  get windDir() {
    return this._windDir;
  }

  set windDir(value) {
    this._windDir = value;
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
    this._bufferSize = bufferSize;
    this._maxProcessTime = maxProcessTime;
    this._turningRadius = turningRadius;
    this._vertexMergeThreshold = vertexMergeThreshold;
    this._virtualizeFlyzone = virtualizeFlyzone;
  }

  get bufferSize() {
    return this._bufferSize;
  }

  set bufferSize(value) {
    this._bufferSize = value;
  }

  get maxProcessTime() {
    return this._maxProcessTime;
  }

  set maxProcessTime(value) {
    this._maxProcessTime = value;
  }

  get turningRadius() {
    return this._turningRadius;
  }

  set turningRadius(value) {
    this._turningRadius = value;
  }

  get vertexMergeThreshold() {
    return this._vertexMergeThreshold;
  }

  set vertexMergeThreshold(value) {
    this._vertexMergeThreshold = value;
  }

  get virtualizeFlyzone() {
    return this._virtualizeFlyzone;
  }

  set virtualizeFlyzone(value) {
    this._virtualizeFlyzone = value;
  }
};

// A wrapper for transferring waypoint data between Rust and Wasm/JS.
module.exports.WaypointWrapper = class WaypointWrapper {
  /**
   * @param {Object} location - Location as a LocationWrapper.
   * @param {number} radius   - In meters.
   */
  constructor(location, radius) {
    this._location = location;
    this._radius = radius;
  }

  get location() {
    return this._location;
  }

  set location(value) {
    this._location = value;
  }

  get radius() {
    return this._radius;
  }

  set radius(value) {
    this._radius = value;
  }
};