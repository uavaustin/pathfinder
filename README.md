# Pathfinder

[![Build Status](https://travis-ci.org/uavaustin/pathfinder.svg?branch=master)](https://travis-ci.org/uavaustin/pathfinder)

Pathfinder is a rust crate that adjusts a list of waypoints for a plane to avoid obstacles.  Given the plane position and the waypoints, the pathfinder will determine if there are any obstacles between waypoints and if so, attempt to add additional waypoints between them to avoid the obstacles.  Pathfinder currently implements the Tan\* algorithm, which maps the field into a graph upon which the A\* algorithm can be ran.  Pathfinder is designed to be modular so algorithms can be swapped out with ease.

## Use

Creating a new Pathfinder requires the algorithm for path finding, configuration struct for the algorithm, flyzones, and list of obstacles.  A valid flight zone is a list of at least three Points in order representing the zone.  The pathfinder expects the list of waypoints to be represented by a LinkedList from the rust standard library and returns the adjusted path also as a LinkedList.

```rust
let flyzone = vec!(vec!(
      Point::from_degrees(30.32469, -97.60466, 0f32),
      Point::from_degrees(30.32437, -97.60367, 0f32),
      Point::from_degrees(30.32356, -97.60333, 0f32)
  ));
  let obstacles = vec!(
      Obstacle::from_degrees(30.32228, -97.60198, 50f32, 10f32)
  );

let pathfinder = Pathfinder::new(Tanstar::new(), TConfig::default(), flyzone, obstacles);
```

### Tan\*

Tan\* is the current algorithm used.  It takes advantage of all obstacles being circular so that only tangent lines are used to traverse the map.  Further, the algorithm natively accounts for paths to balance minimal turning and path length.  For efficiency, vertices that's within a threshold are merged together, which can be adjusted.

### Grid\*

Grid\* is the depercated algorithm where the map is subdivided into a grid and a path is subsequently generated.  Initialization requires a grid size which determines the speed and accuracy of the path and a list of flight zones. Larger grid size adjusts the path faster at the cost of being less accurate.  

## Getting the adjusted path

Waypoint is a generic type such that additional data can be wrapped.  

```rust
let mut waypoints = LinkedList::new();
waypoints.push_back(
    Waypoint::from_degrees(30.322280883789063, -97.60298156738281, 100f32, 10f32)
);
waypoints.push_back(
    Waypoint::from_degrees(30.322280883789063, -97.60098266601564, 150f32, 10f32)
);

let result = pathfinder.get_adjust_path(
    Plane::from_degrees(30.32298, -97.60310, 100.0),
    waypoints);
```

## Configuring Tan\*

The options to configure tan* are passed in a config struct.  Config can be passed at Initialization or later.

### Parameters

-   `buffer_size` - size of buffer area around obstacles.  Measured in meter (e.g obstacle with radius 3 meter and buffer of 1 meter would have an effective radius of 4 meter)
-   `max_process_time` - the longest the algorithm would process before returning **CURRENTLY NOT IMPLMENTED**
-   `turning_radius` - turning radius of the plane, used as radius for waypoint representing plane and the radius of virtual nodes
-   `vertex_merge_threshold` - as stated above, vertices below this threshold are merged into one to reduce the computatation load

## Configuring Grid\*

The weights used to calculate path preferences can be configured. Pathfinder will first look for environment variables.  If not found, it will search for a config file instead.  You can create a TOML file called `pathfinder.toml` in the project directory root and Pathfinder will use the weights in the configuration to calculate paths. **Parameters in toml file MUST be a float (i.e have a decimal point) or it will be ignored.**

### Parameters

-   `direct_path_modifier_weight` - high value makes Pathfinder prefer direct paths
-   `heading_modifier_weight` - high value makes Pathfinder prefer paths that maintains current heading

## Interacting with Node.js

The `wasm-bindgen` library gives the ability to convert Rust into Web Assembly into Javascript relatively easily.

### Building

For building with `wasm-bindgen` there is a tool called `wasm-pack` which will handle a lot of the required setup and
execution. To build for Node.js the following command should be used:

    $ wasm-pack build --target=nodejs

This will create a `pkg` directory at the crate root with JS and Wasm files.

Note: Use the option `--debug` when Wasm debug information (such as readable errors and stack traces) are desired.