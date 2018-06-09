# Pathfinder

pathfinder is a rust crate that adjusts a list of waypoints for a plane to avoid obstacles.  Given the plane position and the waypoints, the pathfinder will determine if there are any obstacles between waypoints and if so, attempt to add additional waypoints between them to avoid the obstacles.  Pathfinder currently runs on A* algorithm.

## Use

Creating a new Pathfinder

```let mut pathfinder = Pathfinder::new();```

If not initialized, Pathfinder will not do anything.  Initialization requires a grid size which determines the speed and accuracy of the path and a list of flight zones.for initialization.  Larger grid size adjusts the path faster at the cost of being less accurate.  A valid flight zone is a list of at least three Points in order representing the zone.  Obstacles can be provided at initialization or added later.

Initialization with obstacles

```rust
let flyzone = vec!(vec!(
      Point::from_degrees(30.32469, -97.60466, 0f32),
      Point::from_degrees(30.32437, -97.60367, 0f32),
      Point::from_degrees(30.32356, -97.60333, 0f32)
  ));
  let obstacles = vec!(
      Obstacle::from_degrees(30.32228, -97.60198, 50f32, 10f32)
  );

  let mut pathfinder = Pathfinder::new();
  pathfinder.init(5.0, flyzone, obstacles);
```

Initialization without obstacles
```pathfinder.init(5.0, flyzone, Vec::new());```

The pathfinder expects the list of waypoints to be represented by a LinkedList from the rust standard library and returns the adjusted path also as a LinkedList.

Getting the adjusted path
```rust
let mut waypoints = LinkedList::new();
waypoints.push_back(
    Waypoint::from_degrees(0, 30.322280883789063, -97.60298156738281, 100f32, 10f32)
);
waypoints.push_back(
    Waypoint::from_degrees(1, 30.322280883789063, -97.60098266601564, 150f32, 10f32)
);

let result = pathfinder.get_adjust_path(
    Plane::from_degrees(30.32298, -97.60310, 100.0),
    waypoints);
```      
