#ifndef PATH_FINDER_PATH_FINDER_H
#define PATH_FINDER_PATH_FINDER_H

#include <vector>

#include "location.h"
#include "obstacle.h"
#include "waypoint.h"

struct Telemetry {
    loc::Location loc;
    double yaw;
    double pitch;
    double roll;
    double airspeed;
};

struct AdjustPathOptions {
    double mov_obs_time_limit;
};

bool AdjustPath(const std::vector<Waypoint> &waypoints, Telemetry telem,
        std::vector<StationaryObstacle> stat_obs,
        std::vector<MovingObstacle> mov_obs, AdjustPathOptions options);

#endif
