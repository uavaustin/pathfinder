#include <vector>

#include "location.h"
#include "path_finder.h"
#include "obstacle.h"
#include "waypoint.h"

bool AdjustPath(const std::vector<Waypoint> &waypoints, Telemetry telem,
        std::vector<StationaryObstacle> stat_obs,
        std::vector<MovingObstacle> mov_obs, AdjustPathOptions options) {

    if (stat_obs.size() == 0 && mov_obs.size() == 0) {
        return false;
    }

    return false;
}
