#ifndef PATH_FINDER_WAYPOINT_H
#define PATH_FINDER_WAYPOINT_H

#include "location.h"

struct Waypoint {
    loc::Location loc;
    double radius;
};

#endif
