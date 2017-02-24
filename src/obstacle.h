#ifndef PATH_FINDER_OBSTACLE_H
#define PATH_FINDER_OBSTACLE_H

#include "location.h"

struct StationaryObstacle {
    loc::Location loc;
    double height;
    double radius;
    double avoid_radius;
};

struct MovingObstacle {
    loc::Location loc;
    double speed;
    double direction;
    double radius;
    double avoid_radius;
};

double getCrossSectionalRadius(StationaryObstacle obs, double alt);
double getCrossSectionalRadius(MovingObstacle obs, double alt);

double getAvoidSectionalRadius(StationaryObstacle obs, double alt);
double getAvoidSectionalRadius(MovingObstacle obs, double alt);

#endif
