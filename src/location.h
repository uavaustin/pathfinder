#ifndef PATH_FINDER_LOCATION_H
#define PATH_FINDER_LOCATION_H

#include <cfloat>

namespace loc {
    struct Distance {
        Distance() : x(0), y(0), z(0) {}

        Distance(double x_, double y_) : x(x_), y(y_), z(0) {}

        Distance(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

        double x;
        double y;
        double z;
    };

    struct Location {
        Location() : lat(0), lon(0), alt(-DBL_MAX) {}

        Location(double lat_, double lon_) :
                lat(lat_), lon(lon_), alt(-DBL_MAX) {}

        Location(double lat_, double lon_, double alt_) :
                lat(lat_), lon(lon_), alt(alt_) {}

        double lat;
        double lon;
        double alt;
    };
}

#endif
