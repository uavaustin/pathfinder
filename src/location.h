#ifndef PATH_FINDER_LOCATION_H
#define PATH_FINDER_LOCATION_H

#include <cfloat>

const long double PI = 3.141592653589793238L;

namespace loc {
    class Distance {
    public:
        double x;
        double y;

        Distance(double x, double y);

        double GetMagnitude();
        double GetBearing();

        Distance GetTransform(double angle);

        static Distance FromMagnitude(double dist, double bearing);
    };

    Distance operator+(const Distance &dist, Distance other);
    Distance operator-(const Distance &dist, Distance other);

    Distance operator*(const Distance &dist, double k);
    Distance operator*(double k, const Distance &dist);
    Distance operator/(const Distance &dist, double k);

    class Distance3D: public Distance {
    public:
        double z;

        Distance3D(double x, double y, double z);

        double GetMagnitude();

        Distance3D GetTransform(double angle);

        static Distance3D FromMagnitude(double dist, double bearing,
                double alt);
    };

    Distance3D operator+(Distance3D &dist, Distance3D other);
    Distance3D operator+(Distance3D &dist, Distance other);
    Distance3D operator+(Distance other, const Distance3D &dist);
    Distance3D operator-(Distance3D &dist, Distance3D other);
    Distance3D operator-(Distance3D &dist, Distance other);
    Distance3D operator-(Distance other, const Distance3D &dist);

    Distance3D operator*(Distance3D &dist, double k);
    Distance3D operator*(double k, const Distance3D &dist);
    Distance3D operator/(Distance3D &dist, double k);

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
