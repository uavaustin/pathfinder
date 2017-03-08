#include <string>

#include <nan.h>

#include "location.h"
#include "obstacle.h"
#include "path_finder.h"
#include "waypoint.h"

v8::Local<v8::String> V8String(std::string string) {
    return Nan::New(string).ToLocalChecked();
}

std::string ValueToString(v8::Local<v8::Value> value) {
    return std::string(*v8::String::Utf8Value(value->ToString()));
}

double DoubleInObject(v8::Local<v8::Object> object, std::string property) {
    return object->Get(V8String(property))->NumberValue();
}

std::string StringInObject(v8::Local<v8::Object> object,
        std::string property) {
    return std::string(*v8::String::Utf8Value(object->Get(
            V8String(property))));
}

std::vector<Waypoint> UnpackWaypoints(v8::Local<v8::Array> waypoints_array) {
    std::vector<Waypoint> waypoint_vector;

    int num_waypoints = DoubleInObject(waypoints_array, "length");

    for (int i = 0; i < num_waypoints; i++) {
        v8::Local<v8::Object> waypoint =
                v8::Local<v8::Object>::Cast(waypoints_array->Get(i));

        double lat = DoubleInObject(waypoint, "lat");
        double lon = DoubleInObject(waypoint, "lon");
        double alt = DoubleInObject(waypoint, "alt");
        double radius = DoubleInObject(waypoint, "radius");

        Waypoint wp;

        wp.loc = loc::Location(lat, lon, alt);
        wp.radius = radius;

        waypoint_vector.push_back(wp);
    }

    return waypoint_vector;
}

std::vector<StationaryObstacle> UnpackStationaryObstacles(
        v8::Local<v8::Array> stat_obs_array) {
    std::vector<StationaryObstacle> stat_obs_vector;

    int num_stat_obs = DoubleInObject(stat_obs_array, "length");

    for (int i = 0; i < num_stat_obs; i++) {
        v8::Local<v8::Object> stat_obs =
                v8::Local<v8::Object>::Cast(stat_obs_array->Get(i));

        double lat = DoubleInObject(stat_obs, "lat");
        double lon = DoubleInObject(stat_obs, "lon");
        double height = DoubleInObject(stat_obs, "height");
        double radius = DoubleInObject(stat_obs, "radius");
        double avoid_radius = DoubleInObject(stat_obs, "avoid_radius");

        StationaryObstacle obs;

        obs.loc = loc::Location(lat, lon);
        obs.height = height;
        obs.radius = radius;
        obs.avoid_radius = avoid_radius;

        stat_obs_vector.push_back(obs);
    }

    return stat_obs_vector;
}

std::vector<MovingObstacle> UnpackMovingObstacles(
        v8::Local<v8::Array> mov_obs_array) {
    std::vector<MovingObstacle> mov_obs_vector;

    int num_mov_obs = DoubleInObject(mov_obs_array, "length");

    for (int i = 0; i < num_mov_obs; i++) {
        v8::Local<v8::Object> mov_obs =
                v8::Local<v8::Object>::Cast(mov_obs_array->Get(i));

        double lat = DoubleInObject(mov_obs, "lat");
        double lon = DoubleInObject(mov_obs, "lon");
        double alt = DoubleInObject(mov_obs, "alt");
        double speed = DoubleInObject(mov_obs, "speed");
        double direction = DoubleInObject(mov_obs, "direction");
        double radius = DoubleInObject(mov_obs, "radius");
        double avoid_radius = DoubleInObject(mov_obs, "avoid_radius");

        MovingObstacle obs;

        obs.loc = loc::Location(lat, lon, alt);
        obs.speed = speed;
        obs.direction = direction;
        obs.radius = radius;
        obs.avoid_radius = avoid_radius;

        mov_obs_vector.push_back(obs);
    }

    return mov_obs_vector;
}

Telemetry UnpackTelemetry(v8::Local<v8::Object> telemetry_object) {
    Telemetry telem;

    double telem_lat = DoubleInObject(telemetry_object, "lat");
    double telem_lon = DoubleInObject(telemetry_object, "lon");
    double telem_alt = DoubleInObject(telemetry_object, "alt");
    double telem_yaw = DoubleInObject(telemetry_object, "yaw");
    double telem_pitch = DoubleInObject(telemetry_object, "pitch");
    double telem_roll = DoubleInObject(telemetry_object, "roll");
    double telem_airspeed = DoubleInObject(telemetry_object, "airspeed");

    telem.loc = loc::Location(telem_lat, telem_lon, telem_alt);
    telem.yaw = telem_yaw;
    telem.pitch = telem_pitch;
    telem.roll = telem_roll;
    telem.airspeed = telem_airspeed;

    return telem;
}

AdjustPathOptions UnpackAdjustPathOptions(
        v8::Local<v8::Object> options_object) {
    AdjustPathOptions options;

    double mov_obs_time_limit = DoubleInObject(options_object,
            "mov_obs_time_limit");

    options.mov_obs_time_limit = mov_obs_time_limit;

    return options;
}

v8::Local<v8::Array> PackWaypoints(std::vector<Waypoint> waypoint_vector) {
    v8::Local<v8::Array> waypoints_array = Nan::New<v8::Array>();

    waypoints_array->Set(0, Nan::New<v8::Object>());

    for (unsigned i = 0; i < waypoint_vector.size(); i++) {
        v8::Local<v8::Object> wp = Nan::New<v8::Object>();

        wp->Set(V8String("lat"), Nan::New(waypoint_vector[i].loc.lat));
        wp->Set(V8String("lon"), Nan::New(waypoint_vector[i].loc.lon));
        wp->Set(V8String("alt"), Nan::New(waypoint_vector[i].loc.alt));
        wp->Set(V8String("radius"), Nan::New(waypoint_vector[i].radius));

        waypoints_array->Set(i, wp);
    }

    return waypoints_array;
}
