#include <string>

#include <nan.h>

#include "obstacle.h"
#include "path_finder.h"
#include "waypoint.h"

/**
 * Return a string passable into v8 functions.
 */
v8::Local<v8::String> V8String(std::string string);

v8::Local<v8::String> V8String(char *string);

/**
 * Return a string from a v8 value.
 */
std::string ValueToString(v8::Local<v8::Value> value);

/**
 * Return a double from a property inside a v8 object.
 */
double DoubleInObject(v8::Local<v8::Object> object, std::string property);

/**
 * Return a string from a property inside a v8 object.
 */
std::string StringInObject(v8::Local<v8::Object> object,
        std::string property);

std::vector<Waypoint> UnpackWaypoints(v8::Local<v8::Array> waypoints_array);

std::vector<StationaryObstacle> UnpackStationaryObstacles(
        v8::Local<v8::Array> stat_obs_array);

std::vector<MovingObstacle> UnpackMovingObstacles(
        v8::Local<v8::Array> mov_obs_array);

Telemetry UnpackTelemetry(v8::Local<v8::Object> telemetry_object);

AdjustPathOptions UnpackAdjustPathOptions(
        v8::Local<v8::Object> options_object);

v8::Local<v8::Array> PackWaypoints(std::vector<Waypoint> waypoint_vector);
