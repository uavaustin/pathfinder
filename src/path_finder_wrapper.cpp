#include <string>

#include <nan.h>

#include "location.h"
#include "obstacle.h"
#include "path_finder.h"
#include "waypoint.h"
#include "wrapper_util.h"

/*
 * Wraps the C++ AdjustPath function defined for use in Node.js.
 */
void AdjustPathWrapper(const Nan::FunctionCallbackInfo<v8::Value>& info) {
    std::string error_message = "";
    std::vector<Waypoint> waypoints;

    bool did_change = false;

    try {
        // Load the arguments pass into the function
        v8::Local<v8::Array> waypoints_array = info[0].As<v8::Array>();
        v8::Local<v8::Object> telemetry_object = info[1].As<v8::Object>();
        v8::Local<v8::Array> stat_obs_array = info[2].As<v8::Array>();
        v8::Local<v8::Array> mov_obs_array = info[3].As<v8::Array>();
        v8::Local<v8::Object> options_object = info[4].As<v8::Object>();

        waypoints = UnpackWaypoints(waypoints_array);
        Telemetry telem = UnpackTelemetry(telemetry_object);
        std::vector<StationaryObstacle> stat_obs_vector =
                UnpackStationaryObstacles(stat_obs_array);
        std::vector<MovingObstacle> mov_obs_vector =
                UnpackMovingObstacles(mov_obs_array);
        AdjustPathOptions options = UnpackAdjustPathOptions(options_object);

        did_change = AdjustPath(waypoints, telem, stat_obs_vector,
                mov_obs_vector, options);

    } catch (const std::exception& e) {
        error_message += e.what();
    } catch (...) {
        error_message += "Unknown C++ Exception";
    }

    v8::Local<v8::Function> cb = info[5].As<v8::Function>();

    v8::Local<v8::Value> error;

    if (error_message == "") {
        error = Nan::Null();
    } else {
        error = v8::Exception::Error(V8String(error_message));
    }

    v8::Local<v8::Value> new_waypoints;

    if (!did_change || error_message != "") {
        new_waypoints = Nan::Null();
    } else {
        v8::Local<v8::Array> new_waypoints_array = Nan::New<v8::Array>();

        new_waypoints_array->Set(0, Nan::New<v8::Object>());

        for (unsigned i = 0; i < waypoints.size(); i++) {
            v8::Local<v8::Object> wp = Nan::New<v8::Object>();

            wp->Set(V8String("lat"), Nan::New(waypoints[i].loc.lat));
            wp->Set(V8String("lon"), Nan::New(waypoints[i].loc.lon));
            wp->Set(V8String("alt"), Nan::New(waypoints[i].loc.alt));
            wp->Set(V8String("radius"), Nan::New(waypoints[i].radius));

            new_waypoints_array->Set(i, wp);
        }

        new_waypoints = v8::Local<v8::Value>::Cast(new_waypoints_array);
    }

    const unsigned argc = 2;

    v8::Local<v8::Value> argv[argc] = {error, new_waypoints};

    Nan::MakeCallback(Nan::GetCurrentContext()->Global(), cb, argc, argv);
}

/*
 * Add AdjustPathWrapper to the module exports.
 */
void Init(v8::Local<v8::Object> exports) {
    exports->Set(V8String("adjustPath"),
            Nan::New<v8::FunctionTemplate>(AdjustPathWrapper)->GetFunction());
}

NODE_MODULE(path_finder, Init);
