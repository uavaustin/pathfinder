#include <string>

#include <nan.h>

#include "location.h"
#include "obstacle.h"
#include "path_finder.h"
#include "waypoint.h"
#include "wrapper_util.h"

#include <unistd.h>

/*
 * Wrapper class for Asynchronous AdjustPath calls
 */
class PathFinderWorker : public Nan::AsyncWorker {
public:
    /*
     * Constructor; essentially stores path finder variables in private members
     */
    PathFinderWorker(const std::vector<Waypoint> waypoints, Telemetry telem, 
            std::vector<StationaryObstacle> stat_obs,
            std::vector<MovingObstacle> mov_obs, AdjustPathOptions options,
            Nan::Callback *callback) :
            Nan::AsyncWorker(callback) {
        this->waypoints  = waypoints;
        this->telem      = telem;
        this->stat_obs   = stat_obs;
        this->mov_obs    = mov_obs;
        this->options    = options;
        this->did_change = false;
    }

    /* 
     * Worker Thread stuff goes here (i.e. the Async stuff)
     * No Nan/V8 voodoo allowed
     */
    void Execute() {
        this->did_change = AdjustPath(waypoints, telem, stat_obs, mov_obs, 
                options);
    }

    /*
     * Called internally upon completion of execute
     * Nan/V8 voodoo allowed
     */
    void HandleOKCallback() {
        
        v8::Local<v8::Value> new_waypoints;

        if (!this->did_change) {
            new_waypoints = Nan::Null();
        } else {
            v8::Local<v8::Array> new_waypoints_array = 
                    PackWaypoints(waypoints);

            new_waypoints = v8::Local<v8::Value>::Cast(new_waypoints_array);
        }

        const unsigned argc = 2;

        v8::Local<v8::Value> argv[argc] = {Nan::Null(), new_waypoints};

        callback->Call(argc, argv);
    }

    /*
     * Called internally to handle errors in Execute()
     */
    void HandleErrorCallback() {

        // Get the error
        v8::Local<v8::Value> error = 
                v8::Exception::Error(V8String(ErrorMessage()));

        const unsigned argc = 2;

        v8::Local<v8::Value> argv[argc] = {error, Nan::Null()};

        callback->Call(argc, argv);
    }

private:
    std::vector<Waypoint> waypoints;
    Telemetry telem; 
    std::vector<StationaryObstacle> stat_obs;
    std::vector<MovingObstacle> mov_obs;
    AdjustPathOptions options;
    bool did_change;
};


/*
 * Wraps the C++ AdjustPath function defined for use in Node.js.
 * This function will run synchronously.
 */
void AdjustPathWrapper(const Nan::FunctionCallbackInfo<v8::Value>& info) {

    std::string error_message = "";
    std::vector<Waypoint> waypoints;

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

    Nan::Callback *cb = new Nan::Callback(info[5].As<v8::Function>());
    // v8::Local<v8::Function> cb = info[5].As<v8::Function>();

    AsyncQueueWorker(new PathFinderWorker(waypoints, telem, stat_obs_vector,
                mov_obs_vector, options, cb));
}

/*
 * Add AdjustPathWrapper to the module exports.
 */
void Init(v8::Local<v8::Object> exports) {
    exports->Set(V8String((char*)"adjustPath"),
            Nan::New<v8::FunctionTemplate>(AdjustPathWrapper)->GetFunction());
}

NODE_MODULE(path_finder, Init);
