'use strict';

const pathFinder = require('./build/Release/path-finder');

exports.adjustPath = function adjustPath(waypoints, telemetry,
        stationaryObstacles, movingObstacles, options) {
    return new Promise((resolve, reject) => {
        pathFinder.adjustPath(waypoints, telemetry, stationaryObstacles,
                movingObstacles, options, (err, newWaypoints) => {
            if (err) reject(err);
            else resolve(newWaypoints);
        });
    });
};
