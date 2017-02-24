const assert = require('chai').assert;

const pathFinder = require('../..');

describe('#adjustPath()', function () {
    let waypoints;
    let telemetry;
    let stationaryObstacles;
    let movingObstacles;
    let options;

    beforeEach(function () {
        waypoints = [
            {
                lat: 1,
                lon: 1.5,
                alt: 100,
                radius: 40
            },
            {
                lat: 2.5,
                lon: 2,
                alt: 150,
                radius: 20.1
            }
        ];

        telemetry = {
            lat: 3,
            lon: 4,
            alt: 5,
            yaw: 6,
            pitch: 7,
            roll: 8,
            airspeed: 9
        };

        stationaryObstacles = [
            {
                lat: 10,
                lon: 11.0,
                height: 110,
                radius: 50,
                avoid_radius: 30
            },
            {
                lat: 20,
                lon: 21.0,
                height: 210,
                radius: 52,
                avoid_radius: 30
            },
            {
                lat: 30,
                lon: 21.0,
                height: 310,
                radius: 53,
                avoid_radius: 30
            }
        ];

        movingObstacles = [
            {
                lat: 11,
                lon: 11.1,
                alt: 111,
                speed: 1.11,
                direction: 1,
                radius: 11,
                avoid_radius: 40
            },
            {
                lat: 22,
                lon: 22.1,
                alt: 222,
                speed: 2.11,
                direction: 2,
                radius: 22,
                avoid_radius: 40
            }
        ];

        options = {
            mov_obs_time_limit: 30
        };
    });

    it('should return null if no obstacles are given', function (done) {
        pathFinder.adjustPath(waypoints, telemetry, {}, {}, options).then(
                (newWaypoints) => {
            assert.isNull(newWaypoints);

            done();
        }).catch(done);
    });
});
