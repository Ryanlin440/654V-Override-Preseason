// Distance-sensor wall reset (littleX). See src/littleX_handoff.md for the math and conventions.
//
// Call distanceReset() whenever the robot is sitting still near walls. It uses the current odom
// pose's heading and position to work out which wall each distance sensor is looking at, then solves
// for the robot's x or y from that wall's reading, no matter how far it is from the odom pose.
// Only the axes that got a good reading are changed; heading is never touched.
//
// Conventions (same as LemLib and the Desmos graph):
//   field is a square centered at (0, 0), walls at x = +-FIELD_HALF and y = +-FIELD_HALF
//   heading 0 = facing +Y, positive = clockwise (degrees)
//   sensor offsets: +x = right of robot center, +y = forward, measured to the sensor's FACE

#include "robot.h"
#include <cmath>
#include <cstdio>

namespace {

// Distance from the field center (0, 0) to each wall. Field measured 140.5 in wall to wall.
constexpr double FIELD_HALF = 140.5 / 2;
// V5 distance sensor limits, in inches.
constexpr double MAX_RANGE = 78.74;
constexpr double MIN_RANGE = 0.8;
// Beams that hit the wall at a steep angle read short (the beam is a cone). Skip them.
constexpr double MAX_INCIDENCE_DEG = 35.0;
// Confidence (0-63) is only reported past 200 mm; below that PROS gives 63.
constexpr int MIN_CONFIDENCE = 30;

constexpr double DEG_TO_RAD = M_PI / 180.0;

struct SensorMount {
    pros::Distance* sensor;
    double offX;  // inches, + = right of center
    double offY;  // inches, + = forward
    double angle; // degrees clockwise from robot forward: front 0, right 90, back 180, left 270
    const char* name;
};

// TODO: placeholder offsets from the Desmos graph. Measure the real robot to each sensor's face.
const SensorMount mounts[] = {
    {&frontSensor, 4, 0.5, 0, "front"},//change later cause cad isn't perfect
    {&rightSensor, 3.75, .268, 90, "right"},
    {&leftSensor, -3.75, .268, 270, "left"},
};

// Reading in inches, or a negative number if the sensor has nothing usable.
// mm and conf are the raw values, for logging.
double readInches(pros::Distance& sensor, int& mm, int& conf) {
    mm = sensor.get();
    conf = sensor.get_confidence();
    if (mm == PROS_ERR || mm <= 0 || mm >= 9999) return -1;
    if (mm > 200 && conf < MIN_CONFIDENCE) return -1;
    double inches = mm / 25.4;
    if (inches < MIN_RANGE || inches > MAX_RANGE) return -1;
    return inches;
}

// One sensor's usable wall reading, solved for the robot center on one axis.
struct WallReading {
    bool isX;      // true = solved x (hit a left/right wall), false = solved y
    double solved; // robot center x or y, inches
    double d;      // the reading, inches
    const char* name;
};

constexpr int NUM_MOUNTS = sizeof(mounts) / sizeof(mounts[0]);

// Fills `out` with every sensor that has a usable reading from `pose` and returns how many.
// The pose is used to pick which wall each sensor faces. With log = true, prints one line per
// sensor to the terminal (pros terminal).
int wallReadings(const lemlib::Pose& pose, WallReading out[NUM_MOUNTS], bool log = false) {
    double t = pose.theta * DEG_TO_RAD;
    double sinT = std::sin(t);
    double cosT = std::cos(t);
    int count = 0;

    if (log) std::printf("pose (%.2f, %.2f, %.1f)\n", pose.x, pose.y, pose.theta);

    for (const SensorMount& m : mounts) {
        // sensor offset from robot center, in field frame
        double dx = m.offX * cosT + m.offY * sinT;
        double dy = -m.offX * sinT + m.offY * cosT;
        double sx = pose.x + dx;
        double sy = pose.y + dy;

        // beam direction in field frame
        double a = t + m.angle * DEG_TO_RAD;
        double dirX = std::sin(a);
        double dirY = std::cos(a);

        // which wall the beam should hit from the guessed pose (closest positive hit)
        double tX = INFINITY, tY = INFINITY;
        if (dirX > 1e-9) tX = (FIELD_HALF - sx) / dirX;
        else if (dirX < -1e-9) tX = (-FIELD_HALF - sx) / dirX;
        if (dirY > 1e-9) tY = (FIELD_HALF - sy) / dirY;
        else if (dirY < -1e-9) tY = (-FIELD_HALF - sy) / dirY;

        bool hitsXWall = tX < tY; // x = +-FIELD_HALF (left/right walls)
        double expected = hitsXWall ? tX : tY;
        // angle between beam and the wall's normal
        double incidence = std::acos(std::fabs(hitsXWall ? dirX : dirY)) / DEG_TO_RAD;

        int mm, conf;
        double d = readInches(*m.sensor, mm, conf);

        const char* skip = nullptr;
        if (expected <= 0) skip = "odom says sensor is past the wall";
        else if (incidence > MAX_INCIDENCE_DEG) skip = "angle to wall too steep";
        else if (d < 0) skip = "no usable reading";

        if (log) {
            std::printf("  %-5s raw %4d mm  conf %2d  read %6.2f  expect %6.2f  wall %s  inc %4.1f",
                        m.name, mm, conf, mm / 25.4, expected, hitsXWall ? "x" : "y", incidence);
        }
        if (skip) {
            if (log) std::printf("  SKIP: %s\n", skip);
            continue;
        }

        // sensor position + d * beam direction = wall, solved for the robot center
        double solved;
        if (hitsXWall) {
            double wall = dirX > 0 ? FIELD_HALF : -FIELD_HALF;
            solved = wall - d * dirX - dx;
        } else {
            double wall = dirY > 0 ? FIELD_HALF : -FIELD_HALF;
            solved = wall - d * dirY - dy;
        }
        if (log) std::printf("  -> %s = %.2f\n", hitsXWall ? "x" : "y", solved);
        out[count++] = {hitsXWall, solved, d, m.name};
    }
    return count;
}

// Works out the robot's x and/or y from the walls, using the current odom pose as the guess.
// hasX/hasY say which axes got at least one good reading. Doesn't change the odom pose.
void wallEstimate(double& x, bool& hasX, double& y, bool& hasY, bool log = false) {
    WallReading readings[NUM_MOUNTS];
    int count = wallReadings(chassis.getPose(), readings, log);

    // readings are weighted by 1/d^2: the sensor error grows with distance (~5% past 200 mm),
    // so a far sensor shouldn't pull the answer around or make it jump when it drops out
    double xSum = 0, ySum = 0;
    double xWeight = 0, yWeight = 0;
    for (int i = 0; i < count; i++) {
        double weight = 1.0 / (readings[i].d * readings[i].d);
        if (readings[i].isX) xSum += readings[i].solved * weight, xWeight += weight;
        else ySum += readings[i].solved * weight, yWeight += weight;
    }

    hasX = xWeight > 0;
    hasY = yWeight > 0;
    x = hasX ? xSum / xWeight : 0;
    y = hasY ? ySum / yWeight : 0;
}

// ---- pose fusion tuning (see startPoseFusion) ----
// Odom variance (in^2) added per inch driven. 0.01 = about 1 in of drift after 100 in.
constexpr double FUSE_Q_PER_INCH = 0.01;
// Odom variance right after a reset (0.25 = sigma 0.5 in), and a cap so it can't grow forever.
constexpr double FUSE_P_RESET = 0.25;
constexpr double FUSE_P_MAX = 100.0;
// Sensor noise: sigma = max(SENSOR_SIGMA_MIN, SENSOR_SIGMA_FRAC * reading). From the V5 spec:
// +-15 mm below 200 mm, ~5% past that.
constexpr double SENSOR_SIGMA_MIN = 0.6;
constexpr double SENSOR_SIGMA_FRAC = 0.05;
// Skip a reading if it's more than this many sigmas from odom (robot/game object in the way).
constexpr double FUSE_GATE_SIGMAS = 3.0;
// Skip updates while turning faster than this; the reading lags and the beam sweeps the wall.
constexpr double FUSE_MAX_TURN_RATE = 60.0; // deg/s
// A pose jump bigger than this in one loop means someone called setPose; treat it as a reset.
constexpr double FUSE_TELEPORT_IN = 5.0;
// Sensors update about every 33 ms, so faster than this would reuse the same reading.
constexpr int FUSE_LOOP_MS = 35;

// Odom variance for x and y. Written by the fusion task and distanceReset().
double fuseP[2] = {FUSE_P_RESET, FUSE_P_RESET};

} // namespace

bool poseFusionEnabled = true;

bool distanceReset(double heading, bool resetX, bool resetY, int samples) {
    // set the heading first, so the wall each sensor faces is picked with the right angle
    if (!std::isnan(heading)) {
        lemlib::Pose now = chassis.getPose();
        chassis.setPose(now.x, now.y, heading);
    }

    // average several readings; the sensors update about every 33 ms
    double xSum = 0, ySum = 0;
    int xCount = 0, yCount = 0;
    for (int i = 0; i < samples; i++) {
        if (i > 0) pros::delay(35);
        double x, y;
        bool hasX, hasY;
        wallEstimate(x, hasX, y, hasY);
        if (hasX) xSum += x, xCount++;
        if (hasY) ySum += y, yCount++;
    }

    bool fixX = resetX && xCount > 0;
    bool fixY = resetY && yCount > 0;
    if (!fixX && !fixY) return false;

    // re-read the pose so heading/odom drift during the loop isn't lost
    lemlib::Pose now = chassis.getPose();
    chassis.setPose(fixX ? xSum / xCount : now.x, fixY ? ySum / yCount : now.y, now.theta);
    if (fixX) fuseP[0] = FUSE_P_RESET;
    if (fixY) fuseP[1] = FUSE_P_RESET;
    return true;
}

void startPoseFusion() {
    pros::Task fusion([]() {
        lemlib::Pose last = chassis.getPose();
        int loops = 0;
        while (true) {
            pros::delay(FUSE_LOOP_MS);
            lemlib::Pose pose = chassis.getPose();
            bool log = loops++ % 15 == 0; // terminal details every ~0.5 s

            // predict: the farther odom has driven since the last fix, the less we trust it
            double moved = std::hypot(pose.x - last.x, pose.y - last.y);
            for (double& p : fuseP) {
                if (moved > FUSE_TELEPORT_IN) p = FUSE_P_RESET;
                else p = std::fmin(p + FUSE_Q_PER_INCH * moved, FUSE_P_MAX);
            }
            last = pose;

            double gain[2] = {-1, -1}; // last K used per axis, -1 = no update (for the screen)
            bool turningFast = std::fabs(imu.get_gyro_rate().z) > FUSE_MAX_TURN_RATE;
            if (poseFusionEnabled && !turningFast) {
                WallReading readings[NUM_MOUNTS];
                int count = wallReadings(pose, readings, log);

                // update: one 1D Kalman step per reading
                double fused[2] = {pose.x, pose.y};
                for (int i = 0; i < count; i++) {
                    int axis = readings[i].isX ? 0 : 1;
                    double sigma = std::fmax(SENSOR_SIGMA_MIN, SENSOR_SIGMA_FRAC * readings[i].d);
                    double r = sigma * sigma;
                    double innovation = readings[i].solved - fused[axis];
                    if (std::fabs(innovation) > FUSE_GATE_SIGMAS * std::sqrt(fuseP[axis] + r)) {
                        if (log) std::printf("  %s: innov %.2f GATED\n", readings[i].name, innovation);
                        continue;
                    }
                    double k = fuseP[axis] / (fuseP[axis] + r);
                    fused[axis] += k * innovation;
                    fuseP[axis] *= 1 - k;
                    gain[axis] = k;
                    if (log) std::printf("  %s: innov %.2f  K %.3f\n", readings[i].name, innovation, k);
                }

                // apply as a nudge on top of the latest pose so odom motion since `pose` isn't lost
                double nudgeX = fused[0] - pose.x;
                double nudgeY = fused[1] - pose.y;
                if (nudgeX != 0 || nudgeY != 0) {
                    lemlib::Pose now = chassis.getPose();
                    chassis.setPose(now.x + nudgeX, now.y + nudgeY, now.theta);
                    last.x += nudgeX; // our own nudge isn't travel
                    last.y += nudgeY;
                }
            }

            char kx[8] = "--", ky[8] = "--";
            if (gain[0] >= 0) std::snprintf(kx, sizeof(kx), "%.2f", gain[0]);
            if (gain[1] >= 0) std::snprintf(ky, sizeof(ky), "%.2f", gain[1]);
            screenPrint(7, "Kx %s Ky %s  sx %.2f sy %.2f%s", kx, ky, std::sqrt(fuseP[0]),
                        std::sqrt(fuseP[1]), turningFast ? " TURN" : "");
        }
    });
}

void startWallEstimatePrinter() {
    pros::Task printer([]() {
        int loops = 0;
        while (true) {
            double x, y;
            bool hasX, hasY;
            // per-sensor details to the terminal every ~0.5 s
            wallEstimate(x, hasX, y, hasY, loops % 10 == 0);
            loops++;

            char xText[12] = "--", yText[12] = "--";
            if (hasX) std::snprintf(xText, sizeof(xText), "%.2f", x);
            if (hasY) std::snprintf(yText, sizeof(yText), "%.2f", y);
            screenPrint(7, "Wall X: %s  Y: %s", xText, yText);
            pros::delay(50);
        }
    });
}
