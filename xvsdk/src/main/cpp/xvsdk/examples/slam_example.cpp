#define _USE_MATH_DEFINES

#include <xv-sdk.h>

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

#include "frequency_counter.hpp"

void onPose(xv::Pose const& pose){

    // use the conversion function to get the pitch, yaw and roll of the orientation
    // (be carefull of gimbal lock of Euler angles, that is why Euler angles are not in Pose structure)
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
    static FrequencyCounter fps;
    fps.tic();
    if (fps.count() % 500 == 1) {
        std::cout << "SLAM pose callback : " << fps.fps() << " Hz [timestamp=" << pose.hostTimestamp() << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                  << " pitch="  << pitchYawRoll[0]*180./M_PI << "°" << " yaw="  << pitchYawRoll[1]*180./M_PI << "°" << " roll="  << pitchYawRoll[2]*180./M_PI << "°"
                  << " confidence=" << pose.confidence() << std::endl;
    }
}

std::mutex stereoImageMtx;
std::shared_ptr<const xv::FisheyeImages> lastStereoImage;

int main( int /*argc*/, char* /*argv*/[] )
{

    // may change the log level this way :
    xv::setLogLevel(xv::LogLevel::debug);

    // return a map of devices with serial number as key, wait at most 3 seconds if no device detected
    auto devices = xv::getDevices(5.);

    // if no device: quit
    if (devices.empty()) {
        std::cerr << "Timeout for device detection." << std::endl;
        return EXIT_FAILURE;
    }

    // take the first device in the map
    auto device = devices.begin()->second;

    if (!device->slam()) {
        std::cerr << "Host SLAM algorithm not supported." << std::endl;
        return EXIT_FAILURE;
    }

    // register a callback to get the current pose based on SLAM, the callback is call at IMU framerate.
    // Framerate can be high, for AR/VR device we recommand using `getPose` function to get the current 6dof pose with no latency.
    device->slam()->registerCallback(onPose);

    // if access to full stereo camera images it is possible to get images via a callback :
    if (device->fisheyeCameras()) {
        device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const & images) {
            stereoImageMtx.lock();
            lastStereoImage = std::make_shared<xv::FisheyeImages>(images);
            stereoImageMtx.unlock();
            static FrequencyCounter fps;
            fps.tic();
            if (fps.count() % 30 == 1) {
                std::cout << "Fisheyes : " << fps.fps() << " Hz" << std::endl;
            }
        });
    }

    // Simulate a 60Hz loop to show the use of the `getPose` function.
    std::atomic<bool> stop(false);
    std::thread threadLoop60Hz([&stop, &device]{

        while (!stop) {

            auto now = std::chrono::steady_clock::now();

            xv::Pose pose;
            // get the pose at current time (no latency because internally compensated with small prediction after the last IMU data received) ...
            if (device->slam()->getPose(pose)) {
                auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                static FrequencyCounter fps;
                fps.tic();
                if (fps.count() % 120 == 1) {
                    std::cout << "Current SLAM : " << fps.fps() << " Hz [timestamp=" << pose.hostTimestamp() << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                              << " pitch="  << pitchYawRoll[0]*180./M_PI << "°" << " yaw="  << pitchYawRoll[1]*180./M_PI << "°" << " roll="  << pitchYawRoll[2]*180./M_PI << "°"
                              << "confidence=" << pose.confidence() << std::endl;
                }
            }

            // it is also possible to make prediction and to get the pose 30 ms in the future:
            xv::Pose pose30ms;
            device->slam()->getPose(pose30ms, 0.030);

            stereoImageMtx.lock();
            if (lastStereoImage) {
                // it is also possible to get the pose at a given time, for example the last stereo images time:
                xv::Pose poseAtStereoImage;
                device->slam()->getPoseAt(poseAtStereoImage, lastStereoImage->hostTimestamp);
            }
            stereoImageMtx.unlock();

            // it is also possible to get the pose of the device at a specific timestamp (based on std::chrono::steady_clock)
            // the pose cannot be to old and not too much in the future
            double t = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            t += 0.0123;
            xv::Pose poseAt;
            device->slam()->getPoseAt(poseAt, t);

            // to simulate the 60Hz loop
            std::this_thread::sleep_until(now+std::chrono::microseconds(long(1./60.*1e6)));
        }

    });

    std::cout << "Press enter to start SLAM ..." << std::endl;
    std::cin.get();

    // start the SLAM
    device->slam()->start();

    std::cout << "Press enter to stop SLAM ..." << std::endl;
    std::cin.get();

    // stop the slam
    device->slam()->stop();

    stop = true;

    if (threadLoop60Hz.joinable()) {
        threadLoop60Hz.join();
    }

    return EXIT_SUCCESS;
}
