#define _USE_MATH_DEFINES // for C++

#include <xv-sdk.h>

#include <iostream>
#include <thread>
#include <atomic>
#include <cmath>

#include "frequency_counter.hpp"

#include <iomanip>

void on3dof(xv::Orientation const & orientation){

    // use the conversion function to get the pitch, yaw and roll of the orientation
    // (be carefull of gimbal lock of Euler angles, that is why Euler angles are not in Pose structure)
    auto pitchYawRoll = xv::rotationToPitchYawRoll(orientation.rotation());
    static FrequencyCounter fps;
    fps.tic();
    if (fps.count() % 500 == 1) {
        std::cout << "3dof callback : " << fps.fps() << " Hz [timestamp=" << orientation.hostTimestamp
                  << " pitch="  << pitchYawRoll[0]*180./M_PI << "degree" << " yaw="  << pitchYawRoll[1]*180./M_PI << "degree" << " roll="  << pitchYawRoll[2]*180./M_PI << "degree"
                  << std::endl;
    }
}

void onPose(xv::Pose const& pose){

    // use the conversion function to get the pitch, yaw and roll of the orientation
    // (be carefull of gimbal lock of Euler angles, that is why Euler angles are not in Pose structure)
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
    static FrequencyCounter fps;
    fps.tic();
    if (fps.count() % 500 == 1) {
        std::cout << "SLAM pose callback : " << fps.fps() << " Hz [timestamp=" << pose.hostTimestamp() << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                  << " pitch="  << pitchYawRoll[0]*180./M_PI << "degree" << " yaw="  << pitchYawRoll[1]*180./M_PI << "degree" << " roll="  << pitchYawRoll[2]*180./M_PI << "degree"
                  << std::endl;
    }
}

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

    // register the orientation (3dof) tracking using IMU only (also possible to use device->imuSensor()->getOrientation() for direct access of current 3dof)
    device->orientationStream()->registerCallback(on3dof);

    // start the 3dof tracking
    device->orientationStream()->start();

    auto display = device->display();

    if (display && display->calibration().size()==2) {
        std::cout << "left eye dispaly: " << display->calibration()[0].pdcm[0].w << "x" << display->calibration()[0].pdcm[0].h << std::endl;
        std::cout << "right eye dispaly: " << display->calibration()[1].pdcm[0].w << "x" << display->calibration()[1].pdcm[0].h << std::endl;
    }

    // register a callback to get the current pose based on SLAM, the callback is call at IMU framerate.
    // Framerate can be high, for AR/VR device we recommand using `getPose` function to get the current 6dof pose with no latency.
    device->slam()->registerCallback(onPose);



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
                              << std::endl;
                }
            }

            // it is also possible to make prediction and to get the pose 30 ms in the future:
            xv::Pose pose30ms;
            device->slam()->getPose(pose30ms, 0.030);


            // the same way for the orientation (if the slam is not started) using only the IMU:
            xv::Orientation orientaion;
            if (device->orientationStream()->get(orientaion, 0.30)) {
                auto pitchYawRoll = xv::rotationToPitchYawRoll(orientaion.rotation());
                static FrequencyCounter fps;
                fps.tic();
                if (fps.count() % 120 == 1) {
                    std::cout << "Current 3dof : " << fps.fps() << " Hz [timestamp=" << orientaion.hostTimestamp
                              << " pitch="  << pitchYawRoll[0]*180./M_PI << "°" << " yaw="  << pitchYawRoll[1]*180./M_PI << "°" << " roll="  << pitchYawRoll[2]*180./M_PI << "°"
                              << std::endl;
                }
            }

            // to simulate the 60Hz loop
            std::this_thread::sleep_until(now+std::chrono::microseconds(long(1./60.*1e6)));
        }

    });

    std::cout << "Press enter to start SLAM ..." << std::endl;
    std::cin.get();

    // stop the 3dof tracking based on IMU only
    device->orientationStream()->stop();

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
