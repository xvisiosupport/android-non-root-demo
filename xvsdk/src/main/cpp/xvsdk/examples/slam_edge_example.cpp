#define _USE_MATH_DEFINES

#include <xv-sdk.h>

#include <iostream>
#include <thread>
#include <atomic>
#include <cmath>

#include "frequency_counter.hpp"

void onPose(xv::Pose const& pose){

    // use the conversion function to get the pitch, yaw and roll of the orientation
    // (be carefull of gimbal lock of Euler angles, that is why Euler angles are not in Pose structure)
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
    static FrequencyCounter fps;
    fps.tic();
    if (fps.count() % 120 == 1) {
        std::cout << "Current SLAM : " << fps.fps() << " Hz [timestamp=" << pose.hostTimestamp() << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                  << " pitch="  << pitchYawRoll[0]*180./M_PI << "°" << " yaw="  << pitchYawRoll[1]*180./M_PI << "°" << " roll="  << pitchYawRoll[2]*180./M_PI << "°"
                  << std::endl;
    }
}

std::shared_ptr<const xv::ColorImage> lastColorImage;

int main( int /*argc*/, char* /*argv*/[] )
{

    // return a map of devices with serial number as key, wait at most 3 seconds if no device detected
    auto devices = xv::getDevices(3.);

    // if no device: quit
    if (devices.empty()) {
        std::cerr << "Timeout for device detection." << std::endl;
        return EXIT_FAILURE;
    }

    // take the first device in the map
    auto device = devices.begin()->second;

    if (!device->slam()) {
        std::cerr << "SLAM not supported." << std::endl;
        return EXIT_FAILURE;
    }

    // register a callback to get the current pose based on edge SLAM, the callback is call at IMU framerate.
    device->slam()->registerCallback(onPose);

    // if access to full stereo camera images it is possible to get images via a callback :
    if (device->colorCamera()) {
        device->colorCamera()->registerCallback([](xv::ColorImage const & image) {
            lastColorImage = std::make_shared<xv::ColorImage>(image);
            static FrequencyCounter fps;
            fps.tic();
            if (fps.count() % 120 == 1) {
                std::cout << "Current color image : " << fps.fps() << " Hz [timestamp=" << image.hostTimestamp << " width=" << image.width << " height=" << image.height
                          << "]" << std::endl;
            }
        });
    } else {
        std::cerr << "Access to color camera not supported." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Press enter to start SLAM ..." << std::endl;
    std::cin.get();

    // start the SLAM
    device->slam()->start(xv::Slam::Mode::Edge);

    std::cout << "Press enter to stop SLAM ..." << std::endl;
    std::cin.get();

    // to get the a color image in RGB format, can do the conversion with:
    xv::RgbImage rgbImage = lastColorImage->toRgb();

    // stop the slam
    device->slam()->stop();

    return EXIT_SUCCESS;
}
