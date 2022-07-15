#define _USE_MATH_DEFINES

#include <xv-sdk.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

#include "frequency_counter.hpp"

// A pointer to the last retrieved depth image
std::mutex colorImageMtx;
std::shared_ptr<const xv::ColorImage> colorImage;
void onColorImage(xv::ColorImage const & im) {
    static FrequencyCounter fps;
    fps.tic();
    if (fps.count() % 100 == 0) {
        std::cout << fps.count() << " color images (" << im.height << "x" << im.width << ") at " << fps.fps() << " fps" << std::endl;
    }
    colorImageMtx.lock();
    colorImage = std::make_shared<xv::ColorImage>(im);
    colorImageMtx.unlock();
}

std::mutex depthImageMtx;
std::shared_ptr<const xv::DepthImage> depthImage;
void onDepthImage(xv::DepthImage const & im) {
    static FrequencyCounter fps;
    fps.tic();
    if (fps.count() % 60 == 0) {
        std::cout << fps.count() << " depth images (" << im.height << "x" << im.width << ") at " << fps.fps() << " fps" << std::endl;
    }
    depthImageMtx.lock();
    depthImage = std::make_shared<xv::DepthImage>(im);
    depthImageMtx.unlock();
}


int main( int /*argc*/, char* /*argv*/[] )
{

    // return a map of devices with serial number as key, wait at most 3 seconds if no device detected
    auto devices = xv::getDevices(3.);

    // if no device: quit
    if (devices.empty()) {
        std::cout << "Timeout for device detection." << std::endl;
        return EXIT_FAILURE;
    }

    // take the first device in the map
    auto device = devices.begin()->second;

    // if the device has color camera ...
    if (device->colorCamera()) {
        // register a callback to get image
        device->colorCamera()->registerCallback(onColorImage);
    } else {
        std::cout << "Device do not have color camera." << std::endl;
    }

    std::shared_ptr<xv::TofCamera> tof;

    // check if the device has a ToF camera ...
    if (device->tofCamera()) {
        tof = device->tofCamera();
        // register the callback with the ToF image
        device->tofCamera()->registerCallback(onDepthImage);
    } else {
        std::cout << "Device do not have ToF camera." << std::endl;
    }

    // simulate a 60Hz loop to get the pose and have a localized ToF point cloud
    std::atomic<bool> stop(false);
    std::thread threadLoop60Hz([&stop, &device, &tof]{

        while (!stop) {

            auto now = std::chrono::steady_clock::now();

            if (tof) {
                depthImageMtx.lock();
                auto depthImage_ = depthImage;
                depthImageMtx.unlock();
                // compute the point cloud from the depth image
                auto pointCloud = tof->depthImageToPointCloud(*depthImage_);
                if (pointCloud) {
                    static FrequencyCounter fps;
                    fps.tic();
                    if (fps.count() % 120 == 1) {
                        std::cout << "Point cloud: " << fps.count() << " [size=" << pointCloud->points.size() << "]" << std::endl;
                        xv::Pose pose;
                        auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                        // get the pose at the point cloud timestamp
                        if (device->slam() && device->slam()->getPoseAt(pose, pointCloud->hostTimestamp)) {
                            std::cout << "Device pose at the point cloud: [timestamp=" << pose.hostTimestamp() << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                                      << " pitch="  << pitchYawRoll[0]*180./M_PI << "°" << " yaw="  << pitchYawRoll[1]*180./M_PI << "°" << " roll="  << pitchYawRoll[2]*180./M_PI << "°"
                                      << std::endl;
                       }
                    }
                }
            }

            xv::Pose pose;
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

            // to simulate the 60Hz loop
            std::this_thread::sleep_until(now+std::chrono::microseconds(long(1./60.*1e6)));
        }

    });

    std::cout << "Press enter to start SLAM ..." << std::endl;
    std::cin.get();

    if (!device->slam()->start()) {
        std::cout << "Something went wrong when starting the SLAM." << std::endl;
        return EXIT_FAILURE;
    }

   std::cout << "Press enter to stop SLAM ..." << std::endl;
   std::cin.get();

   device->slam()->stop();
   stop = true;

   if (threadLoop60Hz.joinable()) {
       threadLoop60Hz.join();
   }

   std::cout << "Press enter to stop the process ..." << std::endl;
   std::cin.get();


   return EXIT_SUCCESS;
}
