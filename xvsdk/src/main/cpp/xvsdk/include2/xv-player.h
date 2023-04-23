#pragma once

#include "xv-sdk-ex.h"

#include <fstream>
#include <future>

namespace xv {

/**
 * @brief Class to replay a recorded dataset.
 */
class Player {

    std::shared_ptr<xv::DeviceEx> m_device;
    std::ifstream m_imuFile;
    std::ifstream m_fisheyesFile;
    std::ifstream m_fisheyesKeyPointsFile;
    
    std::atomic<bool> m_stop, m_finished;
    std::chrono::steady_clock::time_point m_t0;
    std::chrono::steady_clock::time_point m_tOffset;
    double m_hostTimestampOffset;
    
    std::future<bool> m_imuEnd;
    std::future<bool> m_fisheyesEnd;
    std::future<bool> m_fisheyesKeyPointsEnd;
    
    std::shared_ptr<xv::DeviceEx> init(std::string const& devicePath);
    
public:

     /**
      * @brief Create the player
      * @param devicePath : device file (correspond to xv::fbs::Device serialization)
      * @param imuFile : file with IMU data
      * @param fisheyesFile : file with fisheye images data
      * @param fisheyesKeyPointsFile : file with fisheye keypoints data
      */
     Player(std::string const& devicePath, std::string const& imuFile, std::string const& fisheyesFile, std::string const& fisheyesKeyPointsFile);
     
    std::shared_ptr<xv::DeviceEx> device();
    
    bool finished();
    
    bool start();
    
    /**
     * @brief Host timestamp offset added to the recorded timestamp to sync with the current timestamp of the replay
     */
    double hostTimestmapOffset() const;
    
};

}
