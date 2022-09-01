#define _USE_MATH_DEFINES

#include <cstdlib>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <signal.h>
#include <cstring>

#include <xv-sdk.h>
#include "colors.h"

#define USE_EX
//#define USE_PRIVATE

bool s_stop = false;
static std::map<std::string,int> enableDevMap;

static struct xv::sgbm_config global_config = {
    1 ,//enable_dewarp
    1.0, //dewarp_zoom_factor
    0, //enable_disparity
    1, //enable_depth
    0, //enable_point_cloud
    0.08, //baseline
    96, //fov
    255, //disparity_confidence_threshold
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
    1, //enable_gamma
    2.2, //gamma_value
    0, //enable_gaussian
    0, //mode
    8000, //max_distance
    100, //min_distance
};


#ifdef USE_EX
#include "../../include2/xv-sdk-ex.h"
#endif
#ifdef USE_PRIVATE
#include "xv-sdk-private.h"
#endif

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>

int s_x=-1,s_y=-1;
static int k = 0;
void onMouse(int event,int x,int y,int flag,void* _user_data)
{
    switch (event)
    {
    case cv::EVENT_MOUSEMOVE:
        s_x = x;
        s_y = y;
        break;
    default:
        break;
    }
}

cv::Mat raw_to_opencv( std::shared_ptr<const xv::ColorImage> rgb);
cv::Mat raw_to_opencv( std::shared_ptr<const xv::DepthImage> tof);
cv::Mat raw_to_opencv_tof_ir( const xv::GrayScaleImage& tof_ir);
cv::Mat raw_to_opencv_tof_ir_grey( const xv::GrayScaleImage& tof_ir);
std::pair<cv::Mat,cv::Mat> raw_to_opencv( std::shared_ptr<const xv::FisheyeImages> stereo);
cv::Mat raw_to_opencv( std::shared_ptr<const xv::DepthColorImage> rgbd);
cv::Mat raw_to_opencv(std::shared_ptr<const xv::SgbmImage> sbgm_image);
std::pair<cv::Mat,cv::Mat> raw_to_opencv(std::shared_ptr<const xv::EyetrackingImage> eyetracking);

void add_tags(cv::Mat& im, std::vector<std::pair<int, std::array<xv::Vector2d, 4>>> const& tags) {
    for(auto const& t : tags){
        auto const& pts = t.second;
        std::size_t i=0;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[i+1][0],pts[i+1][1]), cv::Scalar(0,255,0) ); i++;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[i+1][0],pts[i+1][1]), cv::Scalar(0,255,0) ); i++;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[i+1][0],pts[i+1][1]), cv::Scalar(0,255,0) ); i++;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[0][0],pts[0][1]), cv::Scalar(0,255,0) );
        cv::putText(im, std::to_string(t.first),
                    cv::Point(pts[i][0], pts[i][1]), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.25,
                    CV_RGB(0, 255, 0), //font color
                    1.);
    }
}

#ifdef USE_EX

void add_tags(cv::Mat& im, std::vector<xv::TagDetection> const& tags) {
    for(auto const& t : tags){
        auto const& pts = t.corners;
        std::size_t i=0;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[i+1][0],pts[i+1][1]), cv::Scalar(0,255,0) ); i++;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[i+1][0],pts[i+1][1]), cv::Scalar(0,255,0) ); i++;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[i+1][0],pts[i+1][1]), cv::Scalar(0,255,0) ); i++;
        cv::line(im, cv::Point(pts[i][0],pts[i][1]), cv::Point(pts[0][0],pts[0][1]), cv::Scalar(0,255,0) );
        cv::putText(im, std::to_string(t.tagId),
                    cv::Point(pts[i][0], pts[i][1]), //top-left position
                    cv::FONT_HERSHEY_DUPLEX,
                    0.25,
                    CV_RGB(0, 255, 0), //font color
                    1.);
    }
}

std::pair<cv::Mat,cv::Mat> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo, std::shared_ptr<const xv::FisheyeKeyPoints<2,32>> keypoints, std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> tags)
{
    cv::Mat left;
    cv::Mat right;

    if (stereo) {
        auto const& leftInput = stereo->images[0];
        auto const& rightInput = stereo->images[1];
        if( leftInput.data != nullptr ){
            left = cv::Mat::zeros(leftInput.height, leftInput.width, CV_8UC1);
            std::memcpy(left.data, leftInput.data.get(), static_cast<size_t>(left.rows*left.cols));
        }
        if( rightInput.data != nullptr ){
            right = cv::Mat::zeros(rightInput.height, rightInput.width, CV_8UC1);
            std::memcpy(right.data, rightInput.data.get(), static_cast<size_t>(right.rows*right.cols));
        }
    } else {
        left = cv::Mat::zeros(400, 640, CV_8UC1);
        right = cv::Mat::zeros(400, 640, CV_8UC1);
    }

    cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

    if (keypoints) {
        const int size = 2;
        int s=0;
        for(unsigned int i=0; i<keypoints->descriptors[s].size; i++ ){
            auto p = keypoints->descriptors[s].keypoints.get() + i*2;
            cv::Point pt(*p, *(p+1));
            cv::line(left, pt - cv::Point(size,0), pt + cv::Point(size,0), cv::Scalar(0,0,255) );
            cv::line(left, pt - cv::Point(0,size), pt + cv::Point(0,size), cv::Scalar(0,0,255) );
        }
        s=1;
        for(unsigned int i=0; i<keypoints->descriptors[s].size; i++ ){
            auto p = keypoints->descriptors[s].keypoints.get() + i*2;
            cv::Point pt(*p, *(p+1));
            cv::line(right, pt - cv::Point(size,0), pt + cv::Point(size,0), cv::Scalar(0,0,255) );
            cv::line(right, pt - cv::Point(0,size), pt + cv::Point(0,size), cv::Scalar(0,0,255) );
        }
    }

        if (tags) {
            add_tags(left, *tags);
        }

    return {left, right};
}

std::array<cv::Mat,4> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo, std::shared_ptr<const xv::FisheyeKeyPoints<4,32>> keypoints, std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> tags)
{

    std::array<cv::Mat,4> images;

    for (auto& im : images) {
        im = cv::Mat::zeros(400, 640, CV_8UC1);
    }

    if (stereo) {
        std::size_t i=0;
        for (auto& im : stereo->images) {
            if (im.data != nullptr && i < images.size()) {
                std::memcpy(images[i].data, im.data.get(), static_cast<size_t>(images[i].rows*images[i].cols));
                cv::cvtColor(images[i], images[i], cv::COLOR_GRAY2BGR);
            }
            ++i;
        }
    } else {
        for (auto& im : images) {
            im = cv::Mat::zeros(400, 640, CV_8UC1);
        }
    }

    if (keypoints) {

        const int size = 2;
        std::size_t ii=0;
        for (auto& descriptors : keypoints->descriptors) {
            for(unsigned int i=0; i<descriptors.size; i++ ){
                auto p = descriptors.keypoints.get() + i*2;
                cv::Point pt(*p, *(p+1));
                cv::line(images[ii], pt - cv::Point(size,0), pt + cv::Point(size,0), cv::Scalar(0,0,255) );
                cv::line(images[ii], pt - cv::Point(0,size), pt + cv::Point(0,size), cv::Scalar(0,0,255) );
            }
            ++ii;
        }
    }

        if (tags) {
            add_tags(images[0], *tags);
        }

    return images;
}

std::pair<cv::Mat,cv::Mat> raw_to_opencv(std::shared_ptr<const xv::EyetrackingImage> eyetracking)
{
    cv::Mat left;
    cv::Mat right;

    if (eyetracking) {
        auto const& leftInput = eyetracking->images[0];
        auto const& rightInput = eyetracking->images[1];
        if( leftInput.data != nullptr ){
            left = cv::Mat::zeros(leftInput.height, leftInput.width, CV_8UC1);
            std::memcpy(left.data, leftInput.data.get(), static_cast<size_t>(left.rows*left.cols));
        }
        if( rightInput.data != nullptr ){
            right = cv::Mat::zeros(rightInput.height, rightInput.width, CV_8UC1);
            std::memcpy(right.data, rightInput.data.get(), static_cast<size_t>(right.rows*right.cols));
        }
    } else {
        left = cv::Mat::zeros(400, 640, CV_8UC1);
        right = cv::Mat::zeros(400, 640, CV_8UC1);
    }

    cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

    return {left, right};
}

#endif


std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::shared_ptr<const xv::DepthImage> s_tof = nullptr;
std::shared_ptr<const xv::GrayScaleImage> s_ir = nullptr;
std::shared_ptr<const xv::FisheyeImages> s_stereo = nullptr;
std::shared_ptr<const xv::DepthColorImage> s_depthColor = nullptr;
std::shared_ptr<const xv::SgbmImage> s_ptr_sgbm = nullptr;
std::shared_ptr<const xv::EyetrackingImage> s_eyetracking = nullptr;

#ifdef USE_EX
std::shared_ptr<const xv::FisheyeKeyPoints<2,32>> s_keypoints = nullptr;
std::shared_ptr<const xv::FisheyeKeyPoints<4,32>> s_keypoints4cam = nullptr;
std::mutex s_mtx_tags;
std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> s_tags;
std::mutex s_mtx_rgb_tags;
xv::GrayScaleImage s_rgb_gray;
std::vector<xv::TagDetection> s_rgb_tags;
#endif

std::mutex s_mtx_rgb;
std::mutex s_mtx_tof;
std::mutex s_mtx_depthColor;
std::mutex s_mtx_ir;
std::mutex s_mtx_stereo;
std::mutex s_mtx_sgbm;
std::mutex s_mtx_eyetracking;

void display() {
    if (enableDevMap["fisheye"]) {
        cv::namedWindow("Left");
        cv::moveWindow("Left", 20, 20);
        cv::namedWindow("Right");
        cv::moveWindow("Right", 660, 20);
    }
    if (enableDevMap["rgb"]) {
        cv::namedWindow("RGB");
        cv::moveWindow("RGB", 20, 462);
    }
    if (enableDevMap["tof"]) {
        cv::namedWindow("TOF");
        cv::moveWindow("TOF", 500, 462);
        cv::namedWindow("IR");
        cv::moveWindow("IR", 500 + 640, 462);
        cv::namedWindow("Depth");
        cv::moveWindow("Depth", 500 , 650);
    }
    if (enableDevMap["sgbm"]) {
        cv::namedWindow("ET-Left");
        cv::moveWindow("ET-Left", 600, 60);
        cv::namedWindow("ET-Right");
        cv::moveWindow("ET-Right", 800, 60);
    }
    cv::waitKey(1);

    while( !s_stop ){
        std::shared_ptr<const xv::ColorImage> rgb = nullptr;
        std::shared_ptr<const xv::DepthImage> tof = nullptr;
        std::shared_ptr<const xv::GrayScaleImage> ir = nullptr;
        std::shared_ptr<const xv::FisheyeImages> stereo = nullptr;
        std::shared_ptr<const xv::SgbmImage> ptr_sgbm = nullptr;
        std::shared_ptr<const xv::EyetrackingImage> eyetracking = nullptr;
#ifdef USE_EX
        std::shared_ptr<const xv::FisheyeKeyPoints<2,32>> keypoints = nullptr;
        std::shared_ptr<const xv::FisheyeKeyPoints<4,32>> keypoints4cam = nullptr;
        std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> tags;
        decltype (s_rgb_tags) rgb_tags;
#endif
        if (enableDevMap["fisheye"]) {
            s_mtx_stereo.lock();
            stereo = s_stereo;
#ifdef USE_EX
            keypoints = s_keypoints;
            keypoints4cam = s_keypoints4cam;
            s_mtx_tags.lock();
            tags = s_tags;
            s_mtx_tags.unlock();
            s_mtx_rgb_tags.lock();
            rgb_tags = s_rgb_tags;
            s_mtx_rgb_tags.unlock();
#endif
            s_mtx_stereo.unlock();

#ifdef USE_EX
            if (keypoints) {
                auto imgs = raw_to_opencv(stereo, keypoints, tags);
                cv::imshow("Left", imgs.first);
                cv::imshow("Right", imgs.second);
            }
            if (keypoints4cam) {
                auto imgs = raw_to_opencv(stereo, keypoints4cam, tags);
                for (std::size_t i=0; i<imgs.size(); ++i) {
                    cv::imshow("Cam"+std::to_string(i), imgs[i]);
                }
            }
#else
            if (stereo) {
                auto imgs = raw_to_opencv(stereo);
                cv::imshow("Left", imgs.first);
                cv::imshow("Right", imgs.second);
            }
#endif
        }
        if (enableDevMap["rgb"]) {
#ifdef USE_EX
            s_mtx_rgb_tags.lock();
            rgb_tags = s_rgb_tags;
            s_mtx_rgb_tags.unlock();
#endif
            s_mtx_rgb.lock();
            rgb = s_rgb;
            s_mtx_rgb.unlock();
            if (rgb && rgb->width>0 && rgb->height>0) {
                cv::Mat img = raw_to_opencv(rgb);
#ifdef USE_EX
                add_tags(img, rgb_tags);
#else
                cv::resize(img, img, cv::Size(), 0.25, 0.25);
#endif
                cv::imshow("RGB", img);
            }
        }
        if (enableDevMap["tof"]) {
            s_mtx_tof.lock();
            tof = s_tof;
            s_mtx_tof.unlock();
            if (tof) {
                cv::Mat img = raw_to_opencv(tof);
                if (img.rows>0 && img.cols>0)
                    cv::imshow("TOF", img);
            }
            s_mtx_depthColor.lock();
            auto depthColor = s_depthColor;
            s_mtx_depthColor.unlock();
            if (depthColor) {
                cv::Mat img = raw_to_opencv(depthColor);
                if (img.rows>0 && img.cols>0)
                    cv::imshow("RGBD (depth)", img);
            }
            s_mtx_ir.lock();
            ir = s_ir;
            s_mtx_ir.unlock();
            if (ir) {
                cv::Mat img = raw_to_opencv_tof_ir(*ir);
                if (img.rows>0 && img.cols>0)
                    cv::imshow("IR", img);
                cv::Mat img_grey = raw_to_opencv_tof_ir_grey(*ir);
                if (img_grey.rows>0 && img_grey.cols>0)
                    cv::imshow("IR (grey)", img_grey);
            }
        }

        if (enableDevMap["sgbm"]) {
            s_mtx_sgbm.lock();
            ptr_sgbm = s_ptr_sgbm;
            s_mtx_sgbm.unlock();


            if(ptr_sgbm)
            {
                if(ptr_sgbm->type == xv::SgbmImage::Type::Depth)
                {
#ifdef USE_FILLHOLES
                    {
                        int hole = 1;
                        xv::SgbmImage img = fillHoles(*ptr_sgbm.get(), [](uint16_t d){return d < d1  || d > d2; }, hole);
                        ptr_sgbm = make_shared<xv::SgbmImage>(img);
                    }
#endif
                    cv::Mat img = raw_to_opencv(ptr_sgbm);
                    char text[256];
                    uint16_t* p16 = (uint16_t*)ptr_sgbm->data.get();

                    if( s_x!=-1 && ( s_x > 0 && s_x < ptr_sgbm->width && s_y > 0 && s_y < ptr_sgbm->height ) )
                    {
                        int width = ptr_sgbm->width;
                        int height = ptr_sgbm->height;
                        if(p16[s_x+s_y*width]==0)
                        {
                            if(k++%20==0)
                            {
                                memset(text,0,256);
                                sprintf(text,"x:%d,y:%d depth:%d mm",s_x,s_y,p16[s_x+s_y*width]);
                            }
                        }
                        else
                        {
                            memset(text,0,256);
                            sprintf(text,"x:%d,y:%d depth:%d mm",s_x,s_y,p16[s_x+s_y*width]);
                        }

                        putText(img,text,cv::Point(25,height-30),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,255));
                    }
                    imshow("Depth",img);
                }

            }
        }

        
        if (enableDevMap["eyetracking"]) {
            s_mtx_eyetracking.lock();
            eyetracking = s_eyetracking;
            s_mtx_eyetracking.unlock();
            if (eyetracking) {
                auto imgs = raw_to_opencv(eyetracking);
                cv::imshow("Left", imgs.first);
                cv::imshow("Right", imgs.second);
            }
        }
        cv::waitKey(1);
    }
}
#endif

#include "fps_count.hpp"

std::string timeShowStr(std::int64_t edgeTimestampUs, double hostTimestamp) {
    char s[1024];
    double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
    std::sprintf(s, " (device=%lld host=%.4f now=%.4f delay=%.4f) ", (long long)edgeTimestampUs, hostTimestamp, now, now-hostTimestamp);
    return std::string(s);
}
std::string timeShowStr(double hostTimestamp) {
    char s[1024];
    double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
    std::sprintf(s, " (host=%.4f now=%.4f delay=%.4f) ", hostTimestamp, now, now-hostTimestamp);
    return std::string(s);
}

int main( int argc, char* argv[] ) try
{
    std::cout << "xvsdk version: " << xv::version() << std::endl;

    xv::setLogLevel(xv::LogLevel::debug);

    std::string json = "";
    if (argc >1 && *argv[1]!='\0') {
        std::ifstream ifs( argv[1] );
        if( !ifs.is_open() ){
            std::cerr << "Failed to open: " << argv[1] << std::endl;
        }
        else
        {
            std::stringstream fbuf;
            fbuf << ifs.rdbuf();
            json = fbuf.str();
        }
    }
    enableDevMap["rgb"] = true;
    enableDevMap["tof"]= true;
    enableDevMap["fisheye"] = true;
    enableDevMap["sgbm"] = true;
    enableDevMap["slam"] = true;
    enableDevMap["slam_edge"] = true;
    enableDevMap["imu"] = true;
    enableDevMap["eyetracking"] = true;
    enableDevMap["sync"] = false;
    enableDevMap["dewarp"] = true;
    enableDevMap["VGA"] = true;
    enableDevMap["720P"] = false;
    enableDevMap["tof_mode"] = 3;//default lablize sf
    enableDevMap["tof_point_cloud"] = false;
    enableDevMap["log"]=true;
    enableDevMap["ir"]=true;
    if (argc == 3)
    {
        std::string enableDevStr(argv[2]);
        enableDevStr+=" ";
        int index,index2;
        while(true)
        {
            index = enableDevStr.find(' ');
            if(index == std::string::npos)
            {
                break;
            }
            auto one = enableDevStr.substr(0,index);
            index2 = one.find(':');
            auto key = one.substr(0,index2);
            auto value = one.substr(index2+1,one.size()-index2-1)=="1"?true:false;
            enableDevStr = enableDevStr.substr(index+1);
            std::cout<<key<<" : "<<value<<std::endl;
            enableDevMap[key] = value;
        }
    }

    auto devices = xv::getDevices(10., json);
    if(enableDevMap["log"])
    {
        xv::setLogLevel(xv::LogLevel::debug);
    }
    std::ofstream ofs;
    if (devices.empty())
    {
        std::cout << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }

    auto device = devices.begin()->second;

    enableDevMap["rgb"] &= device->colorCamera() != nullptr;
    enableDevMap["tof"] &= device->tofCamera() != nullptr;
    enableDevMap["fisheye"] &= device->fisheyeCameras() != nullptr;
    enableDevMap["sgbm"] &= device->sgbmCamera() != nullptr;
    enableDevMap["slam"] &= device->slam() != nullptr;
    enableDevMap["imu"] &= device->imuSensor() != nullptr;
    enableDevMap["eyetracking"] &= device->eyetracking() != nullptr;

    if(enableDevMap["dewarp"])
    {
        global_config.enable_dewarp = 1;
    }
    else
    {
        global_config.enable_dewarp = 0;
    }

    device->enableSync(enableDevMap["sync"]);

    if (enableDevMap["rgb"])
    {
        device->colorCamera()->registerCallback( [](xv::ColorImage const & rgb){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%25==0){
                if(enableDevMap["log"])
                {
                    std::cout << "rgb      " << timeShowStr(rgb.edgeTimestampUs, rgb.hostTimestamp)
                            << rgb.width << "x" << rgb.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
        });
        device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1920x1080);
        device->colorCamera()->start();
    }
    else
    {
        std::cout << "No RGB camera.\n";
    }

    if (enableDevMap["tof"]) {

#ifdef USE_PRIVATE
        auto devPriv = std::dynamic_pointer_cast<xv::DevicePrivate>(device);
        devPriv->setTofIrEnabled(true);
        device->tofCamera()->registerCallback([](xv::DepthImage const & tof){
            if (tof.type == xv::DepthImage::Type::IR) {
                static FpsCount fc;
                fc.tic();
                static int k = 0;
                if(k++%15==0){
                    std::cout << "tof IR   " << timeShowStr(tof.edgeTimestampUs, tof.hostTimestamp)
                              << tof.width << "x" << tof.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
        });
#endif
        bool ret = device->tofCamera()->setLibWorkMode(static_cast<xv::TofCamera::SonyTofLibMode>(enableDevMap["tof_mode"]));
        if(!ret)
        {
            std::cout<<"setLibWorkMode failed"<<std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if(enableDevMap["tof_point_cloud"])
        {
            ofs.open("./tof_pointcloud.txt",std::ios::out);
        }

        device->tofCamera()->registerCallback([&](xv::DepthImage const & tof){
            if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32) {
                static FpsCount fc;
                fc.tic();
                static int k = 0;
                if(enableDevMap["tof_point_cloud"])
                {
                    auto points = device->tofCamera()->depthImageToPointCloud(tof)->points;
                    char buff[128]={0};
                    for (auto iter = points.begin(); iter != points.end();iter++)
                    {
                        auto point = *iter;
                        sprintf(buff, "x=%f ,y=%f ,z=%f\n", point[0], point[1], point[2]);
                        ofs << buff;
                    }
                    ofs.flush();
                }
                if(k++%15==0){
                    if(enableDevMap["log"])
                    {
                        std::cout << "tof      " << timeShowStr(tof.edgeTimestampUs, tof.hostTimestamp)
                                << tof.width << "x" << tof.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                    }
                }
            }
            else if(tof.type == xv::DepthImage::Type::IR && enableDevMap["ir"])
            {
                static FpsCount fc;
                fc.tic();
                static int k = 0;
                if(k++%15==0){
                    if(enableDevMap["log"])
                    {
                        std::cout << "tof IR      " << timeShowStr(tof.edgeTimestampUs, tof.hostTimestamp)
                                << tof.width << "x" << tof.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                    }
                }
            }
        });
        xv::TofCamera::Manufacturer manufacturer = device->tofCamera()->getManufacturer();
        if(enableDevMap["ir"] && manufacturer == xv::TofCamera::Manufacturer::Pmd)
        {
            std::vector<unsigned char> result(63);
            bool bOK = device->hidWriteAndRead({0x02,0x10,0xf5,0x02,0x01}, result);
            if(bOK)
                std::cout << "Enable IR successfully" << std::endl;
            else
                std::cout << "Enable IR failed" << std::endl;
        } 
        device->tofCamera()->start();

        device->tofCamera()->registerColorDepthImageCallback([](const xv::DepthColorImage& depthColor){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%15==0){
                if(enableDevMap["log"])
                {
                    std::cout << "RGBD     " << timeShowStr(depthColor.hostTimestamp)
                            << depthColor.width << "x" << depthColor.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
        });
    }

    if (enableDevMap["imu"]) {
        device->imuSensor()->registerCallback([](xv::Imu const & imu){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%500==0){
                if(enableDevMap["log"])
                {
                    std::cout << "imu      " << timeShowStr(imu.edgeTimestampUs, imu.hostTimestamp) << "@" << std::round(fc.fps()) << "fps" << " Accel(" << imu.accel[0] << "," << imu.accel[1] << "," << imu.accel[2] << "), Gyro(" << imu.gyro[0] << "," << imu.gyro[1] << "," << imu.gyro[2] << ")" << std::endl;
                }
            }
        });
    }

    if (device->eventStream()) {
        device->eventStream()->registerCallback( [](xv::Event const & event){
            if(enableDevMap["log"])
            {
                std::cout << "event      " << timeShowStr(event.edgeTimestampUs, event.hostTimestamp)
                        << " (" << event.type << "," << event.state << ")" << std::endl;
            }
        });
        device->eventStream()->start();
    }
    if(enableDevMap["sgbm"])
    {
        device->sgbmCamera()->registerCallback([](const xv::SgbmImage& sgbm_image){
            if(sgbm_image.type == xv::SgbmImage::Type::Depth)
            {
                static int k=0;
                if(k++%50==0){
                    if(enableDevMap["log"])
                    {
                        std::cout<<"sgbm: "<<sgbm_image.width<<"*"<<sgbm_image.height<<std::endl;
                    }
                }
            }
        });
        device->sgbmCamera()->start(global_config);
    }

#ifdef USE_EX
    if (enableDevMap["slam_edge"])
    {
        if (std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()) {
            std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->registerCallback( [](const xv::Pose& pose){
                static FpsCount fc;
                fc.tic();
                static int k = 0;
                if(k++%500==0){
                    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                    if(enableDevMap["log"])
                    {
                        std::cout << "edge-pose" << timeShowStr(pose.edgeTimestampUs(), pose.hostTimestamp()) << "@" << std::round(fc.fps()) << "fps" << " (" << pose.x() << "," << pose.y() << "," << pose.z() << ") (" << pitchYawRoll[0]*180/M_PI << "," << pitchYawRoll[1]*180/M_PI << "," << pitchYawRoll[2]*180/M_PI << ")" << pose.confidence() << std::endl;
                    }
                }
            });
            std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->start(xv::Slam::Mode::Edge);
        } else {
            std::cout << "No edge in camera.\n";
        }
    }
#endif

    std::string tagDetectorId;
    if (enableDevMap["fisheye"]) {
#ifdef USE_EX
        std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->registerKeyPointsCallback([](const xv::FisheyeKeyPoints<2,32>& keypoints){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%50==0){
                if(enableDevMap["log"])
                {
                    std::cout << "keypoints  "  << timeShowStr(keypoints.edgeTimestampUs, keypoints.hostTimestamp) << keypoints.descriptors[0].size << ":" << keypoints.descriptors[1].size << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
        });
#endif
        device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const & stereo){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%50==0){
                if(enableDevMap["log"])
                {
                    std::cout << "stereo   "  << timeShowStr(stereo.edgeTimestampUs, stereo.hostTimestamp) << stereo.images[0].width << "x" << stereo.images[0].height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
        });
#ifdef USE_EX
        tagDetectorId = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->startTagDetector(device->slam(),  "36h11", 0.0639, 50.);

        if(enableDevMap["VGA"])
        {
            std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::MEDIUM);
        }
        if(enableDevMap["720P"])
        {
            std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::HIGH);
        }
        // device->fisheyeCameras()->setStereoResolutionMode(xv::ResolutionMode::R_720P);
        device->fisheyeCameras()->start();
    }

#endif

    if (enableDevMap["slam"]) {
        device->slam()->registerCallback([](const xv::Pose& pose){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%500==0){
                auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                if(enableDevMap["log"])
                {
                    std::cout << "slam-pose" << timeShowStr(pose.edgeTimestampUs(), pose.hostTimestamp()) << "@" << std::round(fc.fps()) << "fps" << " (" << pose.x() << "," << pose.y() << "," << pose.z() << "," << pitchYawRoll[0]*180/M_PI << "," << pitchYawRoll[1]*180/M_PI << "," << pitchYawRoll[2]*180/M_PI << ")" << pose.confidence() << std::endl;
                }
            }
        });

        device->slam()->registerStereoPlanesCallback([] (std::shared_ptr<const std::vector<xv::Plane>> planes) {
            if (!planes) return;
            static int k = 0;
            if(k++%30==0){
                if(enableDevMap["log"])
                {
                    std::cout << "Stereo-planes update (#" << planes->size() << " planes" << std::endl;
                }
            }
        });

        device->slam()->start();
    }

    if (enableDevMap["eyetracking"]) {
        device->eyetracking()->registerCallback([] (xv::EyetrackingImage const & eyetracking) {
            static FpsCount fc;
            fc.tic();
            static int k=0;
            if(k++%30==0){
                if(enableDevMap["log"])
                {
                    std::cout << "eyetracking  " << eyetracking.images[0].width << "x" << eyetracking.images[0].height << "@" << std::round(fc.fps()) << "fps"
                        << std::endl;
                }
            }
        });
    }

#ifdef USE_EX
    if (enableDevMap["slam_edge"])
    {
        // Must set device to edge mode to make both edge and mixed slam work.
        std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->start(xv::Slam::Mode::Edge);
    }
#endif


    std::cout << " == Initialized ==" << std::endl;

#ifdef USE_OPENCV
    //Display in thread to not slow down callbacks

    if (device->colorCamera()) {
        device->colorCamera()->registerCallback( [&device](xv::ColorImage const & im){
#ifdef USE_EX

        auto rgb = im.toRgb();
        std::shared_ptr<std::uint8_t> data(new std::uint8_t[rgb.width*rgb.height], std::default_delete<std::uint8_t[]>());
        for (std::size_t i=0; i < rgb.width*rgb.height; ++i) {
            data.get()[i] = 0.299*rgb.data.get()[3*i]+0.587*rgb.data.get()[3*i+1]+0.114*rgb.data.get()[3*i+2];
        }
        xv::GrayScaleImage img;
        img.width = rgb.width;
        img.height = rgb.height;
        img.data = data;
        //xv::AprilTagDetector rgbDetector("36h11");
        auto t0 = std::chrono::steady_clock::now();
        s_mtx_rgb_tags.lock();
        //s_rgb_tags = rgbDetector.detect(img);
        s_rgb_gray = img;
        s_mtx_rgb_tags.unlock();
        auto t1 = std::chrono::steady_clock::now();
        static auto tLast = t0 - std::chrono::milliseconds(500);
        if (t0 >= tLast+std::chrono::milliseconds(500)) {
            tLast = t0;
            if(enableDevMap["log"])
            {
                std::cout << "RGB tag detection: " << s_rgb_tags.size() << " in " << std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count()*1e-3 << " ms" << std::endl;
            }
        }
#endif
        s_mtx_rgb.lock();
        s_rgb = std::make_shared<xv::ColorImage>(im);
        s_mtx_rgb.unlock();
        });
    }
    if (enableDevMap["fisheye"]) {
        device->fisheyeCameras()->registerCallback( [&device](xv::FisheyeImages const & stereo){
        s_mtx_stereo.lock();
        s_stereo = std::make_shared<xv::FisheyeImages>(stereo);
        s_mtx_stereo.unlock();
#ifdef USE_EX
        s_mtx_tags.lock();
        auto tags = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->detectTags(stereo.images[0], "36h11");
        s_tags = std::make_shared<std::vector<std::pair<int,std::array<xv::Vector2d,4>>>>(tags);
        s_mtx_tags.unlock();
#endif
        });
#ifdef USE_EX
        std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->registerKeyPointsCallback([](const xv::FisheyeKeyPoints<2,32>& keypoints){
        s_mtx_stereo.lock();
        s_keypoints = std::make_shared<xv::FisheyeKeyPoints<2,32>>(keypoints);
        s_mtx_stereo.unlock();
        });
        std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->registerKeyPointsCallback([](const xv::FisheyeKeyPoints<4,32>& keypoints){
        s_mtx_stereo.lock();
        s_keypoints4cam = std::make_shared<xv::FisheyeKeyPoints<4,32>>(keypoints);
        s_mtx_stereo.unlock();
        });
#endif
    }
    if (enableDevMap["tof"]) {
        device->tofCamera()->registerCallback([](xv::DepthImage const & tof){
            if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32) {
                std::lock_guard<std::mutex> l(s_mtx_tof);
                s_tof = std::make_shared<xv::DepthImage>(tof);
            } else if (tof.type ==  xv::DepthImage::Type::IR) {
                auto ir = std::make_shared<xv::GrayScaleImage>();
                ir->width = tof.width;
                ir->height = tof.height;
                ir->data = tof.data;
                std::lock_guard<std::mutex> l(s_mtx_ir);
                s_ir = ir;
            }
        });

        device->slam()->registerTofPlanesCallback([] (std::shared_ptr<const std::vector<xv::Plane>> planes) {
            static FpsCount fc;
            if (!planes) return;
            fc.tic();
            if(enableDevMap["log"])
            {
                std::cout << "ToF-planes update (#" << planes->size() << " planes" << std::endl;
            }
        });

        //std::dynamic_pointer_cast<xv::TofCameraEx>(device->tofCamera())->registerColorDepthImageCallback([](const xv::DepthColorImage& depthColor){
        device->tofCamera()->registerColorDepthImageCallback([](const xv::DepthColorImage& depthColor){
            s_mtx_depthColor.lock();
            s_depthColor = std::make_shared<xv::DepthColorImage>(depthColor);
            s_mtx_depthColor.unlock();
        });

    }
    if(enableDevMap["sgbm"])
    {
        device->sgbmCamera()->registerCallback([](const xv::SgbmImage& sgbm_image){
            if(sgbm_image.type == xv::SgbmImage::Type::Depth)
            {
                s_mtx_sgbm.lock();
                s_ptr_sgbm = std::make_shared<const xv::SgbmImage>(sgbm_image);
                s_mtx_sgbm.unlock();
            }
        });
        device->sgbmCamera()->start(global_config);
    }

    if (enableDevMap["eyetracking"]) {
        device->eyetracking()->registerCallback([] (xv::EyetrackingImage const & eyetracking) {
            s_mtx_eyetracking.lock();
            s_eyetracking = std::make_shared<xv::EyetrackingImage>(eyetracking);
            s_mtx_eyetracking.unlock();
        });
    }

    s_stop = false;
    std::thread t(display);

#else
#ifdef USE_EX
    s_stop = false;
    std::thread t([&device,&tagDetectorId](){
        while (!s_stop) {
            auto t0 = std::chrono::steady_clock::now();
            std::this_thread::sleep_until(t0 + std::chrono::milliseconds(25));
            if (!tagDetectorId.empty()) {
                auto detections = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->getTagDetections(tagDetectorId);
                if (!detections.empty())
                    std::cout << "Tag detections: ";
                for (auto const& d : detections) {
                    auto const& pose = d.second;
                    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                    if(enableDevMap["log"])
                    {
                        std::cout << "id=" << d.first
                              << " (" << pose.x() << "," << pose.y() << "," << pose.z() << ","
                              << pitchYawRoll[0]*180/M_PI << "," << pitchYawRoll[1]*180/M_PI << "," << pitchYawRoll[2]*180/M_PI << ") " << pose.confidence() << std::endl;
                    }
                }
            }

        }
    });
#endif
#endif

    std::cout << " ################## " << std::endl;
    std::cout << "        Start       " << std::endl;
    std::cout << " ################## " << std::endl;

#ifdef USE_EX
    if (!tagDetectorId.empty()) {
        std::cerr << "ENTER to stop tag detection" << std::endl;
        std::cin.get();
        std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->stopTagDetector(tagDetectorId);
    }
    std::string getkey;
    xv::FisheyeCamerasEx::ResolutionMode res = xv::FisheyeCamerasEx::ResolutionMode::MEDIUM;
    xv::SgbmCamera::Resolution sgbmres = xv::SgbmCamera::Resolution::SGBM_640x480;
    bool sgbm_ctl = true;
    while(true)
    {
        if(res == xv::FisheyeCamerasEx::ResolutionMode::MEDIUM)
            std::cerr << "ENTER 'f' to switch FE to HIGH res" << std::endl;
        else
            std::cerr << "ENTER 'f' to switch FE to MEDIUM res" << std::endl;

        if(sgbm_ctl == true){
            std::cerr << "ENTER 's' to stop SGBM to res" << std::endl;
            std::cerr << "ENTER 'r' to modify SGBM resolution" << std::endl;
        }
        else{
            std::cerr << "ENTER 's' to start SGBM to res" << std::endl;
        }
        
        std::cerr << "ENTER 'q' to exit" << std::endl;

        std::cin >> getkey;
        //getkey = std::cin.get();
        if(getkey == "f")
        {
            res = (res == xv::FisheyeCamerasEx::ResolutionMode::MEDIUM)?(xv::FisheyeCamerasEx::ResolutionMode::HIGH):(xv::FisheyeCamerasEx::ResolutionMode::MEDIUM);
            std::cout<<"device->fisheyeCameras())->setResolutionMode:"<<static_cast<int>(res)<<std::endl;
            std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(res);
        }
        else if(getkey == "s")
        {
            
            if(sgbm_ctl == false)
            {
                std::cout<<"device->sgbmCamera()->start(global_config)"<<static_cast<int>(res)<<std::endl;
                device->sgbmCamera()->start(global_config);
            }
            else
            {
                std::cout<<"device->sgbmCamera()->stop()"<<static_cast<int>(res)<<std::endl;
                device->sgbmCamera()->stop();
            }
            sgbm_ctl = !sgbm_ctl;
        }
        else if(getkey == "r")
        {
            sgbmres = (sgbmres == xv::SgbmCamera::Resolution::SGBM_640x480) ? xv::SgbmCamera::Resolution::SGBM_1280x720 : xv::SgbmCamera::Resolution::SGBM_640x480;
            bool ret = device->sgbmCamera()->setSgbmResolution(sgbmres);
            if(ret){
                std::cout << "device->sgbmCamera()->setSgbmResolution: " << static_cast<int>(res)<<std::endl;
            }else{
                std::cout << "device->sgbmCamera()->setSgbmResolution failed"<<std::endl;
            }
        }
        else if(getkey == "q")
        {
            break;
        }
     
    }
#endif

    // std::cerr << "ENTER to stop" << std::endl;
    // std::cin.get();

    s_stop = true;

    std::cout << " ################## " << std::endl;
    std::cout << "        Stop        " << std::endl;
    std::cout << " ################## " << std::endl;

#ifdef USE_EX
    if (enableDevMap["slam_edge"])
    {
        if (std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2())
            std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->stop();
    }
#endif

    if (device->slam())
        device->slam()->stop();


#ifdef USE_OPENCV
    s_stop = true;
    if (t.joinable()) {
        t.join();
    }
#endif
    ofs.close();
    return EXIT_SUCCESS;
}
catch( const std::exception &e){
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
