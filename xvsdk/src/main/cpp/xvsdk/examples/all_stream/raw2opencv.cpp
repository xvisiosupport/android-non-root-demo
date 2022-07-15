#include <opencv2/opencv.hpp>
#include <xv-sdk.h>

#include <cstring>
#include "colors.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//for Windows
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static bool colorize_disparity = true;
static bool colorize_depth = true;

static double depth_max_distance_m = 5;
static double depth_min_distance_m = 0.1;

static bool stretch_disparity = true;

cv::Mat raw_to_opencv(std::shared_ptr<const xv::ColorImage> rgb)
{
    cv::Mat img;
    switch (rgb->codec) {
    case xv::ColorImage::Codec::YUV420p: {
        img = cv::Mat::zeros(rgb->height, rgb->width, CV_8UC3);
        auto raw = rgb->data.get();
        auto rawImg = cv::Mat(rgb->height * 3 / 2, rgb->width, CV_8UC1, const_cast<unsigned char*>(raw));
        cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_I420);
        break;
    }
    case xv::ColorImage::Codec::YUYV: {
        img = cv::Mat::zeros(rgb->height, rgb->width, CV_8UC3);
        auto raw = rgb->data.get();
        auto rawImg = cv::Mat(rgb->height, rgb->width, CV_8UC2, const_cast<unsigned char*>(raw));
        cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_YUYV);
        break;
    }
    case xv::ColorImage::Codec::JPEG: {
        cv::Mat raw(1, rgb->width*rgb->height, CV_8UC1, const_cast<unsigned char*>(rgb->data.get()));
        img = cv::imdecode(raw, cv::IMREAD_COLOR);
        break;
    }
    }
    return img;
}

cv::Mat raw_to_opencv_tof_ir(xv::GrayScaleImage const& tof_ir) {
    cv::Mat out;
    out = cv::Mat::zeros(tof_ir.height, tof_ir.width, CV_8UC3);
    float dmax = 2191/4;
    auto tmp_d = reinterpret_cast<short const*>(tof_ir.data.get());
    for (unsigned int i=0; i< tof_ir.height*tof_ir.width; i++) {
        short d = tmp_d[i];
        unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
        if( u < 15 )
            u = 0;
        const auto &cc = colors.at(u);
        out.at<cv::Vec3b>( i/tof_ir.width, i%tof_ir.width ) = cv::Vec3b(cc.at(2), cc.at(1),cc.at(0) );
        /*short u = tof->ir.get()[i];
                cvtof.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3s(u*255/2191,u*255/2191,u*255/2191);*/
    }
    return out;
}

cv::Mat raw_to_opencv_tof_ir_grey(xv::GrayScaleImage const& tof_ir) {
    cv::Mat out;
    out = cv::Mat::zeros(tof_ir.height, tof_ir.width, CV_8UC1);
    std::memcpy(out.data, (short*)tof_ir.data.get(), static_cast<size_t>(out.rows*out.cols));
    cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
    return out;
}


cv::Mat raw_to_opencv( std::shared_ptr<const xv::DepthImage> tof)
{
    cv::Mat out;
    if (tof->height>0 && tof->width>0) {
        out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
        if (tof->type == xv::DepthImage::Type::Depth_32) {
            float dmax = 7.5;
            const auto tmp_d = reinterpret_cast<float const*>(tof->data.get());
            for (unsigned int i=0; i< tof->height*tof->width; i++) {
                const auto &d = tmp_d[i];
                if( d < 0.01 || d > 9.9 ) {
                    out.at<cv::Vec3b>(i / tof->width, i % tof->width) = 0;
                } else {
                    unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                    const auto &cc = colors.at(u);
                    out.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
                }
            }
        } else if (tof->type == xv::DepthImage::Type::Depth_16) {
            // #define DEPTH_RANGE_20M_SF       7494  //7.494m
            // #define DEPTH_RANGE_60M_SF       2494  //2.494m
            // #define DEPTH_RANGE_100M_SF      1498  //1.498m
            // #define DEPTH_RANGE_120M_SF      1249  //1.249m
            float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
            const auto tmp_d = reinterpret_cast<int16_t const*>(tof->data.get());
            for (unsigned int i=0; i< tof->height*tof->width; i++) {
                const auto &d = tmp_d[i];
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
            }
        } else if( tof->type == xv::DepthImage::Type::IR ){
            out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
            float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
            auto tmp_d = reinterpret_cast<unsigned short const*>(tof->data.get());
            for (unsigned int i=0; i< tof->height*tof->width; i++) {
                unsigned short d = tmp_d[i];
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                if( u < 15 )
                    u = 0;
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3b(cc.at(2), cc.at(1),cc.at(0) );
                /*short u = tof->ir.get()[i];
                cvtof.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3s(u*255/2191,u*255/2191,u*255/2191);*/
            }
        }
    }
    return out;
}


std::pair<cv::Mat,cv::Mat> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo)
{
    auto const& leftInput = stereo->images[0];
    auto const& rightInput = stereo->images[1];
    cv::Mat left = cv::Mat::zeros(leftInput.height, leftInput.width, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(rightInput.height, rightInput.width, CV_8UC1);

    if( leftInput.data != nullptr ){
        std::memcpy(left.data, leftInput.data.get(), static_cast<size_t>(left.rows*left.cols));
    }
    if( rightInput.data != nullptr ){
        std::memcpy(right.data, rightInput.data.get(), static_cast<size_t>(right.rows*right.cols));
    }

    cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

    return {left, right};
}

cv::Mat raw_to_opencv( std::shared_ptr<const xv::DepthColorImage> rgbd)
{
    cv::Mat out;
    out = cv::Mat::zeros(rgbd->height, rgbd->width*2, CV_8UC3);
    auto w = rgbd->width;

    if (rgbd->height>0 && rgbd->width>0) {
        float dmax = 7.5;
        const auto tmp_d = reinterpret_cast<std::uint8_t const*>(rgbd->data.get()+3);
        for (unsigned int i=0; i< rgbd->height*rgbd->width; i++) {
            const auto &d = *reinterpret_cast<float const*>(tmp_d + i*(3+sizeof(float)));
            if( d < 0.01 || d > 9.9 ) {
                out.at<cv::Vec3b>(i / w, i % rgbd->width) = 0;
            } else {
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>( i/ w, i%rgbd->width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
            }
        }
        const auto tmp_rgb = reinterpret_cast<std::uint8_t const*>(rgbd->data.get());
        for (unsigned int i=0; i< rgbd->height*rgbd->width; i++) {
            const auto rgb = reinterpret_cast<std::uint8_t const*>(tmp_rgb + i*(3+sizeof(float)));
            out.at<cv::Vec3b>( i/ w, (i%rgbd->width) + rgbd->width) = cv::Vec3b(rgb[2], rgb[1], rgb[0]);
        }
    }
    return out;
}

static std::tuple<int, int, int> color(double distance, double distance_min, double distance_max, double threshold)
{
    double d = std::max(distance_min, std::min(distance, distance_max));
    d = (d - distance_min) / (distance_max - distance_min);
    // std::cout<<"color max"<<distance_max<<"color min"<<distance_min<<"color"<<distance<<std::endl;
    if (distance <= threshold || distance > distance_max)
    {
        return std::tuple<int, int, int>(0, 0, 0);
    }
    int b = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.5))), 1.0));
    int g = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.25))), 1.0));
    int r = static_cast<int>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * d)), 1.0));
    return std::tuple<int, int, int>(r, g, b);
}

static std::shared_ptr<unsigned char> depthImage(uint16_t *data, unsigned int width, unsigned int height, double min_distance_m, double max_distance_m, bool colorize)
{
    std::shared_ptr<unsigned char> out;
    if (colorize)
    {
        out = std::shared_ptr<unsigned char>(new unsigned char[width * height * 3], std::default_delete<unsigned char[]>());
    }
    else
    {
        out = std::shared_ptr<unsigned char>(new unsigned char[width * height], std::default_delete<unsigned char[]>());
    }

    for (unsigned int i = 0; i < width * height; i++)
    {
        double distance_mm = data[i];
        if (colorize)
        {
            double distance_m = distance_mm / 1000.;

            auto c = color(distance_m, min_distance_m, max_distance_m, min_distance_m);
            out.get()[i * 3 + 0] = static_cast<unsigned char>(std::get<2>(c));
            out.get()[i * 3 + 1] = static_cast<unsigned char>(std::get<1>(c));
            out.get()[i * 3 + 2] = static_cast<unsigned char>(std::get<0>(c));
        }
        else
        {
            double max_distance_mm = max_distance_m * 1000.;
            double min_distance_mm = min_distance_m * 1000.;
            distance_mm = std::min(max_distance_mm, distance_mm);
            distance_mm = std::max(distance_mm, min_distance_mm);

            double norm = (distance_mm - min_distance_mm) / (max_distance_mm - min_distance_mm);
            auto c = 255. * norm;
            out.get()[i] = static_cast<unsigned char>(c);
        }
    }

    return out;
}
cv::Mat convDepthToMat(std::shared_ptr<const xv::SgbmImage> sgbm_image,bool _colorize_depth)
{
    uint16_t* p16 = (uint16_t*)sgbm_image->data.get();

    // cv::Mat mask;
    // cv::Mat im_gray_d = cv::Mat(cv::Size(sgbm_image->width, sgbm_image->height),  CV_16UC1, p16); //18
    // cv::inRange(im_gray_d, cv::Scalar(1), cv::Scalar(65535), mask);
    // p16 = (uint16_t *)im_gray_d.data;

    double focal_length = sgbm_image->width / (2.f * tan(/*global_config.fov*/69 / 2 / 180.f * M_PI));
    double max_distance_m = (focal_length * /*global_config.baseline*/0.11285 / 1);
    double min_distance_m = 0; //0 is considered invalid distance (0 disparity == unknown)
    max_distance_m = std::min(max_distance_m, depth_max_distance_m);
    min_distance_m = depth_min_distance_m;
    assert(max_distance_m > min_distance_m);

    static std::shared_ptr<unsigned char> tmp;
    tmp = depthImage(p16, sgbm_image->width, sgbm_image->height, min_distance_m, max_distance_m, !!_colorize_depth);
    if (_colorize_depth)
    {
        cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3, tmp.get());
        // cv::Mat roi = cv::Mat::zeros(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3);
        // im_col.copyTo(roi,mask);
        return im_col;
    }
    else
    {
        cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC1, tmp.get());
        return im_col;
    }
}

static void stretchDisparityRange(cv::Mat &frame, int disp)
{
    const int scaleFactorLowerD = 2;
    const int scaleFactorUpperD = 1;

    int halfIntervalD = disp / 2;

    for (int row = 0; row < frame.rows; row++)
        for (int col = 0; col < frame.step; col++)
        {
            // if disparity value = [0, D/2), multiply it with 4
            if (frame.data[col + row * frame.step] < halfIntervalD)
                frame.data[col + row * frame.step] *= scaleFactorLowerD;
            else
                // if disparity value = [D/2, D), multiply it with 2
                //frame.data[col + row * frame.step] *= scaleFactorUpperD;
                frame.data[col + row * frame.step] =
                    frame.data[col + row * frame.step] * scaleFactorUpperD +
                    scaleFactorLowerD * halfIntervalD;
        }
}

cv::Mat convdispToMat(std::shared_ptr<const xv::SgbmImage> sbgm_image, bool col_map)
{
    cv::Mat im_gray;

    im_gray = cv::Mat(cv::Size(sbgm_image->width, sbgm_image->height), CV_8UC1, const_cast<uint8_t*>(sbgm_image->data.get()));

    if (stretch_disparity)
    {
        //for better visualization
        stretchDisparityRange(im_gray, 96);
    }

    if (col_map)
    {
        cv::Mat im_col(cv::Size(sbgm_image->width, sbgm_image->height), CV_8UC3);
        applyColorMap(im_gray, im_col, cv::COLORMAP_JET);
        return im_col;
    }
    else
    {
        return im_gray;
    }
}

cv::Mat raw_to_opencv(std::shared_ptr<const xv::SgbmImage> sbgm_image)
{
    if(xv::SgbmImage::Type::Depth == sbgm_image->type)
    {
        cv::Mat mat = convDepthToMat(sbgm_image,colorize_depth);
        // cvtColor(mat, mat, cv::COLOR_GRAY2RGB);
        // cv::Mat mask;
        // cv::medianBlur(mat, mat, 5);
        // cv::inRange(mat, cv::Scalar(1), cv::Scalar(255), mask);
        return mat;
        
    }
    else if(xv::SgbmImage::Type::Disparity == sbgm_image->type)
    {
        return convdispToMat(sbgm_image,colorize_disparity);
    }
    return cv::Mat();
}
