//
// Created by user on 23-11-1.
//


#include <atomic>
#include <condition_variable>
#include <iostream>

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <mutex>
#include <thread>

using namespace std;



int main(int argc, char **argv) {
    ros::init(argc, argv, "live");
    ros::NodeHandle nh;

    rs2::threshold_filter thr_filter(0.15,10);   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

    // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);


    auto pub_rgb = nh.advertise<sensor_msgs::Image>("rgb", 1);
    auto pub_depth = nh.advertise<sensor_msgs::Image>("depth", 1);

    int width_img, height_img;


    auto ref_camera = RS2_STREAM_COLOR;
    rs2::align align_to(ref_camera);

    // Declare RealSense pipeline, encapsulating the actual device and sensors



    rs2::frameset frameset;
    
    int seq = 0;


    // try_wait_for_frames will keep repeating the last frame at the end of the file,
    // so we need to exit the look in some other way!
    rs2::pipeline pipe;
    rs2::config cfg;

    // RGB stream
//    cfg.enable_stream(RS2_STREAM_COLOR,1280,720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8, 30);

    // Depth stream
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 30);

    auto pipe_profile = pipe.start(cfg);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " <<
              intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;
    while (ros::ok())
    {
        // Block program until frames arrive
        frameset = pipe.wait_for_frames();
        double timestamp = frameset.get_timestamp() * 1e-3;

        auto processed = align_to.process(frameset);

        // Trying to get both other and aligned depth frames
        auto color_frame = processed.get_color_frame();
        auto depth_frame = processed.get_depth_frame();

        depth_frame = thr_filter.process(depth_frame);
        width_img = depth_frame.get_width();
        height_img = depth_frame.get_height();

        auto im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void *) (color_frame.get_data()),
                          cv::Mat::AUTO_STEP);
        auto depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);



        const auto msg_rostime = ros::Time(timestamp);
        std_msgs::Header header; // empty header
        header.seq = seq++; // user defined counter
        header.stamp = msg_rostime;
        header.frame_id = "camera";
//        header.seq = seq++;

        auto rgb_msg = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,im).toImageMsg();
        auto depth_msg = cv_bridge::CvImage(header,sensor_msgs::image_encodings::TYPE_16UC1,depth).toImageMsg();
        pub_rgb.publish(rgb_msg);
        pub_depth.publish(depth_msg);
    }

    
    return 0;


}



