//
// Created by user on 23-11-1.
//

//
// Created by user on 23-10-30.
//


#include <atomic>
#include <condition_variable>
#include <iostream>

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

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

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();


    cloud->points.resize( 0);

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();
    pcl::PointXYZRGB point;
    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        point.x = Vertex[i].x;
        point.y = Vertex[i].y;
        point.z = Vertex[i].z;
        if(point.z<0.5 || point.z>10)
            continue;


        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        point.r = get<2>(RGB_Color); // Reference tuple<2>
        point.g = get<1>(RGB_Color); // Reference tuple<1>
        point.b = get<0>(RGB_Color); // Reference tuple<0>
        cloud->push_back(point);

    }

    return cloud; // PCL RGB Point Cloud generated
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_replay");
    ros::NodeHandle nh;

    rs2::threshold_filter thr_filter(0.15,10);   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

    // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);


    std::string bag_file = argv[1];
//    rosbag::Bag bag_out(bag_file, rosbag::bagmode::Write);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_device_from_file(bag_file);

    // auto pub_frame_cloud = nh.advertise<sensor_msgs::PointCloud2>("frame_cloud",100);
    auto pub_rgb = nh.advertise<sensor_msgs::Image>("rgb", 1);
    auto pub_depth = nh.advertise<sensor_msgs::Image>("depth", 1);

    int width_img, height_img;


    auto ref_camera = RS2_STREAM_COLOR;
    rs2::align align_to(ref_camera);

    auto pipe_profile = pipe.start(cfg);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;

    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = intrinsics_cam.fx;
    K.at<float>(1,1) = intrinsics_cam.fy;
    K.at<float>(0,2) = intrinsics_cam.ppx;
    K.at<float>(1,2) = intrinsics_cam.ppy;
    std::cout<<"K: "<<K<<std::endl;
    cv::Mat D = cv::Mat::zeros(1,5,CV_32F);
    memcpy(D.data,intrinsics_cam.coeffs,sizeof(float)*5);
    std::cout<<"D: "<<D<<std::endl;

    auto device = pipe.get_active_profile().get_device();
    rs2::playback playback = device.as<rs2::playback>();
    playback.set_real_time(true);

    auto duration = playback.get_duration();
    int progress = 0;

    rs2::frameset frameset;
    uint64_t posCurr = playback.get_position();
    
    int seq = 0;

    std::mutex mu;
    rs2::frame color_frame_copy;
    rs2::frame depth_frame_copy;
    std_msgs::Header header_copy;
    bool need_pub_cloud = false;
    std::thread pub_cloud_thread([&](){
        rs2::pointcloud pc;
        while(1){
            mu.lock();
            if(!need_pub_cloud){
                mu.unlock();
                std::this_thread::sleep_for(10ms);
                continue;
            }
            rs2::frame color_frame = std::move(color_frame_copy);
            rs2::frame depth_frame = std::move(depth_frame_copy);
            std_msgs::Header header = std::move(header_copy);
            need_pub_cloud = false;
            mu.unlock();

            pc.map_to(color_frame);
            auto points = pc.calculate(depth_frame);
            auto cloud = PCL_Conversion(points, color_frame);
            sensor_msgs::PointCloud2 PointsMsg;
            pcl::toROSMsg(*cloud, PointsMsg);
            PointsMsg.header = header;
            // pub_frame_cloud.publish(PointsMsg);
        }
    });
    pub_cloud_thread.detach();

    // try_wait_for_frames will keep repeating the last frame at the end of the file,
    // so we need to exit the look in some other way!
    while (pipe.try_wait_for_frames(&frameset, 1000) && ros::ok()) {
        int posP = static_cast<int>(posCurr * 100. / duration.count());
        if (posP > progress) {
            progress = posP;
        }

        std::cout<<posCurr * 100. / duration.count()<<std::endl;

        auto posNext = playback.get_position();


        if (posNext < posCurr)
            break;

        posCurr = posNext;

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

        cv::Mat im_undist,depth_undist;
        cv::undistort(im,im_undist,K,D,K);
        cv::undistort(depth,depth_undist,K,D,K);



        const auto msg_rostime = ros::Time(timestamp);
        std_msgs::Header header; // empty header
        header.seq = seq++; // user defined counter
        header.stamp = msg_rostime;
        header.frame_id = "camera";
//        header.seq = seq++;

        auto rgb_msg = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,im_undist).toImageMsg();
        auto depth_msg = cv_bridge::CvImage(header,sensor_msgs::image_encodings::TYPE_16UC1,depth_undist).toImageMsg();
        pub_rgb.publish(rgb_msg);
        pub_depth.publish(depth_msg);


//        mu.lock();
//        color_frame_copy = std::move(color_frame);
//        depth_frame_copy = std::move(depth_frame);
//        header_copy = std::move(header);
//        need_pub_cloud=true;
//        mu.unlock();
    }

    
    return 0;


}



