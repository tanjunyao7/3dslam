#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <nav_msgs/OccupancyGrid.h>

class PointCloudToOccupancyGrid {
public:
    PointCloudToOccupancyGrid() : nh("~") {
        // Set up the ROS publisher and subscriber
        cloud_subscriber = nh.subscribe("/aft_pgo_map", 1, &PointCloudToOccupancyGrid::pointCloudCallback, this);
        occupancy_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);

        // Define the resolution of the occupancy grid (adjust according to your needs)
        resolution_xy = 0.05; // meters
        resolution_z = 0.02;
        // Set the height threshold to filter out ground and ceiling points
        height_min = -0.2; // meters
        height_max = 1.0;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // Convert ROS point cloud message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);


        // Create and publish the occupancy grid
        nav_msgs::OccupancyGrid occupancy_grid;
        convertPointCloudToOccupancyGrid(cloud, occupancy_grid);
        occupancy_grid_publisher.publish(occupancy_grid);
    }

    void convertPointCloudToOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::OccupancyGrid &occupancyGrid) {
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloud, minPoint, maxPoint);

        int width = std::ceil((maxPoint.x - minPoint.x) / resolution_xy);
        int height = std::ceil((maxPoint.y - minPoint.y) / resolution_xy);

        occupancyGrid.info.width = width;
        occupancyGrid.info.height = height;
        occupancyGrid.info.resolution = resolution_xy;
        occupancyGrid.info.origin.position.x = minPoint.x;
        occupancyGrid.info.origin.position.y = minPoint.y;

        // Initialize data vector with unknown values (-1)
        occupancyGrid.data.assign(width * height, 0);

        std::map<int,std::set<int>> voxels;

        for (const auto &point : cloud->points) {
            if(point.z<height_min || point.z>height_max)
                continue;
            int x = std::floor((point.x - minPoint.x) / resolution_xy);
            int y = std::floor((point.y - minPoint.y) / resolution_xy);
            int z = std::floor((point.z - minPoint.z) / resolution_z);
            int index = y * width + x;
            if(voxels.count(index))
                voxels.at(index).insert(z);
            else
                voxels.insert({index,{z}});
        }
        
        for(const auto& v:voxels){
            int index = v.first;
//            int occ = v.second.second-v.second.first>2.0;
            int8_t occ = 1;
            occupancyGrid.data[index] = occ;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_subscriber;
    ros::Publisher occupancy_grid_publisher;
    double resolution_xy;
    double resolution_z;
    double height_min,height_max;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_occupancy_grid_node");

    PointCloudToOccupancyGrid node;

    ros::spin();

    return 0;
}
