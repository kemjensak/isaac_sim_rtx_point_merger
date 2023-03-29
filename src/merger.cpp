#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

std::string output_type;
std::string lidar_topic = "/point_cloud"; 

const uint32_t sizeof_full_scan = 32768;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPC;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPC;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_new(new pcl::PointCloud<pcl::PointXYZ>());

template<typename T>
void publish_points(T &new_pc, const sensor_msgs::msg::PointCloud2 &old_msg) {
   
    new_pc->is_dense = true;
    sensor_msgs::msg::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg.header;
    pc_new_msg.header.frame_id = "velodyne";
    pc_new_msg.header.stamp = old_msg.header.stamp;
    pubPC->publish(pc_new_msg);
}

void lidar_handle(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg_pointer) {
    sensor_msgs::msg::PointCloud2 pc_msg;
    pc_msg = *pc_msg_pointer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::fromROSMsg(pc_msg, *pc);

    for (uint64_t point_id = 0; point_id < pc->points.size(); ++point_id) {

        pcl::PointXYZ new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        if (pc_new->points.size() >= sizeof_full_scan)
        {        
            publish_points(pc_new, pc_msg);
            pc_new->points.clear();
        }
        pc_new->points.push_back(new_point);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("converter");

    subPC = node->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, 10, lidar_handle);
    pubPC = node->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);
    RCLCPP_INFO(node->get_logger(), "Listening to lidar topic ......");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}