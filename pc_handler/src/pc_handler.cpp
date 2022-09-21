#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include<Eigen/Dense>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
using std::placeholders::_1;
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

rclcpp::Node::SharedPtr node = nullptr;
class PCHandler : public rclcpp::Node
{
    public:

        sensor_msgs::msg::PointCloud2 m_cloud;
        sensor_msgs::msg::PointCloud2 m_cropped_cloud;
        geometry_msgs::msg::PointStamped m_bottle_center;
        Eigen::Vector4f m_centroid;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_br;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        PCHandler(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "%snode is success run.",name.c_str());
           // m_pointcloud_sub=this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points",10,std::bind(&PCHandler::getPC,this,std::placeholders::_1));
            m_center_sub=this->create_subscription<geometry_msgs::msg::PointStamped>("/point_in_3d",10,std::bind(&PCHandler::getCenter,this,std::placeholders::_1));
            m_goalpose_pub=this->create_publisher<geometry_msgs::msg::PoseStamped>("/bottle_detection/goalpose", 1);

            m_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_sub ;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_center_sub ;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goalpose_pub;

    public:
    void getPC(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        m_cloud= *msg;
        RCLCPP_INFO(this->get_logger(),"successfully get PointCloud");
        geometry_msgs::msg::TransformStamped transform;
        sensor_msgs::msg::PointCloud2  w_cloud;

        transform = tf_buffer_->lookupTransform(
           "base_link", m_cloud.header.frame_id,
            tf2::TimePointZero);
        
        tf2::doTransform(m_cloud, w_cloud, transform);
        std::cout << " getPC called" << std::endl;    

//"camera_depth_optical_frame"
    }

    public:
    void getCenter( geometry_msgs::msg::PointStamped::SharedPtr msg)
    {        
        m_bottle_center = *msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(m_cloud, *pointcloud_ptr);
        RCLCPP_INFO(this->get_logger(),"size of original PointCloud: %lu", pointcloud_ptr->size());
        ////define a 3d volume around bottle
        float d = 0.1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y-d, m_bottle_center.point.z+0.3));
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y-d, m_bottle_center.point.z+0.3));
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y+d, m_bottle_center.point.z+0.3));
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y+d, m_bottle_center.point.z+0.3));
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y-d, m_bottle_center.point.z-0.1));
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y-d, m_bottle_center.point.z-0.1));
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y+d, m_bottle_center.point.z-0.1));
        boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y+d, m_bottle_center.point.z-0.1)); 
       
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(boundingbox_ptr);
        hull.setDimension(3);
        std::vector<pcl::Vertices> polygons;
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
        hull.reconstruct(*surface_hull, polygons);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropHull<pcl::PointXYZ> bb_filter;
        bb_filter.setDim(3);
        bb_filter.setInputCloud(pointcloud_ptr);
        bb_filter.setHullIndices(polygons);
        bb_filter.setHullCloud(surface_hull);
        bb_filter.filter(*objects);
        
        RCLCPP_INFO(this->get_logger(),"1size of cap cloud: %lu", objects->size());

        //test, get the maximun and minimun of a pointcloud
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*objects, minPt, maxPt);

        //filter out points which belongs to the bottle cap
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(objects);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(maxPt.z-0.005, maxPt.z);
        pass.filter(*objects);

        RCLCPP_INFO(this->get_logger(),"size of cap cloud: %lu", objects->size());

        //calculate centroid of bottle cap
        if(objects->size()>300)
        {
            pcl::compute3DCentroid(*objects, m_centroid);
            RCLCPP_INFO(this->get_logger(),"center of the point cloud: %f, %f, %f", m_centroid(0), m_centroid(1), m_centroid(2));
        }

        tf2::Quaternion q;
        q.setRPY(-3.1415926/2, 0, 0);

        if(m_centroid(2)>=0.20 && m_centroid(2)<=0.40){
            geometry_msgs::msg::PoseStamped goalpose;

            goalpose.pose.orientation.x = q.x();
            goalpose.pose.orientation.y = q.y();
            goalpose.pose.orientation.z = q.z();
            goalpose.pose.orientation.w = q.w();
            goalpose.pose.position.x = m_centroid[0];
            goalpose.pose.position.y = m_centroid[1];
            goalpose.pose.position.z = 0.4;
            goalpose.header.frame_id = "base_link";
            goalpose.header.stamp = node->now();

            m_goalpose_pub->publish(goalpose);
            RCLCPP_INFO(this->get_logger(),"goal pose is published!");

            //Publish transform
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp =  node->now();
            transform.header.frame_id = "base_link";
            transform.child_frame_id = "goal_position_0";
            transform.transform.translation.x = goalpose.pose.position.x;
            transform.transform.translation.y = goalpose.pose.position.y;
            transform.transform.translation.z = goalpose.pose.position.z;
            transform.transform.rotation.x = goalpose.pose.orientation.x;
            transform.transform.rotation.y = goalpose.pose.orientation.y;
            transform.transform.rotation.z = goalpose.pose.orientation.z;
            transform.transform.rotation.w = goalpose.pose.orientation.w;
            m_br->sendTransform(transform);
        }
        std::cout << " getCenter called" << std::endl;    

    }
   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<PCHandler>("PCHandler");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}