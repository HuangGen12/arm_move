
#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
//#include "darknet_ros_msgs/msg/BoundingBoxes.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/trigger.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using std::placeholders::_1;

class Deprojector : public rclcpp::Node
{

public:
    
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_br;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;



    Deprojector(std::string name) : Node(name)
    {
    RCLCPP_INFO(this->get_logger(), "%snode is success run.",name.c_str());
    
    tf_broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    m_bbx_sub=this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>("/darknet_ros/bounding_boxes",1,std::bind(&Deprojector::getBBX,this,_1));
    m_intrinsic=this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/aligned_depth_to_color/camera_info",1,std::bind(&Deprojector::getIntrinsics,this,_1));
    m_depth_sub=this->create_subscription<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_raw",1,std::bind(&Deprojector::getDepth,this,_1));

   // m_depth_sub=this->create_subscription<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_raw",10,std::bind(&Deprojector::getDepth,this,_1));

    m_points_pub=this->create_publisher<geometry_msgs::msg::PointStamped>("point_in_3d", 1000);
    m_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    }

private:

    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr m_bbx_sub ;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_intrinsic;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depth_sub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_points_pub;
   // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_detect_trigger;


    sensor_msgs::msg::CameraInfo* m_intrinsics_ptr;
    sensor_msgs::msg::CameraInfo m_camera_info;
    sensor_msgs::msg::Image* m_depth_ptr;
    sensor_msgs::msg::Image m_depth;
    darknet_ros_msgs::msg::BoundingBoxes* m_bbx_ptr;
    darknet_ros_msgs::msg::BoundingBoxes m_bbx;
    geometry_msgs::msg::PointStamped m_point;
    geometry_msgs::msg::PointStamped m_point_out;
    //std::shared_ptr<tf2_ros::Buffer> m_tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped transform;

    float depth_at_left_pixel;
    float depth_at_right_pixel;
    float point_left[3];
    float point_right[3];
    float point_aver[3];
    float pixel_left[2];
    float pixel_right[2];
    bool bottle_detected= false;
    struct rs2_intrinsics m_rs_intrinsic;
    //tf2_ros::TransformListener* m_tf_listener = NULL;


void getDepth(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::cout << " getDepth called" << std::endl;
    if(bottle_detected==true){
      cv_bridge::CvImagePtr cv_ptr;
      m_depth = *msg;
      cv_ptr = cv_bridge::toCvCopy(msg);
      printf("pixel position: %d, %d \n", (int)pixel_left[0], (int)pixel_left[1]);
      depth_at_left_pixel = cv_ptr->image.at<uint16_t>(cv::Point((int)pixel_left[0], (int)pixel_left[1]));
      depth_at_right_pixel = cv_ptr->image.at<uint16_t>(cv::Point((int)pixel_right[0], (int)pixel_right[1]));
      printf("depth is %f \n", depth_at_left_pixel);
      rs2_deproject_pixel_to_point(point_left,&m_rs_intrinsic, pixel_left, depth_at_left_pixel);
      rs2_deproject_pixel_to_point(point_right,&m_rs_intrinsic, pixel_right, depth_at_right_pixel); 
      point_aver[0] = (point_left[0]+point_right[0])/2.0;
      point_aver[1] = (point_left[1]+point_right[1])/2.0;
      point_aver[2] = (point_left[2]+point_right[2])/2.0;
      printf("point in 3d space: %f, %f, %f \n", point_aver[0], point_aver[1], point_aver[2]);
      m_point.header.stamp = this->now();
      m_point.header.frame_id = "camera_depth_optical_frame";
      m_point.point.x = point_aver[0]/1000.0;
      m_point.point.y = point_aver[1]/1000.0;
      m_point.point.z = point_aver[2]/1000.0;

      
      //transform=tf_buffer_->lookupTransform("base_link", "camera_depth_optical_frame", tf2::TimePointZero);
      
      //m_tf_listener->lookupTransform("/base_link", "/camera_depth_optical_frame",m_point.header.stamp, tf2::durationFromSec(3.0));
      //printf("11111111111111133333333333111 \n");
      
      //tf2::doTransform(m_point, m_point_out,transform);
      //m_tf_listener->transform(m_point, m_point,"/base_link", tf2::durationFromSec(3.0));
      m_points_pub->publish(m_point);
      //printf("point in 3d space: %f", m_point_out);
      rclcpp::Time now = this->now();
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = now;
      t.header.frame_id = "/camera_depth_optical_frame";
      t.child_frame_id = "object";

      t.transform.translation.x = point_aver[0] / 1000.0;
      t.transform.translation.y = point_aver[1] / 1000.0;
      t.transform.translation.z = point_aver[2] / 1000.0;

      t.transform.rotation.x = 1.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 0.0;

      tf_broadcaster_->sendTransform(t);
      bottle_detected = false;
    }   
}

void getBBX(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
{
    std::cout << " getBBX called" << std::endl;    
    m_bbx = *msg;
 for (size_t i = 0; i < m_bbx.bounding_boxes.size(); i++)
  {//class
    if(m_bbx.bounding_boxes[i].class_id=="bottle"){
      std::cout << " class bottole called" << std::endl;   
      pixel_right[0] = (float)m_bbx.bounding_boxes[i].xmax;
      pixel_right[1] = (float)m_bbx.bounding_boxes[i].ymax;
      pixel_left[0] = (float)m_bbx.bounding_boxes[i].xmin;
      pixel_left[1] = (float)m_bbx.bounding_boxes[i].ymin;
      bottle_detected = true;
      // prevent the situation that only half bottle is detected
      //if((pixel_left[0]+pixel_right[0])/2<=500 && (pixel_left[1]+pixel_right[1])/2<=400){
      //std_srvs::srv::Trigger srv;
      //m_detect_trigger.call(srv);
      //}
      //break;
    }
  }
}




void getIntrinsics(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::cout << " getIntrinsics called" << std::endl;
    m_camera_info = *msg;
    m_rs_intrinsic.width = m_camera_info.width;
    m_rs_intrinsic.height = m_camera_info.height;
    m_rs_intrinsic.ppx = m_camera_info.k[2];
    m_rs_intrinsic.ppy = m_camera_info.k[5];
    m_rs_intrinsic.fx = m_camera_info.k[0];
    m_rs_intrinsic.fy = m_camera_info.k[4];
    m_rs_intrinsic.model = RS2_DISTORTION_NONE;
       

}

   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Deprojector>("rs_node_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}