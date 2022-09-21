#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "move_interfaces/srv/robot_move.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"

using std::placeholders::_1;

class ArmMove : public rclcpp::Node
{

public:
  ArmMove(std::string name) : Node(name), find_enable(true)
  {
    RCLCPP_INFO(this->get_logger(), "%snode is success run.", name.c_str());
    m_bbx_sub = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, std::bind(&ArmMove::getBBX, this, _1));
    move_clinet = this->create_client<move_interfaces::srv::RobotMove>("/arm_move");
  }

private:
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr m_bbx_sub;
  rclcpp::Client<move_interfaces::srv::RobotMove>::SharedPtr move_clinet;
  darknet_ros_msgs::msg::BoundingBoxes *m_bbx_ptr;
  darknet_ros_msgs::msg::BoundingBoxes m_bbx;
  float center_x, center_y;
  float move_x, move_y;
  bool find_enable;

  void getBBX(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
  {
    if (find_enable == true)
    {
      find_enable = false;
      m_bbx = *msg;
      for (int i = 0; i < m_bbx.bounding_boxes.size(); i++)
      { // class
        if (m_bbx.bounding_boxes[i].class_id == "bottle")
        {
          center_x = (m_bbx.bounding_boxes[i].xmin + m_bbx.bounding_boxes[i].xmax) / 2.0;
          center_y = (m_bbx.bounding_boxes[i].ymin + m_bbx.bounding_boxes[i].ymax) / 2.0;
          break;
        }
        else
        {
          center_x=320;
          center_y=240;

        }
      }
      auto request = std::make_shared<move_interfaces::srv::RobotMove::Request>();
       if (center_x > 300 && center_x < 340 and center_y > 220 && center_y < 260)
      //if (center_x > 300 && center_x < 340)
      {
        move_x = 0.0;
        move_y = 0.0;
      }

      else
      {
        move_x = ((center_x - 320) / 10000)*2.25;
        move_y = (-(center_y - 240)/ 10000)*2.25;
        //move_y = 0.0;
      }

      request->x = move_x;
      request->y = move_y;
      move_clinet->async_send_request(request, std::bind(&ArmMove::arm_callback, this, _1));
    }
  }

  void arm_callback(rclcpp::Client<move_interfaces::srv::RobotMove>::SharedFuture response)
  {
    auto result = response.get();
    if (result->result == true)
    {
      find_enable = true;
      std::cout << center_x << std::endl;
      //std::cout << center_y << std::endl;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmMove>("arm_move");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}