#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "move_interfaces/srv/robot_move.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;
using std::placeholders::_2;

//创建一个类节点，名字叫做MoveService,继承自Node.Create a class node named MoveService, which inherits from Node.
class MoveService : public rclcpp::Node
{

public:
    MoveService();
    // 规划笛卡尔路径Planning Cartesian paths
    double plan_cartesian_path(double x, double y, double z);

    // 声明一个回调函数Declare a callback function
    void move_callback(const move_interfaces::srv::RobotMove::Request::SharedPtr request,
                       const move_interfaces::srv::RobotMove::Response::SharedPtr response);

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
    // 声明一个服务回调组Declare a service callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_organization;

    // 声明一个服务端Declare a server side
    rclcpp::Service<move_interfaces::srv::RobotMove>::SharedPtr Move_Server;

    geometry_msgs::msg::Pose start_pose;
};

MoveService::MoveService() : Node("MoveService", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ur_manipulator")
{

    RCLCPP_INFO(this->get_logger(), "Start arm_move service");
    // 实例化回调组, 作用为避免死锁Instantiate a callback group, to avoid deadlocks
    callback_group_organization = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // 实例化机械臂运动的的服务 Services for instantiating robotic arm movements
    Move_Server = this->create_service<move_interfaces::srv::RobotMove>("arm_move",
                                                                        std::bind(&MoveService::move_callback, this, _1, _2),
                                                                        rmw_qos_profile_services_default,
                                                                        callback_group_organization);
    // 设置机械臂运动的速度和加速度Set the velocity and acceleration of the robot arm movement
    this->move_group_.setMaxVelocityScalingFactor(0.2);
    this->move_group_.setMaxAccelerationScalingFactor(0.2);
}

// Cartesian Paths
// ^^^^^^^^^^^^^^^
// 通过指定末端执行器要经过的航点列表来直接规划笛卡尔路径

double MoveService::plan_cartesian_path(double x, double y, double z)
{
    // 获取机器人的当前位置Get the current position of the robot
    start_pose = this->move_group_.getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    // 设置目标点，并到航点列表中Set the target point and go to the waypoint list
    start_pose.position.x += x;
    start_pose.position.y += y;
    start_pose.position.z += z;
    waypoints.push_back(start_pose);

    // 尝试规划一条笛卡尔空间下的路径，依次通过所有路点Try to plan a path in Cartesian space, passing through all waypoints in turn
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = this->move_group_.computeCartesianPath(waypoints, // waypoints, 航点列表
                                                             eef_step,  // eef_step，终端步进值
                                                             jump_threshold,// jump_threshold，跳跃阈值
                                                             trajectory); // avoid_collisions，避障规划

    RCLCPP_INFO(this->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // 如果路径规划成功（覆盖率>80%）,则开始控制机械臂运动If the path planning is successful (coverage > 80%), start controlling the robot arm movement
    if (fraction > 0.8)
    {
        this->move_group_.execute(trajectory);
    }
    return fraction;
}

void MoveService::move_callback(const move_interfaces::srv::RobotMove::Request::SharedPtr request,
                                const move_interfaces::srv::RobotMove::Response::SharedPtr response)
{
    double result;
    result = plan_cartesian_path(request->x, request->y, request->z);
    if (result > 0.8)
    {
        response->result = true;
    }
    else
    {
        response->result = false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveService>();
    // 把节点的执行器变成多线程执行器, 避免死锁Turn node executors into multi-threaded executors to avoid deadlocks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
