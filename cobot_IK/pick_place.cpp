#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Tau = 1 full rotation in radians
const double tau = 2 * M_PI;

void close_gripper(moveit::planning_interface::MoveGroupInterface& move_gripper)
{
    move_gripper.setJointValueTarget("gripper_right_joint", 0.055);
    move_gripper.move();
}

void open_gripper(moveit::planning_interface::MoveGroupInterface& move_gripper)
{
    move_gripper.setJointValueTarget("gripper_right_joint", 0.0);
    move_gripper.move();
}

void move_to_target(moveit::planning_interface::MoveGroupInterface& move_group, double x, double y, double z)
{
    geometry_msgs::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(tau / 4, -tau / 4, 0); // Maintain orientation
    target_pose.orientation = tf2::toMsg(orientation);

    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    move_group.setPoseTarget(target_pose);
    move_group.move();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Validate input arguments
    if (argc < 4) {
        ROS_ERROR("Usage: cobot_pick_and_place <x> <y> <z>");
        return 1;
    }

    double x = std::stod(argv[1]); // Convert input arguments to double
    double y = std::stod(argv[2]);
    double z = std::stod(argv[3]);

    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface gripper("hand");
    arm.setPlanningTime(45.0);

    ROS_INFO("Moving to target position: x=%.3f, y=%.3f, z=%.3f", x, y, z);

    move_to_target(arm, x, y, z);
    // close_gripper(gripper);  // Simulate grasping
    // open_gripper(gripper)

    return 0;  // Exit once movement is complete
}
