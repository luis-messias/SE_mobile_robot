#ifndef MICRO_ROS_H
#define MICRO_ROS_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <freertos/FreeRTOS.h>
#include <RobotHandle.h>

class MicroROS {
public:

    MicroROS(RobotHandle* robot);
    ~MicroROS();
    bool initialize();
    void spinOnce(int timeout_ms = 100);
    void publishPose();
    void publishPIDStatus();
    void checkCmdVelTimeout(TickType_t timeout_ticks);

private:
    // ROS2 core components
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    // Publishers
    rcl_publisher_t pose_publisher;
    rcl_publisher_t rpm_publisher;
    rcl_publisher_t rpm_left_publisher;
    rcl_publisher_t rpm_right_publisher;
    rcl_publisher_t rpm_left_setpoint_publisher;
    rcl_publisher_t rpm_right_setpoint_publisher;
    rcl_publisher_t pid_left_output_publisher;
    rcl_publisher_t pid_right_output_publisher;
    geometry_msgs__msg__PoseStamped pose_msg;
    std_msgs__msg__Float32 rpm_left_msg;
    std_msgs__msg__Float32 rpm_right_msg;
    std_msgs__msg__Float32 rpm_left_setpoint_msg;
    std_msgs__msg__Float32 rpm_right_setpoint_msg;
    std_msgs__msg__Float32 pid_left_output_msg;
    std_msgs__msg__Float32 pid_right_output_msg;

    // Subscribers
    rcl_subscription_t cmd_vel_subscriber;
    rcl_subscription_t goal_pose_subscriber;
    geometry_msgs__msg__Twist cmd_vel_msg;
    geometry_msgs__msg__PoseStamped goal_pose_msg;

    // Robot reference
    RobotHandle* robot_handle;

    // Watchdog variables
    TickType_t last_cmd_vel_time;
    bool getting_cmd_vel_msg;
    bool initialized;

    // Callback functions (static to work with C API)
    static void cmdVelCallback(const void* msgin);
    static void goalPoseCallback(const void* msgin);

    // Static reference to instance for callbacks
    static MicroROS* instance;
    char frame_id_buffer[32];
};

#endif // MICRO_ROS_H
