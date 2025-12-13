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

/**
 * @brief Micro-ROS interface for robot control and monitoring
 *
 * This class provides ROS2 communication capabilities for the mobile robot,
 * including velocity command subscription, odometry publishing, and PID
 * status monitoring. It uses micro-ROS for embedded ROS2 functionality.
 */
class MicroROS {
public:
    /**
     * @brief Constructor - initializes MicroROS with robot interface
     * @param robot Pointer to RobotHandle for accessing robot state and control
     */
    MicroROS(RobotHandle* robot);

    /**
     * @brief Destructor - cleans up ROS2 resources
     */
    ~MicroROS();

    /**
     * @brief Initialize ROS2 node, publishers, and subscribers
     * @return true if initialization successful, false otherwise
     *
     * This method creates the ROS2 node, sets up all publishers and subscribers,
     * and establishes network connection to the ROS2 agent.
     */
    bool initialize();

    /**
     * @brief Process ROS2 communication for specified timeout
     * @param timeout_ms Maximum time to spend processing messages (default: 100ms)
     *
     * This method handles incoming ROS2 messages and executes callbacks.
     * It should be called regularly to maintain ROS2 communication.
     */
    void spinOnce(int timeout_ms = 100);

    /**
     * @brief Publish current robot pose (odometry)
     *
     * Publishes the robot's current position, orientation, and velocity
     * as a PoseStamped message on the "Pose" topic.
     */
    void publishPose();

    /**
     * @brief Publish PID controller status and wheel information
     *
     * Publishes RPM measurements, setpoints, and PID outputs for both wheels
     * on individual topics for monitoring and debugging.
     */
    void publishPIDStatus();

    /**
     * @brief Check for command velocity timeout and stop robot if needed
     * @param timeout_ticks Maximum allowed ticks since last velocity command
     *
     * This safety feature automatically stops the robot if no velocity commands
     * are received within the specified timeout period.
     */
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
    geometry_msgs__msg__Twist cmd_vel_msg;

    // Robot reference
    RobotHandle* robot_handle;

    // Watchdog variables
    TickType_t last_cmd_vel_time;
    bool getting_cmd_vel_msg;
    bool initialized;

    // Callback functions (static to work with C API)
    static void cmdVelCallback(const void* msgin);

    // Static reference to instance for callbacks
    static MicroROS* instance;
    char frame_id_buffer[32];
};

#endif // MICRO_ROS_H
