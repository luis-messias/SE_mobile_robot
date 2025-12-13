#include "MicroROS.h"
#include <config.h>
#include <uros_network_interfaces.h>
#include <geometry_msgs/msg/quaternion.h>
#include <utils.h>

// Static member initialization
MicroROS* MicroROS::instance = nullptr;

MicroROS::MicroROS(RobotHandle* robot)
    : robot_handle(robot), last_cmd_vel_time(0), initialized(false) {
    instance = this;
}

MicroROS::~MicroROS() {
    if (initialized) {

        // Clean up ROS resources
        rcl_ret_t ret;
        ret = rcl_publisher_fini(&pose_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini pose publisher: %d", ret);
        }

        ret = rcl_publisher_fini(&rpm_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini RPM publisher: %d", ret);
        }

        ret = rcl_publisher_fini(&rpm_left_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini RPM left publisher: %d", ret);
        }

        ret = rcl_publisher_fini(&rpm_right_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini RPM right publisher: %d", ret);
        }

        ret = rcl_publisher_fini(&rpm_left_setpoint_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini RPM left setpoint publisher: %d", ret);
        }

        ret = rcl_publisher_fini(&rpm_right_setpoint_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini RPM right setpoint publisher: %d", ret);
        }

        ret = rcl_publisher_fini(&pid_left_output_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini PID left output publisher: %d", ret);
        }

        ret = rcl_publisher_fini(&pid_right_output_publisher, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini PID right output publisher: %d", ret);
        }

        ret = rcl_subscription_fini(&cmd_vel_subscriber, &node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini cmd_vel subscriber: %d", ret);
        }

        ret = rclc_executor_fini(&executor);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini executor: %d", ret);
        }

        ret = rcl_node_fini(&node);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini node: %d", ret);
        }

        ret = rclc_support_fini(&support);
        if (ret != RCL_RET_OK) {
            ESP_LOGW("MICROROS", "Failed to fini support: %d", ret);
        }

        // Clean up messages
        geometry_msgs__msg__PoseStamped__fini(&pose_msg);
        geometry_msgs__msg__Twist__fini(&cmd_vel_msg);
        std_msgs__msg__Float32__fini(&rpm_left_msg);
        std_msgs__msg__Float32__fini(&rpm_right_msg);
        std_msgs__msg__Float32__fini(&rpm_left_setpoint_msg);
        std_msgs__msg__Float32__fini(&rpm_right_setpoint_msg);
        std_msgs__msg__Float32__fini(&pid_left_output_msg);
        std_msgs__msg__Float32__fini(&pid_right_output_msg);
        getting_cmd_vel_msg = false;
    }
}

bool MicroROS::initialize() {
    if (initialized) {
        ESP_LOGI("MICROROS", "Already initialized, skipping...");
        return true;
    }

    ESP_LOGI("MICROROS", "Starting MicroROS initialization...");

    allocator = rcl_get_default_allocator();

    // Initialize support with
    ESP_LOGI("MICROROS", "Initializing ROS support...");
    rclc_support_t temp_support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t init_options_ret = rcl_init_options_init(&init_options, allocator);
    if (init_options_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to initialize init options: %d", init_options_ret);
        return false;
    }
    ESP_LOGD("MICROROS", "Init options initialized successfully");

    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    if (rmw_options == NULL) {
        ESP_LOGE("MICROROS", "Failed to get RMW init options");
        return false;
    }

    rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
    ESP_LOGD("MICROROS", "UDP address configured: %s:%s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);

    rcl_ret_t support_ret = rclc_support_init_with_options(&temp_support, 0, NULL, &init_options, &allocator);
    if (support_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to initialize support: %d", support_ret);
        return false;
    }
    support = temp_support;

    // Initialize executor
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(EXECUTOR_TIMEOUT_MS));

    // Create node
    rcl_node_t temp_node;
    rcl_ret_t node_ret = rclc_node_init_default(&temp_node, "SE_mobile_robot", "", &support);
    if (node_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create node: %d", node_ret);
        return false;
    }
    node = temp_node;

    // Initialize pose publisher
    geometry_msgs__msg__PoseStamped__init(&pose_msg);
    pose_msg.header.frame_id.data = (char*)"map";
    pose_msg.header.frame_id.size = strlen("map");
    pose_msg.header.frame_id.capacity = strlen("map") + 1;

    rcl_publisher_t temp_publisher;
    rcl_ret_t pub_ret = rclc_publisher_init_default(
        &temp_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), "Pose");
    if (pub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create pose publisher: %d", pub_ret);
        return false;
    }
    pose_publisher = temp_publisher;
    ESP_LOGI("MICROROS", "Pose publisher created successfully");

    // Initialize PID status publishers
    std_msgs__msg__Float32__init(&rpm_left_msg);
    std_msgs__msg__Float32__init(&rpm_right_msg);
    std_msgs__msg__Float32__init(&rpm_left_setpoint_msg);
    std_msgs__msg__Float32__init(&rpm_right_setpoint_msg);
    std_msgs__msg__Float32__init(&pid_left_output_msg);
    std_msgs__msg__Float32__init(&pid_right_output_msg);

    // Initialize RPM left publisher
    rcl_publisher_t temp_rpm_left_pub;
    rcl_ret_t rpm_left_pub_ret = rclc_publisher_init_default(
        &temp_rpm_left_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pid/rpm_left");
    if (rpm_left_pub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create RPM left publisher: %d", rpm_left_pub_ret);
        return false;
    }
    rpm_left_publisher = temp_rpm_left_pub;
    ESP_LOGD("MICROROS", "RPM left publisher created successfully");

    // Initialize RPM right publisher
    rcl_publisher_t temp_rpm_right_pub;
    rcl_ret_t rpm_right_pub_ret = rclc_publisher_init_default(
        &temp_rpm_right_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pid/rpm_right");
    if (rpm_right_pub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create RPM right publisher: %d", rpm_right_pub_ret);
        return false;
    }
    rpm_right_publisher = temp_rpm_right_pub;
    ESP_LOGD("MICROROS", "RPM right publisher created successfully");

    // Initialize RPM left setpoint publisher
    rcl_publisher_t temp_rpm_left_sp_pub;
    rcl_ret_t rpm_left_sp_pub_ret = rclc_publisher_init_default(
        &temp_rpm_left_sp_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pid/rpm_left_setpoint");
    if (rpm_left_sp_pub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create RPM left setpoint publisher: %d", rpm_left_sp_pub_ret);
        return false;
    }
    rpm_left_setpoint_publisher = temp_rpm_left_sp_pub;
    ESP_LOGD("MICROROS", "RPM left setpoint publisher created successfully");

    // Initialize RPM right setpoint publisher
    rcl_publisher_t temp_rpm_right_sp_pub;
    rcl_ret_t rpm_right_sp_pub_ret = rclc_publisher_init_default(
        &temp_rpm_right_sp_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pid/rpm_right_setpoint");
    if (rpm_right_sp_pub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create RPM right setpoint publisher: %d", rpm_right_sp_pub_ret);
        return false;
    }
    rpm_right_setpoint_publisher = temp_rpm_right_sp_pub;
    ESP_LOGD("MICROROS", "RPM right setpoint publisher created successfully");

    // Initialize PID left output publisher
    rcl_publisher_t temp_pid_left_pub;
    rcl_ret_t pid_left_pub_ret = rclc_publisher_init_default(
        &temp_pid_left_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pid/pid_left_output");
    if (pid_left_pub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create PID left output publisher: %d", pid_left_pub_ret);
        return false;
    }
    pid_left_output_publisher = temp_pid_left_pub;
    ESP_LOGD("MICROROS", "PID left output publisher created successfully");

    // Initialize PID right output publisher
    rcl_publisher_t temp_pid_right_pub;
    rcl_ret_t pid_right_pub_ret = rclc_publisher_init_default(
        &temp_pid_right_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pid/pid_right_output");
    if (pid_right_pub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create PID right output publisher: %d", pid_right_pub_ret);
        return false;
    }
    pid_right_output_publisher = temp_pid_right_pub;
    ESP_LOGD("MICROROS", "PID right output publisher created successfully");

    ESP_LOGI("MICROROS", "PID status publishers created successfully");

    // Initialize cmd_vel subscriber
    geometry_msgs__msg__Twist__init(&cmd_vel_msg);

    rcl_subscription_t temp_cmd_sub;
    rcl_ret_t cmd_sub_ret = rclc_subscription_init_default(
        &temp_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    if (cmd_sub_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to create cmd_vel subscriber: %d", cmd_sub_ret);
        return false;
    }
    cmd_vel_subscriber = temp_cmd_sub;
    ESP_LOGD("MICROROS", "Cmd_vel subscriber created successfully");

    rcl_ret_t cmd_add_ret = rclc_executor_add_subscription(
        &executor, &cmd_vel_subscriber, &cmd_vel_msg, &MicroROS::cmdVelCallback, ON_NEW_DATA);
    if (cmd_add_ret != RCL_RET_OK) {
        ESP_LOGE("MICROROS", "Failed to add cmd_vel subscriber to executor: %d", cmd_add_ret);
        return false;
    }
    ESP_LOGD("MICROROS", "Cmd_vel subscriber added to executor successfully");
    ESP_LOGI("MICROROS", "Cmd_vel subscriber created successfully");

    initialized = true;
    ESP_LOGI("MICROROS", "MicroROS initialization completed successfully");
    return true;
}

void MicroROS::spinOnce(int timeout_ms) {
    if (!initialized) return;

    rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(timeout_ms));
    if (spin_ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Executor spin failed: %d.", spin_ret);
    }
}

void MicroROS::publishPose() {
    if (!initialized) return;

    auto [vx, w, x, y, alpha] = robot_handle->getOdometry();

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation = eulerToQuaternion(0, 0, alpha);

    pose_msg.header.stamp.sec = xTaskGetTickCount() / configTICK_RATE_HZ;
    pose_msg.header.stamp.nanosec = (xTaskGetTickCount() % configTICK_RATE_HZ) *
                                    (1000000000 / configTICK_RATE_HZ);

    rcl_ret_t ret = rcl_publish(&pose_publisher, &pose_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Failed to publish pose: %d", ret);
    }
}

void MicroROS::publishPIDStatus() {
    if (!initialized) return;

    pidStatus status = robot_handle->getPIDStatus();

    // Publish RPM left
    rpm_left_msg.data = status.rpmLeft;
    rcl_ret_t ret = rcl_publish(&rpm_left_publisher, &rpm_left_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Failed to publish RPM left: %d", ret);
    }

    // Publish RPM right
    rpm_right_msg.data = status.rpmRight;
    ret = rcl_publish(&rpm_right_publisher, &rpm_right_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Failed to publish RPM right: %d", ret);
    }

    // Publish RPM left setpoint
    rpm_left_setpoint_msg.data = status.rmpLeftSetPoint;
    ret = rcl_publish(&rpm_left_setpoint_publisher, &rpm_left_setpoint_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Failed to publish RPM left setpoint: %d", ret);
    }

    // Publish RPM right setpoint
    rpm_right_setpoint_msg.data = status.rpmRightSetPoint;
    ret = rcl_publish(&rpm_right_setpoint_publisher, &rpm_right_setpoint_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Failed to publish RPM right setpoint: %d", ret);
    }

    // Publish PID left output
    pid_left_output_msg.data = status.outPIDLeft;
    ret = rcl_publish(&pid_left_output_publisher, &pid_left_output_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Failed to publish PID left output: %d", ret);
    }

    // Publish PID right output
    pid_right_output_msg.data = status.outPIDRight;
    ret = rcl_publish(&pid_right_output_publisher, &pid_right_output_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW("MICROROS", "Failed to publish PID right output: %d", ret);
    }
}

void MicroROS::checkCmdVelTimeout(TickType_t timeout_ticks) {
    if (!initialized) return;

    TickType_t current_time = xTaskGetTickCount();
    if (getting_cmd_vel_msg && (current_time - last_cmd_vel_time) > timeout_ticks) {
        robot_handle->stop();
        getting_cmd_vel_msg = false; 
        ESP_LOGI("MICROROS", "cmd_vel timeout - velocity reset to zero");       
    }
}

void MicroROS::cmdVelCallback(const void* msgin) {
    if (!instance) return;

    const geometry_msgs__msg__Twist* cmd_vel = (const geometry_msgs__msg__Twist*)msgin;
    instance->robot_handle->setVelocity(cmd_vel->linear.x, cmd_vel->angular.z);
    instance->last_cmd_vel_time = xTaskGetTickCount();
    instance->getting_cmd_vel_msg = true;
}
