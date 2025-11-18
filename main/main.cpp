#include <math.h>

#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc/executor.h>

#include <EngineDriver.h>
#include <EncoderDriver.h>
#include <PID.h>

#define PI 3.14159265359

typedef struct {
    EngineDriver* engineDriver;
    EncoderDriver* encoderDriver;
    PID* pid;
} wheelHandle;

typedef struct {
    float vx;
    float w;
    float x;
    float y;
    float alpha;
} Odometry;

class robotHandle{
public:
    robotHandle(wheelHandle* leftWheel, wheelHandle* rightWheel, float wheelsRadius, float wheelsDistance) : 
        m_leftWheel(leftWheel),
        m_rightWheel(rightWheel),
        m_wheelsRadius(wheelsRadius),
        m_wheelsDistance(wheelsDistance) {
            m_odometry = {0,0,0,0,0};
        }
    
    void setVelocity(float vx, float w){
        float vLeft = vx + w * m_wheelsDistance / 2;
        float vRight = vx - w * m_wheelsDistance / 2;
        m_leftWheel->pid->setSetPoint((60 * vLeft) / (m_wheelsRadius * 2 * PI));
        m_rightWheel->pid->setSetPoint((60 * vRight) / (m_wheelsRadius * 2 * PI));
    }

    void updateOdometry(){
        auto [vx, w] = getVelocity();
        float dx = 0;
        float dy = 0;
        float dAlpha = 0;

        TickType_t xCurrentTick = xTaskGetTickCount();
        if(xCurrentTick != xLastUpdateTick){
            float dt = ((float)xCurrentTick - xLastUpdateTick)/configTICK_RATE_HZ;
            float ds = vx * dt;

            dAlpha = w * dt;
            dx = std::cos(m_odometry.alpha) * ds;
            dy = std::sin(m_odometry.alpha) * ds;   

            xLastUpdateTick = xCurrentTick;
        }
        m_odometry = {vx, w, m_odometry.x + dx, m_odometry.y + dy, m_odometry.alpha + dAlpha};
    }

    Odometry getOdometry(){return m_odometry;}

private:
    std::pair<float, float> getVelocity(){
        float vLeft = m_leftWheel->encoderDriver->getRPM() * 2 * PI * m_wheelsRadius / 60;
        float VRight = m_rightWheel->encoderDriver->getRPM() * 2 * PI * m_wheelsRadius/ 60;
        return {(vLeft + VRight) / 2, (vLeft - VRight) / m_wheelsDistance};
    }

    wheelHandle* m_leftWheel;
    wheelHandle* m_rightWheel;
    float m_wheelsRadius;
    float m_wheelsDistance;
    Odometry m_odometry;
    TickType_t xLastUpdateTick;
};

#define deadZone 0.05

auto engineDriverLeft = EngineDriver({12, 27}, 
                            LEDC_TIMER_0, 
                            std::pair{LEDC_CHANNEL_0, LEDC_CHANNEL_1},
                            deadZone);
auto encoderDriverLeft = EncoderDriver(21, 22, 823.1);
auto pidLeft = PID(0.003, 0.03, 0.0003);
auto wheelLeft = wheelHandle(&engineDriverLeft, &encoderDriverLeft, &pidLeft);

auto engineDriverRight = EngineDriver({26, 25}, 
                            LEDC_TIMER_0, 
                            std::pair{LEDC_CHANNEL_2, LEDC_CHANNEL_3},
                            deadZone);
auto encoderDriverRight = EncoderDriver(32, 13, 823.1);
auto pidRight = PID(0.003, 0.03, 0.0003);
auto wheelRight = wheelHandle(&engineDriverRight, &encoderDriverRight, &pidRight);

auto robot = robotHandle(&wheelLeft, &wheelRight, 0.130/2, 0.020);

// Watchdog for cmd_vel timeout
static TickType_t lastCmdVelTime = 0;
static const TickType_t CMD_VEL_TIMEOUT = 2000 / portTICK_PERIOD_MS;  // 1 second timeout


extern "C" {
    void app_main(void);
}

void printHeapMemoryUsage() {
    uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t totalHeap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    uint32_t usedHeap = totalHeap - freeHeap;
    float heapUsagePercent = (float)usedHeap / totalHeap * 100.0f;
    
    ESP_LOGI("MEMORY", "Heap Usage: %.1f%% (%lu / %lu bytes)", heapUsagePercent, usedHeap, totalHeap);
}

void printStackMemoryUsage() {
    TaskHandle_t pidEnginesHandle = xTaskGetHandle("pidEngines");
    TaskHandle_t urosTaskHandle = xTaskGetHandle("uros_task");

    if (pidEnginesHandle != NULL) {
        UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(pidEnginesHandle);
        float stackUsagePercent = (1.0f - (float)stackRemaining / 2048.0f) * 100.0f;
        ESP_LOGI("MEMORY", "pidEngines Stack: %.1f%% (%u / 1024 bytes free)", stackUsagePercent, stackRemaining);
    }
    
    if (urosTaskHandle != NULL) {
        UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(urosTaskHandle);
        float stackUsagePercent = (1.0f - (float)stackRemaining / 10096.0f) * 100.0f;
        ESP_LOGI("MEMORY", "uros_task Stack: %.1f%% (%u / 10096 bytes free)", stackUsagePercent, stackRemaining);
    }
}


// Convert Euler angles (roll, pitch, yaw) in radians to geometry_msgs quaternion
geometry_msgs__msg__Quaternion eulerToQuaternion(float roll, float pitch, float yaw) {
    // Compute half angles
    float roll_2 = roll / 2.0f;
    float pitch_2 = pitch / 2.0f;
    float yaw_2 = yaw / 2.0f;
    
    // Precompute sin and cos of half angles
    float sin_roll_2 = sinf(roll_2);
    float cos_roll_2 = cosf(roll_2);
    float sin_pitch_2 = sinf(pitch_2);
    float cos_pitch_2 = cosf(pitch_2);
    float sin_yaw_2 = sinf(yaw_2);
    float cos_yaw_2 = cosf(yaw_2);
    
    // Apply quaternion formula
    geometry_msgs__msg__Quaternion q;
    q.x = sin_roll_2 * cos_pitch_2 * cos_yaw_2 - cos_roll_2 * sin_pitch_2 * sin_yaw_2;
    q.y = cos_roll_2 * sin_pitch_2 * cos_yaw_2 + sin_roll_2 * cos_pitch_2 * sin_yaw_2;
    q.z = cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * cos_yaw_2;
    q.w = cos_roll_2 * cos_pitch_2 * cos_yaw_2 + sin_roll_2 * sin_pitch_2 * sin_yaw_2;
    
    return q;
}

void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * cmd_vel = (const geometry_msgs__msg__Twist *)msgin;
    robot.setVelocity(cmd_vel->linear.x, cmd_vel->angular.z);
    lastCmdVelTime = xTaskGetTickCount();  // Update the timestamp
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, allocator);
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1000));

	// create node
	rcl_node_t node;
	rclc_node_init_default(&node, "SE_mobile_robot", "", &support);

    geometry_msgs__msg__PoseStamped pose_msg;
    pose_msg.header.frame_id.data = (char*)"base_link";
    pose_msg.header.frame_id.size = strlen("base_link");
    rcl_publisher_t publisherPose;
	rcl_ret_t ret0 = rclc_publisher_init_default(
		&publisherPose,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
		"Pose");
    if (ret0 != RCL_RET_OK) {
        ESP_LOGI("UROS", "Failed to create Pose publisher: %d", ret0);
    } else {
        ESP_LOGI("UROS", "Pose publisher created successfully");
    }

    geometry_msgs__msg__Twist cmdVel;
    rcl_subscription_t cmd_vel_sub;
    rcl_ret_t rc1 = rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    if (rc1 != RCL_RET_OK) {
        ESP_LOGI("UROS", "Failed to init Twist subscribe: %d", rc1);
    } else {
        ESP_LOGI("UROS", "Twist subscribe inited successfully");
    }

    rcl_ret_t rc2 = rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmdVel,
        &cmd_vel_callback,
        ON_NEW_DATA);
    if (rc2 != RCL_RET_OK) {
        ESP_LOGI("UROS", "Failed to add Twist subscribe: %d", rc1);
    } else {
        ESP_LOGI("UROS", "Twist subscribe added successfully");
    }

	while(1){
        // Spin the executor to handle subscription callbacks
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        
        // Watchdog: Check if cmd_vel timeout occurred
        TickType_t currentTime = xTaskGetTickCount();
        if ((currentTime - lastCmdVelTime) > CMD_VEL_TIMEOUT) {
            // Timeout occurred - reset velocity to zero
            robot.setVelocity(0.0f, 0.0f);
            ESP_LOGI("WATCHDOG", "cmd_vel timeout - velocity reset to zero");
            lastCmdVelTime = currentTime;  // Reset the timer to avoid spam logging
        }
        
        auto [vx, w, x, y, alpha] = robot.getOdometry();
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = 0;
        pose_msg.pose.orientation = eulerToQuaternion(0, 0, alpha);
        pose_msg.header.stamp.sec = xTaskGetTickCount() / configTICK_RATE_HZ;
        pose_msg.header.stamp.nanosec = (xTaskGetTickCount() % configTICK_RATE_HZ) * (1000000000 / configTICK_RATE_HZ);
        
        rcl_publish(&publisherPose, &pose_msg, NULL);

        vTaskDelay(200/portTICK_PERIOD_MS);
	}
}

void pidEnginesTask(void *args) {
    float rpmLeft, rpmRight;
    float outLeft, outRight;

    while(1){
        encoderDriverLeft.updateRPM();
        encoderDriverRight.updateRPM();

        rpmLeft = encoderDriverLeft.getRPM();
        rpmRight = encoderDriverRight.getRPM();

        outLeft = pidLeft.getOut(rpmLeft, 0.1);
        outRight = pidRight.getOut(rpmRight, 0.1);

        engineDriverLeft.setOutput(outLeft); 
        engineDriverRight.setOutput(outRight); 

        robot.updateOdometry();

        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

void app_main()
{	   
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(&pidEnginesTask, "pidEngines", 2048, NULL, 5, NULL);
    xTaskCreate(&micro_ros_task, "uros_task", 10096, NULL, 5, NULL);

    printf("=====================================================================\n");
    printHeapMemoryUsage();
    printStackMemoryUsage();

    while(1){
        printf("=====================================================================\n");
        printHeapMemoryUsage();
        printStackMemoryUsage();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}