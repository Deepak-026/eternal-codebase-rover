#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

rcl_subscription_t subscriber;
std_msgs__msg__Int32 sub_msg;

// Callback function to handle received messages
void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    printf("Pico received: %d\n", msg->data);
}

int main()
{
    // Initialize the micro-ROS transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Initialize micro-ROS
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize micro-ROS support: %d\n", ret);
        return -1;
    }

    // Create a node
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "pico_subscriber_node", "", &support);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize node: %d\n", ret);
        return -1;
    }

    // Initialize subscriber to listen on /my_test_topic
    ret = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "my_test_topic");
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize subscription: %d\n", ret);
        return -1;
    }

    // Create an executor
    rclc_executor_t executor;
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize executor: %d\n", ret);
        return -1;
    }

    // Add subscription to executor
    ret = rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        printf("Failed to add subscription to executor: %d\n", ret);
        return -1;
    }

    printf("Pico subscriber ready! Listening to /my_test_topic (Int32)\n");

    // Main loop to spin and process messages
    while (true)
    {
        // Spin the executor and process callbacks
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));  // Timeout of 100ms
    }

    return 0;
}

