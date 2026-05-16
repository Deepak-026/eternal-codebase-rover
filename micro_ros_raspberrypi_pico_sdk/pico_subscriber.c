#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

// Function that runs when a message is received
void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    
    // Toggle LED so you can SEE it working physically
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

int main()
{
    // 1. Setup USB Transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // 2. WAIT FOR AGENT (This is the Magic Step!)
    // The Pico will pause here until you start the agent on the PC.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // If computer didn't answer, flash LED fast to show error
        while(1) {
            gpio_put(LED_PIN, 1); sleep_ms(100);
            gpio_put(LED_PIN, 0); sleep_ms(100);
        }
        return ret; 
    }

    // 3. Initialize micro-ROS
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_subscriber_node", "", &support);

    // 4. Create Subscriber (Listening to 'pico_publisher')
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    // 5. Create Executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    // Turn LED on solid to show we are ready
    gpio_put(LED_PIN, 1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
