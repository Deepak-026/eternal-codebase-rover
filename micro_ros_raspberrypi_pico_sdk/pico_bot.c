#include <stdio.h>
#include <math.h>

// Pico SDK Headers
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico_uart_transports.h"

// ROS Headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

// --- PIN DEFINITIONS (L298N) ---
#define ENA 6   // Left Speed
#define IN1 7   // Left Dir
#define IN2 8   // Left Dir
#define IN3 9   // Right Dir
#define IN4 10  // Right Dir
#define ENB 11  // Right Speed

const uint LED_PIN = 25;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

// --- PWM FUNCTION ---
void set_pwm(uint pin, float value) {
    if (value > 1.0) value = 1.0;
    if (value < 0.0) value = 0.0;
    uint16_t level = (uint16_t)(value * 65535);
    pwm_set_gpio_level(pin, level);
}

// --- CALLBACK ---
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // Toggle LED to show we got a command
    gpio_put(LED_PIN, !gpio_get(LED_PIN));

    float linear = msg->linear.x;
    float angular = msg->angular.z;
    float left = linear - angular;
    float right = linear + angular;

    // Left Motor
    if (left > 0) { gpio_put(IN1, 1); gpio_put(IN2, 0); }
    else { gpio_put(IN1, 0); gpio_put(IN2, 1); left = -left; }
    set_pwm(ENA, left);

    // Right Motor
    if (right > 0) { gpio_put(IN3, 1); gpio_put(IN4, 0); }
    else { gpio_put(IN3, 0); gpio_put(IN4, 1); right = -right; }
    set_pwm(ENB, right);
}

void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 65535);
    pwm_set_enabled(slice, true);
}

int main() {
    // 1. Init Transport
    rmw_uros_set_custom_transport(
        true, NULL, pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read
    );

    // 2. Init Hardware
    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);
    uint pins[] = {IN1, IN2, IN3, IN4};
    for(int i=0; i<4; i++) { gpio_init(pins[i]); gpio_set_dir(pins[i], GPIO_OUT); }
    setup_pwm_pin(ENA);
    setup_pwm_pin(ENB);

    // 3. WAIT FOR AGENT (Retry Forever Logic)
    // This is better than your example: it won't quit if it fails once.
    // It keeps blinking slowly until you connect the agent.
    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
        gpio_put(LED_PIN, 1); sleep_ms(200);
        gpio_put(LED_PIN, 0); sleep_ms(200);
    }

    // 4. Setup micro-ROS
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_bot_node", "", &support);

    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA);

    // Solid LED = Connected & Ready
    gpio_put(LED_PIN, 1);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
