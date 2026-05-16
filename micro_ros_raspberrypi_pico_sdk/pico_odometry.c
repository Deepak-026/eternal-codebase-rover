#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Pico SDK Headers
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico_uart_transports.h"

// ROS Headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rmw_microros/rmw_microros.h>

// --- PIN DEFINITIONS ---
// L298N Motors
#define ENA 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ENB 11

// Encoders
#define LEFT_ENCODER_PIN 14
#define RIGHT_ENCODER_PIN 15

const uint LED_PIN = 25;

// --- ROBOT CONSTANTS ---
#define WHEEL_DIAMETER 0.083   // meters (83mm)
#define WHEEL_BASE 0.370       // meters (370mm)
#define TICKS_PER_REV 360.0    // Encoder Resolution

// --- GLOBALS ---
volatile int left_wheel_ticks = 0;
volatile int right_wheel_ticks = 0;

float x = 0.0;
float y = 0.0;
float theta = 0.0;

rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscriber;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// --- INTERRUPTS ---
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER_PIN) left_wheel_ticks++;
    if (gpio == RIGHT_ENCODER_PIN) right_wheel_ticks++;
}

// --- MOTORS ---
void set_pwm(uint pin, float value) {
    if (value > 1.0) value = 1.0;
    if (value < 0.0) value = 0.0;
    pwm_set_gpio_level(pin, (uint16_t)(value * 65535));
}

void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 65535);
    pwm_set_enabled(slice, true);
}

void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    gpio_put(LED_PIN, !gpio_get(LED_PIN)); // Toggle LED

    float linear = msg->linear.x;
    float angular = msg->angular.z;
    float left = linear - angular;
    float right = linear + angular;

    if (left > 0) { gpio_put(IN1, 1); gpio_put(IN2, 0); }
    else { gpio_put(IN1, 0); gpio_put(IN2, 1); left = -left; }
    set_pwm(ENA, left);

    if (right > 0) { gpio_put(IN3, 1); gpio_put(IN4, 0); }
    else { gpio_put(IN3, 0); gpio_put(IN4, 1); right = -right; }
    set_pwm(ENB, right);
}

// --- ODOMETRY ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    static int prev_left = 0;
    static int prev_right = 0;

    int d_left_ticks = left_wheel_ticks - prev_left;
    int d_right_ticks = right_wheel_ticks - prev_right;
    prev_left = left_wheel_ticks;
    prev_right = right_wheel_ticks;

    float dist_per_tick = (M_PI * WHEEL_DIAMETER) / TICKS_PER_REV;
    float d_left = d_left_ticks * dist_per_tick;
    float d_right = d_right_ticks * dist_per_tick;

    float d_center = (d_left + d_right) / 2.0;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    x += d_center * cos(theta);
    y += d_center * sin(theta);
    theta += d_theta;

    // Time Sync (Replaces rclcpp::Clock)
    int64_t time_ns = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec = time_ns / 1000000000;
    odom_msg.header.stamp.nanosec = time_ns % 1000000000;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    
    // Manual Quaternion (Replaces tf2)
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

    rcl_publish(&odom_publisher, &odom_msg, NULL);
}

int main() {
    sleep_ms(3000); // Wait for USB
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);

    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Motor Init
    uint pins[] = {IN1, IN2, IN3, IN4};
    for(int i=0; i<4; i++) { gpio_init(pins[i]); gpio_set_dir(pins[i], GPIO_OUT); }
    setup_pwm_pin(ENA); setup_pwm_pin(ENB);

    // Encoder Init
    gpio_init(LEFT_ENCODER_PIN); gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_init(RIGHT_ENCODER_PIN); gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Wait for Agent
    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
        gpio_put(LED_PIN, 1); sleep_ms(200); gpio_put(LED_PIN, 0); sleep_ms(200);
    }
    rmw_uros_sync_session(1000);

    // ROS Init
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_timer_t timer;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_odom_node", "", &support);

    // Odom Publisher
    rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
    
    // Initialize Strings
    odom_msg.header.frame_id.data = (char*)malloc(20);
    odom_msg.header.frame_id.capacity = 20;
    sprintf(odom_msg.header.frame_id.data, "odom");
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    
    odom_msg.child_frame_id.data = (char*)malloc(20);
    odom_msg.child_frame_id.capacity = 20;
    sprintf(odom_msg.child_frame_id.data, "base_link");
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

    // Subscriber
    rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    // Timer
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);

    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1); // Ready

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
