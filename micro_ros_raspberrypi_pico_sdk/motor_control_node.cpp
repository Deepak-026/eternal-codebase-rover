#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico_uart_transports.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
// Wheel Motors (Existing)
#define ENA 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ENB 11

#define LEFT_ENC_A 14
#define LEFT_ENC_B 15 
#define RIGHT_ENC_A 16 
#define RIGHT_ENC_B 17 

// --- NEW: Z-AXIS CAMERA MOTOR PINS ---
#define Z_ENA 18
#define Z_IN1 19
#define Z_IN2 20
#define Z_ENC_A 21
#define Z_ENC_B 22

const uint LED_PIN = 25;

// ==========================================
// CONSTANTS
// ==========================================
// Wheel Constants
#define WHEEL_DIAMETER 0.083   
#define WHEEL_BASE 0.350       
#define TICKS_PER_REV 360.0    

// Z-Axis Constants
#define Z_TICKS_PER_STEP 550   // 15cm rise based on your rim math
#define Z_TOTAL_STEPS 5        // Do this 5 times
#define Z_PAUSE_TIME 60        // 3 seconds (since loop is 20Hz: 20 * 3 = 60 counts)
#define Z_SPEED_UP 0.8         // Fast up
#define Z_SPEED_DOWN 0.4       // Slow down

// ==========================================
// GLOBALS & STATE
// ==========================================
volatile int left_wheel_ticks = 0;
volatile int right_wheel_ticks = 0;
volatile int z_axis_ticks = 0;

float x = 0.0;
float y = 0.0;
float theta = 0.0;

// Variables for Driving Logic
bool is_moving_autonomous = false;
int auto_timer_count = 0;
int start_left_ticks = 0;
int start_right_ticks = 0;
float current_linear_x = 0.0;
float current_angular_z = 0.0;

// Variables for Z-Axis State Machine
typedef enum {
    Z_IDLE,
    Z_RISING,
    Z_PAUSED,
    Z_UNWINDING
} ZState;

ZState z_state = Z_IDLE;
int z_current_step = 0;
int z_pause_counter = 0;
int z_target_ticks = 0;

// ROS Entities
rcl_publisher_t odom_publisher;
rcl_publisher_t distance_publisher;
rcl_publisher_t left_ticks_publisher;
rcl_publisher_t right_ticks_publisher;
rcl_publisher_t left_wheel_distance_publisher;
rcl_publisher_t right_wheel_distance_publisher;

rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t move_subscriber;
rcl_subscription_t z_axis_subscriber; // <--- NEW

nav_msgs_msg_Odometry odom_msg;
geometry_msgs_msg_Twist cmd_vel_msg;
std_msgs_msg_Int32 move_msg;
std_msgs_msg_Float32 dist_msg;
std_msgs_msg_Int32 left_ticks_msg;
std_msgs_msg_Int32 right_ticks_msg;
std_msgs_msg_Float32 left_wheel_distance_msg;
std_msgs_msg_Float32 right_wheel_distance_msg;
std_msgs_msg_Int32 z_axis_msg; // <--- NEW

// ==========================================
// INTERRUPTS
// ==========================================
void gpio_callback(uint gpio, uint32_t events) {
    // Left Wheel
    if (gpio == LEFT_ENC_A) {
        if (gpio_get(LEFT_ENC_B) == 0) left_wheel_ticks--;
        else left_wheel_ticks++;
    }
    // Right Wheel
    if (gpio == RIGHT_ENC_A) {
        if (gpio_get(RIGHT_ENC_B) == 0) right_wheel_ticks++; 
        else right_wheel_ticks--;
    }
    // Z-Axis Camera Motor (NEW)
    if (gpio == Z_ENC_A) {
        // Assuming Forward (Up) is positive ticks
        if (gpio_get(Z_ENC_B) == 0) z_axis_ticks++; 
        else z_axis_ticks--;
    }
}

// ==========================================
// MOTOR HELPERS
// ==========================================
void set_motor_pwm(int dir_pin1, int dir_pin2, int pwm_pin, float speed) {
    if (speed > 1.0) speed = 1.0;
    if (speed < -1.0) speed = -1.0;

    if (speed >= 0) {
        gpio_put(dir_pin1, 1); gpio_put(dir_pin2, 0);
    } else {
        gpio_put(dir_pin1, 0); gpio_put(dir_pin2, 1);
        speed = -speed;
    }
    pwm_set_gpio_level(pwm_pin, (uint16_t)(speed * 65535));
}

void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 65535);
    pwm_set_enabled(slice, true);
}

// ==========================================
// CALLBACKS
// ==========================================

void cmd_vel_callback(const void * msgin) {
    const geometry_msgs_msgTwist * msg = (const geometry_msgsmsg_Twist *)msgin;
    if (!is_moving_autonomous) {
        current_linear_x = msg->linear.x;
        current_angular_z = msg->angular.z;
    }
}

void move_callback(const void * msgin) {
    const std_msgs_msgInt32 * msg = (const std_msgsmsg_Int32 *)msgin;
    if (msg->data == 1 && !is_moving_autonomous) {
        is_moving_autonomous = true;
        auto_timer_count = 0;
        start_left_ticks = left_wheel_ticks;
        start_right_ticks = right_wheel_ticks;
        gpio_put(LED_PIN, 1); 
    }
}

// NEW: Z-Axis Trigger Callback
void z_axis_callback(const void * msgin) {
    const std_msgs_msgInt32 * msg = (const std_msgsmsg_Int32 *)msgin;
    if (msg->data == 1 && z_state == Z_IDLE) {
        // Start the Sequence
        z_state = Z_RISING;
        z_current_step = 0;
        z_target_ticks = z_axis_ticks + Z_TICKS_PER_STEP; // Target for first step
    }
}

// ==========================================
// MAIN TIMER LOOP (20Hz)
// ==========================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    static int prev_left = 0;
    static int prev_right = 0;
    float dt = 0.05; 

    // --- 1. ODOMETRY ---
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

    // Publish Odom (Raw Topic)
    int64_t time_ns = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec = time_ns / 1000000000;
    odom_msg.header.stamp.nanosec = time_ns % 1000000000;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
    odom_msg.twist.twist.linear.x = d_center / dt;
    odom_msg.twist.twist.angular.z = d_theta / dt;
    rcl_publish(&odom_publisher, &odom_msg, NULL);

    // --- 2. WHEEL MOTOR LOGIC ---
    float target_linear = 0.0;
    float target_angular = 0.0;

    if (is_moving_autonomous) {
        auto_timer_count++;
        if (auto_timer_count <= 20) { // 1 Second run
            target_linear = 0.6; 
            target_angular = 0.0;
        } else {
            // End Autonomous Move
            target_linear = 0.0; 
            target_angular = 0.0;
            
            // Calc stats
            float dist_l = (left_wheel_ticks - start_left_ticks) * dist_per_tick;
            float dist_r = (right_wheel_ticks - start_right_ticks) * dist_per_tick;
            
            dist_msg.data = (dist_l + dist_r) / 2.0;
            rcl_publish(&distance_publisher, &dist_msg, NULL);
            
            left_wheel_distance_msg.data = dist_l;
            rcl_publish(&left_wheel_distance_publisher, &left_wheel_distance_msg, NULL);
            right_wheel_distance_msg.data = dist_r;
            rcl_publish(&right_wheel_distance_publisher, &right_wheel_distance_msg, NULL);

            is_moving_autonomous = false;
            gpio_put(LED_PIN, 0);
        }
    } else {
        target_linear = current_linear_x;
        target_angular = current_angular_z;
    }

    float left_speed = target_linear - target_angular;
    float right_speed = target_linear + target_angular;
    set_motor_pwm(IN1, IN2, ENA, left_speed);
    set_motor_pwm(IN3, IN4, ENB, right_speed);

    // --- 3. Z-AXIS CAMERA LOGIC (STATE MACHINE) ---
    float z_speed = 0.0;

    switch (z_state) {
        case Z_IDLE:
            z_speed = 0.0;
            break;

        case Z_RISING:
            // Check if we reached target height for this step
            if (z_axis_ticks >= z_target_ticks) {
                z_state = Z_PAUSED;
                z_pause_counter = 0;
                z_current_step++;
            } else {
                z_speed = Z_SPEED_UP; // Move Up
            }
            break;

        case Z_PAUSED:
            z_speed = 0.0; // Hold position
            z_pause_counter++;
            // Check if 3 seconds (60 ticks) have passed
            if (z_pause_counter >= Z_PAUSE_TIME) {
                if (z_current_step < Z_TOTAL_STEPS) {
                    // Next Step
                    z_target_ticks += Z_TICKS_PER_STEP; // Add another 15cm
                    z_state = Z_RISING;
                } else {
                    // All steps done, time to unwind
                    z_state = Z_UNWINDING;
                }
            }
            break;

        case Z_UNWINDING:
            // Check if we are back to 0 (or close enough)
            if (z_axis_ticks <= 10) { 
                z_state = Z_IDLE;
                z_axis_ticks = 0; // Reset exactly
            } else {
                z_speed = -Z_SPEED_DOWN; // Move Down Slowly (Negative speed)
            }
            break;
    }
    // Apply Z-Motor PWM
    set_motor_pwm(Z_IN1, Z_IN2, Z_ENA, z_speed);

    // --- 4. DEBUG TICKS ---
    left_ticks_msg.data = left_wheel_ticks;
    right_ticks_msg.data = right_wheel_ticks;
    rcl_publish(&left_ticks_publisher, &left_ticks_msg, NULL);
    rcl_publish(&right_ticks_publisher, &right_ticks_msg, NULL);
}

// ==========================================
// MAIN
// ==========================================
int main() {
    sleep_ms(3000);
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);

    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // --- WHEEL MOTOR INIT ---
    uint pins[] = {IN1, IN2, IN3, IN4};
    for(int i = 0; i < 4; i++) { gpio_init(pins[i]); gpio_set_dir(pins[i], GPIO_OUT); }
    setup_pwm_pin(ENA); setup_pwm_pin(ENB);

    // --- Z-AXIS MOTOR INIT (NEW) ---
    gpio_init(Z_IN1); gpio_set_dir(Z_IN1, GPIO_OUT);
    gpio_init(Z_IN2); gpio_set_dir(Z_IN2, GPIO_OUT);
    setup_pwm_pin(Z_ENA);

    // --- WHEEL ENCODERS ---
    gpio_init(LEFT_ENC_A); gpio_set_dir(LEFT_ENC_A, GPIO_IN); gpio_pull_up(LEFT_ENC_A);
    gpio_init(LEFT_ENC_B); gpio_set_dir(LEFT_ENC_B, GPIO_IN); gpio_pull_up(LEFT_ENC_B);
    gpio_set_irq_enabled_with_callback(LEFT_ENC_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
    gpio_init(RIGHT_ENC_A); gpio_set_dir(RIGHT_ENC_A, GPIO_IN); gpio_pull_up(RIGHT_ENC_A);
    gpio_init(RIGHT_ENC_B); gpio_set_dir(RIGHT_ENC_B, GPIO_IN); gpio_pull_up(RIGHT_ENC_B);
    gpio_set_irq_enabled(RIGHT_ENC_A, GPIO_IRQ_EDGE_RISE, true);

    // --- Z-AXIS ENCODER (NEW) ---
    gpio_init(Z_ENC_A); gpio_set_dir(Z_ENC_A, GPIO_IN); gpio_pull_up(Z_ENC_A);
    gpio_init(Z_ENC_B); gpio_set_dir(Z_ENC_B, GPIO_IN); gpio_pull_up(Z_ENC_B);
    gpio_set_irq_enabled(Z_ENC_A, GPIO_IRQ_EDGE_RISE, true); // Share callback

    // No Sync Loop (Start Raw)
    while (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
        gpio_put(LED_PIN, 1); sleep_ms(200); gpio_put(LED_PIN, 0); sleep_ms(200);
    }

    // ROS Init
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_timer_t timer;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_main_node", "", &support);

    // Publishers
    rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom_raw");
    rclc_publisher_init_default(&distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "distance");
    rclc_publisher_init_default(&left_ticks_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_ticks");
    rclc_publisher_init_default(&right_ticks_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_ticks");
    rclc_publisher_init_default(&left_wheel_distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "left_wheel_distance");
    rclc_publisher_init_default(&right_wheel_distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "right_wheel_distance");

    // Init Strings
    odom_msg.header.frame_id.data = (char*)malloc(20);
    odom_msg.header.frame_id.capacity = 20;
    sprintf(odom_msg.header.frame_id.data, "odom");
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.child_frame_id.data = (char*)malloc(20);
    odom_msg.child_frame_id.capacity = 20;
    sprintf(odom_msg.child_frame_id.data, "base_link");
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

    // Subscribers
    rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    rclc_subscription_init_default(&move_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "move");
    // NEW Z Subscriber
    rclc_subscription_init_default(&z_axis_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "z_axis_control");

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);

    // Executor (Increase handle count to 7)
    rclc_executor_init(&executor, &support.context, 7, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &move_subscriber, &move_msg, &move_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &z_axis_subscriber, &z_axis_msg, &z_axis_callback, ON_NEW_DATA); // <--- NEW
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}