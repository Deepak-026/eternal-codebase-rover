#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- PIN DEFINITIONS ---
#define ENA 6   // Left Speed
#define IN1 7   // Left Dir
#define IN2 8   // Left Dir
#define IN3 9   // Right Dir
#define IN4 10  // Right Dir
#define ENB 11  // Right Speed
#define LED_PIN 25

// Setup a pin for PWM
void setup_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 65535);
    pwm_set_enabled(slice, true);
}

// Set speed (0 to 65535)
void set_speed(uint pin, uint16_t speed) {
    pwm_set_gpio_level(pin, speed);
}

int main() {
    // 1. Initialize LED
    gpio_init(LED_PIN); 
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // 2. Initialize Direction Pins
    uint pins[] = {IN1, IN2, IN3, IN4};
    for(int i=0; i<4; i++) { 
        gpio_init(pins[i]); 
        gpio_set_dir(pins[i], GPIO_OUT); 
    }

    // 3. Initialize Speed Pins (PWM)
    setup_pwm(ENA);
    setup_pwm(ENB);

    while (true) {
        // --- PHASE 1: FORWARD (2 Seconds) ---
        gpio_put(LED_PIN, 1); // LED ON
        
        // Set Direction Forward
        gpio_put(IN1, 1); gpio_put(IN2, 0);
        gpio_put(IN3, 1); gpio_put(IN4, 0);
        
        // Set Speed (50% power)
        set_speed(ENA, 30000);
        set_speed(ENB, 30000);
        
        sleep_ms(2000);

        // --- PHASE 2: STOP (1 Second) ---
        set_speed(ENA, 0);
        set_speed(ENB, 0);
        sleep_ms(1000);

        // --- PHASE 3: BACKWARD (2 Seconds) ---
        gpio_put(LED_PIN, 0); // LED OFF
        
        // Set Direction Backward
        gpio_put(IN1, 0); gpio_put(IN2, 1);
        gpio_put(IN3, 0); gpio_put(IN4, 1);
        
        // Set Speed (50% power)
        set_speed(ENA, 30000);
        set_speed(ENB, 30000);

        sleep_ms(2000);
        
        // --- PHASE 4: STOP (1 Second) ---
        set_speed(ENA, 0);
        set_speed(ENB, 0);
        sleep_ms(1000);
    }
}
