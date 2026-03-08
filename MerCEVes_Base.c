#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "stepper.h"
#include <math.h>

#define ADC_PIN 26
#define STEP_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4

//dc driver
#define DC_PWM_PIN 0
#define M1INA_PIN 14
#define M1INB_PIN 15
#define M2INA_PIN 12
#define M2INB_PIN 13

//rpm
#define RPM_PIN_L 27
#define RPM_PIN_R 28

//spi
#define SPI_PORT spi0;

const int steps_per_rev = 1600;
const int margin_of_error = 20;

const int min_step_angle = 600;
const int max_step_angle = 1000;

volatile int current_step_angle;
volatile int pulse_count_L = 0;
volatile int pulse_count_R = 0;

const int magnets_per_wheel = 68;
const int radius = .86;

void rpm_interrupt_L(uint gpio, uint32_t events) {
    pulse_count_L++;
}
void rpm_interrupt_R(uint gpio, uint32_t events) {
    pulse_count_R++;
}

static inline int pollADC() {
    return floorf((adc_read() * steps_per_rev) / 4095.0f);
}
//sending back hall sensor from the left and hall sensor from the right and the checksum

void core1_entry() {
    while (true) {
        int adc_value = pollADC();
        int error = current_step_angle - adc_value;

        gpio_put(DIR_PIN, error > 0 ? 0 : 1);

        if (abs(error) > margin_of_error) {
            if(error > 0 && current_step_angle > min_step_angle || error < 0 && current_step_angle < max_step_angle) {
                for(int i = 0; i < 5; i++ ) {
                    gpio_put(STEP_PIN, 1);
                    sleep_us(100);   // Slightly longer than microseconds
                    gpio_put(STEP_PIN, 0);
                    sleep_us(100);   // Slightly longer than microseconds
                }
                current_step_angle += error > 0 ? -1 : 1;
            }
        }

        sleep_us(100);  // Give USB time to flush
    }
}

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    stepper_init(STEP_PIN, DIR_PIN, ENA_PIN);

    gpio_set_function(DC_PWM_PIN, GPIO_FUNC_PWM);
    uint dc_slice_num = pwm_gpio_to_slice_num(DC_PWM_PIN);

    pwm_set_enabled(dc_slice_num, true);
    pwm_set_wrap(dc_slice_num, 1000); // 16-bit resolution
    pwm_set_chan_level(dc_slice_num, PWM_CHAN_A, 150); // 50% duty cycle

    gpio_init(M1INA_PIN);
    gpio_set_dir(M1INA_PIN, GPIO_OUT);
    gpio_put(M1INA_PIN, 1);

    gpio_init(M2INA_PIN);
    gpio_set_dir(M2INA_PIN, GPIO_OUT);
    gpio_put(M2INA_PIN, 1);

    current_step_angle = 800; // Start at the middle position
    multicore_launch_core1(core1_entry);

    //rpm
    gpio_init(RPM_PIN_L);
    gpio_set_dir(RPM_PIN_L, GPIO_IN);
    gpio_set_irq_enabled_with_callback(RPM_PIN_L, GPIO_IRQ_EDGE_RISE, true, &rpm_interrupt_L);

    gpio_init(RPM_PIN_R);
    gpio_set_dir(RPM_PIN_R, GPIO_IN);
    gpio_set_irq_enabled_with_callback(RPM_PIN_R, GPIO_IRQ_EDGE_RISE, true, &rpm_interrupt_R);

    while (true) {
        sleep_ms(100);
        double rpm_L = (pulse_count_L* 2 * 2 * 3.141592) / magnets_per_wheel; // Convert to RPM
        printf("RPM (rad/s): %f\n", rpm_L);
        pulse_count_L = 0; // Reset pulse count for the next interval

        double rpm_R = (pulse_count_R* 2 * 2 * 3.141592) / magnets_per_wheel; // Convert to RPM
        printf("RPM (rad/s): %f\n", rpm_R);
        pulse_count_R = 0; // Reset pulse count for the next interval

        // get data from spi
        uint8_t rx_data[17] = {0};
        // calculate checksum
        uint8_t checksum = 0;
        for (int i = 0; i < 16; i++) {
            checksum ^= rx_data[i];
        }
        printf("Received checksum: %02X, Calculated checksum: %02X\n", rx_data[16], checksum);

    }

    
}