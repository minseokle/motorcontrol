/*
 * hall_sensor.h
 *
 *  Created on: Jul 15, 2025
 *      Author: 0311b
 */

#ifndef INC_HALL_SENSOR_H_
#define INC_HALL_SENSOR_H_


#include <stdint.h>
#include "encoder_struct.h"

// --- USER CONFIGURATION REQUIRED ---
// This value must be set according to the clock frequency of the hardware timer.
// Example: If the timer clock is 1MHz (1,000,000 Hz), 1 tick is 1us.
// 1us = 0.000001s
#define TIMER_TICK_S 0.0001f

// HallSensorStruct struct definition
typedef struct {
    // --- State Variables (used internally) ---
    volatile uint8_t  hall_state;               // Current Hall sensor state (1-6)
    volatile float    electrical_angle_rad;     // Final calculated electrical angle (radians)
    volatile float    electrical_velocity_rad_s;// Estimated electrical angular velocity (rad/s)
    volatile uint32_t last_capture_time_ticks;  // Timestamp of the last Hall event (timer ticks)
    volatile int8_t   direction;                // Direction of rotation (1: forward, -1: reverse)
    volatile uint8_t  prev_hall_state;          // Previous Hall state (for direction detection)
    volatile uint8_t  is_stop;                  // is stopping not calculate angular velocity term
    volatile uint8_t  is_ovf;                   // is stop and overflow tim buf
    volatile int32_t  multiturn_cnt;            // multi turn count

    // --- Configuration Values ---
    float pole_pairs; // Number of motor pole pairs

} HallSensorStruct;

// --- Function Prototypes ---

/**
 * @brief Reads the current Hall sensor state from GPIO pins.
 * @return The current Hall state as a bitmask (1-6).
 */
uint8_t hall_sensor_get_state_from_gpio();

/**
 * @brief Initializes the HallSensorStruct struct.
 * @param h Pointer to the HallSensorStruct struct.
 * @param pole_pairs Number of motor pole pairs.
 */
void hall_sensor_init(HallSensorStruct *h, uint8_t new_hall_state, float pole_pairs);

/**
 * @brief This function must be called from an interrupt (ISR) whenever the Hall sensor state changes.
 * It calculates velocity, direction, and updates the state.
 * @param h Pointer to the HallSensorStruct struct.
 * @param new_hall_state The new Hall state (1-6) read from GPIO.
 * @param current_time_ticks The current count from the hardware timer.
 */
void hall_sensor_update(HallSensorStruct *h, uint8_t new_hall_state, uint16_t current_time_ticks);

/**
 * @brief Called from the FOC control loop to estimate the current electrical angle.
 * It interpolates the current angle based on the time elapsed since the last Hall event.
 * @param h Pointer to the HallSensorStruct struct.
 * @param current_time_ticks The current count from the hardware timer.\
 */
void hall_sensor_get_estimate_angle(HallSensorStruct *h, uint16_t current_time_ticks, BasicEncoderStruct *res);

#endif /* INC_HALL_SENSOR_H_ */
