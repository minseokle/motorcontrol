/*
 * hall_sensor.c
 *
 *  Created on: Jul 15, 2025
 *      Author: 0311b
 */
#include "hall_sensor.h"

#include <limits.h>  // For UINT32_MAX
#include <math.h>    // For fmodf

#include "hw_config.h"
#include "main.h"
#include "math_ops.h"
#include "user_config.h"

// --- Constants ---

// These tables are now logically consistent. They assume a standard 120-degree
// Hall sensor placement and a W-V-U to CBA bit mapping (W=MSB).
// This mapping defines the START angle of each 60-degree sector.

static const float hall_state_angle_lut[7] = {
  0.0f,                           // [0] Error state
  4 * PI_OVER_3_F + PI_OVER_6_F,  // 240 deg (State 1: 001)
  2 * PI_OVER_3_F + PI_OVER_6_F,  // 120 deg (State 2: 010)
  3 * PI_OVER_3_F + PI_OVER_6_F,  // 180 deg (State 3: 011)
  0 * PI_OVER_3_F + PI_OVER_6_F,  // 0 deg   (State 4: 100)
  5 * PI_OVER_3_F + PI_OVER_6_F,  // 300 deg (State 5: 101)
  1 * PI_OVER_3_F + PI_OVER_6_F   // 60 deg  (State 6: 110)
};

// Forward sequence derived from the angle LUT above.
// hall_forward_sequence[current_state] = next_state
static const uint8_t hall_forward_sequence[7] = {
  0,  // [0]
  5,  // State 1 (240) -> State 5 (300)
  3,  // State 2 (120) -> State 3 (180)
  1,  // State 3 (180) -> State 1 (240)
  6,  // State 4 (0)   -> State 6 (60)
  4,  // State 5 (300) -> State 4 (360/0)
  2   // State 6 (60)  -> State 2 (120)
};

// Reverse sequence is the inverse of the forward sequence.
static const uint8_t hall_reverse_sequence[7] = {
  0,  // [0]
  3,  // State 1 (240) -> State 3 (180)
  6,  // State 2 (120) -> State 6 (60)
  2,  // State 3 (180) -> State 2 (120)
  5,  // State 4 (0)   -> State 5 (300)
  1,  // State 5 (300) -> State 1 (240)
  4   // State 6 (60)  -> State 4 (0)
};

uint8_t hall_sensor_get_state_from_gpio()
{
  return (
    (HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) << 2) |
    (HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin) << 1) |
    (HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin) << 0));
}

void hall_sensor_init(HallSensorStruct * h, uint8_t new_hall_state, float pole_pairs)
{
  if (new_hall_state == 0 || new_hall_state > 6) {
    new_hall_state = 0;
  }
  h->hall_state = new_hall_state;
  h->prev_hall_state = new_hall_state;  // Initialize previous state to current state
  h->electrical_angle_rad = hall_state_angle_lut[new_hall_state];
  h->electrical_velocity_rad_s = 0.0f;
  h->last_capture_time_ticks = 0;
  h->direction = 0;  // Default to forward direction
  h->multiturn_cnt = 0;
  h->pole_pairs = pole_pairs;
  h->is_ovf = 1;
  h->is_stop = 1;
  h->list_idx = 0;
}

void hall_sensor_update(HallSensorStruct * h, uint8_t new_hall_state, uint16_t current_time_ticks)
{
  int8_t prev_direction = h->direction;
  if (new_hall_state == 0 || new_hall_state > 6 || new_hall_state == h->hall_state) {
    // Ignore invalid or unchanged Hall states.
    return;
  }
  h->hall_state = new_hall_state;

  // Determine direction.
  if (h->prev_hall_state != 0) {
    if (hall_forward_sequence[h->prev_hall_state] == h->hall_state) {
      h->direction = 1;
      if (h->hall_state == 4) {
        h->multiturn_cnt++;
      }
    } else if (hall_reverse_sequence[h->prev_hall_state] == h->hall_state) {
      h->direction = -1;
      if (h->hall_state == 5) {
        h->multiturn_cnt--;
      }
    } else {
      h->direction = 0;
    }
  }

  if (h->is_ovf) {
    h->is_ovf = 0;
  } else {
    h->is_stop = 0;
  }

  // Calculate time interval (including timer overflow handling).
  uint16_t delta_time_ticks;
  delta_time_ticks = current_time_ticks;

  // Calculate angular velocity.
  if ((h->direction > 0) != (prev_direction > 0) || h->direction == 0 || prev_direction == 0) {
    h->is_stop = 1;
  }

  if (h->is_stop) {
    h->electrical_velocity_rad_s = 0.0;
  } else if (delta_time_ticks > 0) {
    h->electrical_velocity_rad_s =
      ((float)h->direction * PI_OVER_3_F) / ((float)delta_time_ticks * TIMER_TICK_S);
  } else {
    h->electrical_velocity_rad_s = 0.0f;
  }
  int pre_idx, now_idx = h->list_idx;
  pre_idx = (now_idx + N_POS_SAMPLES - 1) % N_POS_SAMPLES;
  h->electrical_velocity_rad_s =
    (h->multiturn_list[now_idx] - h->multiturn_list[pre_idx]) / (N_POS_SAMPLES * DT);

  // Update state.
  h->prev_hall_state = h->hall_state;
  h->last_capture_time_ticks = current_time_ticks;
  h->electrical_angle_rad = hall_state_angle_lut[h->hall_state];
}

void hall_sensor_get_estimate_angle(
  HallSensorStruct * h, uint16_t current_time_ticks, BasicEncoderStruct * res)
{
  // Calculate elapsed time since the last Hall event.
  uint32_t time_since_last_event_ticks = current_time_ticks;

  // Calculate interpolated offset from the last Hall angle.
  float angle_offset =
    h->electrical_velocity_rad_s * ((float)time_since_last_event_ticks * TIMER_TICK_S);
  if (current_time_ticks * TIMER_TICK_S > 1.0) {
    h->electrical_velocity_rad_s = 0.0;
    h->is_ovf = 1;
    h->is_stop = 1;
  } else if (angle_offset > PI_OVER_3_F || angle_offset < -PI_OVER_3_F) {
    h->electrical_velocity_rad_s = 0.0;
    h->is_stop = 1;
  }

  // Estimate current angle.
  float base_angle = hall_state_angle_lut[h->hall_state];
  float estimated_angle, single_turn_angle;

  if (h->is_stop || h->is_ovf) {
    estimated_angle = base_angle;
  } else {
    estimated_angle = base_angle + angle_offset;
    if (h->direction > 0) {
      estimated_angle -= PI_OVER_6_F;
    }
    if (h->direction < 0) {
      estimated_angle += PI_OVER_6_F;
    }
  }
  single_turn_angle = estimated_angle;
  // Normalize angle to the range 0 ~ 2*PI.
  float e_zero_pp = E_ZERO * TWO_PI_F * h->pole_pairs / ENC_CPR;

  estimated_angle -= e_zero_pp;

  estimated_angle = fmodf(estimated_angle, TWO_PI_F);
  if (estimated_angle < 0.0f) {
    estimated_angle += TWO_PI_F;
  }

  h->electrical_angle_rad = estimated_angle;
  res->elec_angle = h->electrical_angle_rad;
  res->elec_velocity = h->electrical_velocity_rad_s;

  float machine_single_turn =
    (single_turn_angle + TWO_PI_F * h->multiturn_cnt) / h->pole_pairs / TWO_PI_F;
  int machine_single_turn_int = (int)machine_single_turn;

  res->angle_singleturn_raw = (machine_single_turn - machine_single_turn_int) * TWO_PI_F;

  res->angle_singleturn = (machine_single_turn - machine_single_turn_int) * TWO_PI_F;
  res->angle_multiturn = (single_turn_angle + TWO_PI_F * h->multiturn_cnt) / h->pole_pairs;
  res->velocity = h->electrical_velocity_rad_s / h->pole_pairs;
  h->list_idx = (h->list_idx + 1) % N_POS_SAMPLES;
  h->multiturn_list[h->list_idx] = res->angle_multiturn * h->pole_pairs;
}

void hall_sensor_overflow(HallSensorStruct * h, BasicEncoderStruct * res)
{
  h->electrical_velocity_rad_s = 0.0;
  h->is_ovf = 1;
  h->is_stop = 1;

  // Estimate current angle.
  float base_angle = hall_state_angle_lut[h->hall_state];
  float estimated_angle, single_turn_angle;

  estimated_angle = base_angle;

  single_turn_angle = estimated_angle;
  // Normalize angle to the range 0 ~ 2*PI.
  float e_zero_pp = E_ZERO * TWO_PI_F * h->pole_pairs / ENC_CPR;

  estimated_angle -= e_zero_pp;

  estimated_angle = fmodf(estimated_angle, TWO_PI_F);
  if (estimated_angle < 0.0f) {
    estimated_angle += TWO_PI_F;
  }

  h->electrical_angle_rad = estimated_angle;
  res->elec_angle = h->electrical_angle_rad;
  res->elec_velocity = h->electrical_velocity_rad_s;

  float machine_single_turn = (single_turn_angle / TWO_PI_F + h->multiturn_cnt) / h->pole_pairs;
  int machine_single_turn_int = (int)machine_single_turn;

  res->angle_singleturn_raw = (machine_single_turn - machine_single_turn_int) * TWO_PI_F;

  res->angle_singleturn = (machine_single_turn - machine_single_turn_int) * TWO_PI_F;
  res->angle_multiturn = (single_turn_angle + TWO_PI_F * h->multiturn_cnt) / h->pole_pairs;
  res->velocity = h->electrical_velocity_rad_s / h->pole_pairs;
}
