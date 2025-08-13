/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "encoder_struct.h"
#include "foc.h"
#include "hall_sensor.h"
#include "position_sensor.h"

#define V_CAL 1.0f  // Calibration voltage
// #define I_CAL	5.0f
#define W_CAL 5.0f  // Calibration speed in rad/s
#define T1 1.0f     // Cal settling period
#define PPAIRS_MAX 64
#define SAMPLES_PER_PPAIR N_LUT
#define N_CAL \
  SAMPLES_PER_PPAIR * PPAIRS_MAX  // Calibration lookup table maximum length, so I don't have to
                                  // deal with dynamic allocation based on number of pole pairs

typedef struct
{
  uint8_t ppairs;     // number of pole pairs measured
  int offset;         // electrical zero position in counts
  float theta_ref;    // reference angle used for calibration
  int start_count;    // loop count at cal start
  uint8_t started;    // has cal started or not?
  float time;         // cal time
  float theta_start;  // cal start angle
  int ezero;
  uint8_t phase_order;                       // phase order correct (0) or swapped (1)
  uint8_t done_ordering, done_cal, done_rl;  // flags for different cals finished
  uint16_t sample_count;                     // keep track of how many samples taken
  float next_sample_time;                    // time to take next sample
  int error_arr[N_CAL];
  int lut_arr[N_LUT];
  BasicEncoderStruct cal_position;  // Position reference used for calibration

} CalStruct;

void order_phases(
  BasicEncoderStruct * encoder, ControllerStruct * controller, CalStruct * cal, int loop_count);
void calibrate_abs_encoder(
  BasicEncoderStruct * encoder, AbsEncoderStruct * abs_encoder, ControllerStruct * controller,
  CalStruct * cal, int loop_count);
void calibrate_hall_encoder(
  BasicEncoderStruct * encoder, HallSensorStruct * hall_sensor, ControllerStruct * controller,
  CalStruct * cal, int loop_count);
void measure_lr(
  BasicEncoderStruct * encoder, ControllerStruct * controller, CalStruct * cal, int loop_count);

// extern int *error_array;
// extern int *lut_array;

#endif /* INC_CALIBRATION_H_ */
