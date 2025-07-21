#ifndef ENCODER_STRUCT_H_
#define ENCODER_STRUCT_H_

#include <stdint.h>

#define ENCODER_ABS 0
#define ENCODER_HALL 1

typedef struct
{
  float angle_singleturn, angle_singleturn_raw, angle_multiturn, elec_angle, velocity, elec_velocity;
} BasicEncoderStruct;

#endif /* ENCODER_STRUCT_H_ */