#include "encoder_util.h"

void encoder_print(BasicEncoderStruct * encoder, int dt_ms){
	// printf("Raw: %d", encoder->raw);
	// printf("   Linearized Count: %d", encoder->count);
	printf("   Single Turn: %f", encoder->angle_singleturn);
	printf("   Multiturn: %f", encoder->angle_multiturn);
	printf("   Electrical: %f\n\r", encoder->elec_angle);
	// printf("   Turns:  %d\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}