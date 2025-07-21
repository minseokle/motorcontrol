/*
 * structs.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#ifndef STRUCTS_H
#define STRUCTS_H




#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "encoder_struct.h"
#include "position_sensor.h"
#include "hall_sensor.h"
#include "preference_writer.h"
#include "fsm.h"
#include "drv8323.h"
#include "foc.h"
#include "calibration.h"
#include "can.h"

typedef struct{
    } GPIOStruct;

typedef struct{
    }COMStruct;


/* Global Structs */
extern ControllerStruct controller;
extern ObserverStruct observer;
extern COMStruct com;
extern FSMStruct state;
extern BasicEncoderStruct basic_encoder;
extern AbsEncoderStruct abs_encoder;
extern HallSensorStruct hall_sensor;
extern DRVStruct drv;
extern PreferenceWriter prefs;
extern CalStruct comm_encoder_cal;
extern CANTxMessage can_tx;
extern CANRxMessage can_rx;

#endif
