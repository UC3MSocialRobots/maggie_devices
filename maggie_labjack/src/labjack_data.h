#ifndef __LABJACK_DATA_H__
#define __LABJACK_DATA_H__

/**
 * @file        labjack_data.h
 * @brief       Global data.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
 * @author      Alvaro Castro Gonzalez <acgonzal@ing.uc3m.es>
 * @date        2007-12
 *
 * @copyright   Copyright (C) 2015 University Carlos III of Madrid.
 *              All rights reserved.
 * @license     LEUC3M v1.0, see LICENSE.txt
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the Licencia Educativa UC3M as published by
 * the University Carlos III of Madrid, either version 1.0, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY. See the Licencia Educativa UC3M
 * version 1.0 or any later version for more details.
 *
 * A copy of the Licencia Educativa UC3M is in the LICENSE file.
 */

#include <touch_skill_msgs/MaggieTouchParts.h>

//////////////////////////////////////////////////
#define AI_VOLTAGE_BASE 0   // AI line to read the voltage of the battery
#define AI_PLUGGED      1   // AI line to detect whether battery is plugged or not
// value of the voltage divider on the analog input for batteries
#define GAIN_VOLT       3.57

//////////////////////////////////////////////////

#define D_WATCHDOG          0   // D line for watchdog
#define IO_ENABLE_BASE      2   // IO line to enable/disable motors of the base
#define IO_ENABLE_OTHERS    3   // IO line to enable/disable the other motors
#define IO_EMERGENCY        0   // IO line to read the emergency state
#define IO_SETA_SF          1   // IO line to activate the emergency button by software
//////////////////////////////////////////////////

#define TOUCHED         1
#define UNTOUCHED       0

#define HEAD            0x0002  // mask for line D1
#define LEFT_HAND       0x0004  // mask for line D2
#define LEFT_FOREARM    0x0008  // mask for line D3
#define LEFT_SHOULDER   0x0020  // mask for line D5
#define LEFT_BACK       0x0040  // mask for line D6
#define LEFT_SIDE       0x0080  // mask for line D7
#define RIGHT_FOREARM   0x0800  // mask for line D11
#define RIGHT_HAND      0x1000  // mask for line D12
#define RIGHT_SHOULDER  0x2000  // mask for line D13
#define RIGHT_BACK      0x4000  // mask for line D14
#define RIGHT_SIDE      0x8000  // mask for line D15
//////////////////////////////////////////////////

#define EMERGENCY       0x0001  // mask for line IO0
#define EMERGENCY_SF    0x0002  // mask for line IO1
#define ENABLE_BASE     0x0004  // mask for line IO2
#define ENABLE_BODY     0x0008  // mask for line IO3
//////////////////////////////////////////////////

// value for an DIO output to open/close a rele
#define ENABLE  1
#define DISABLE 0

//////////////////////////////////////////////////

// seconds for the watchdog
#define TIMEOUT_WD  2

//////////////////////////////////////////////////

#define NUM_LIN_AI          8
#define NUM_LIN_AO          2
#define NUM_LIN_IO          4
#define NUM_LIN_D           16
#define NUM_TOUCH_SENSORS   11

//////////////////////////////////////////////////

// labjack data structure
struct t_ljdata {
        long DsState;
        long IOsState;
        float AIsState[NUM_LIN_AI];
        long DsDirection;
};

struct t_pulseData {
        long level;
        long line;
        long microseconds;
};

struct touch {
        int sensor[touch_skill_msgs::MaggieTouchParts::NUMBER_SENSORS];
};

struct float8 {
        float v_float8[NUM_LIN_AI];
};

struct float2 {
        float v_float2[NUM_LIN_AO];
};

#endif
