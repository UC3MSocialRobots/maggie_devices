#ifndef __MCDC3006S_DATA_H__
#define __MCDC3006S_DATA_H__

/**
 * @file        mcdc3006s_data.h
 * @brief       Global data.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
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

//////////////////////////////////////////////////

#define BAUDRATE                    19200

//////////////////////////////////////////////////

// motors data
#define PULSES_PER_REV              2048.

// reduction factor in horizontal and vertical movement engine
#define NECK_VER_REDUCTION_FACTOR   178.
#define NECK_HOR_REDUCTION_FACTOR   111.

#define ARMS_REDUCTION_FACTOR       166.

// rev = pulses_per_rev * axis_reduction_factor
#define TOTAL_NECK_VER_REDUCTION    (PULSES_PER_REV * NECK_VER_REDUCTION_FACTOR)
#define TOTAL_NECK_HOR_REDUCTION    (PULSES_PER_REV * NECK_HOR_REDUCTION_FACTOR)

#define TOTAL_ARMS_REDUCTION        (PULSES_PER_REV * ARMS_REDUCTION_FACTOR)

// home position of the motors from the limit swtich after the calibration in rads
#define NECK_HOR_HOME_POS	 -1.6
#define NECK_VER_HOME_POS	 -1

//////////////////////////////////////////////////

// semaphores
#define HOR_NECK_SEMAPHORE_FILE     "tmp/horNeckSemaphore"
#define VER_NECK_SEMAPHORE_FILE     "tmp/verNeckSemaphore"

#define LEFT_ARM_SEMAPHORE_FILE     "tmp/leftArmSemaphore"
#define RIGHT_ARM_SEMAPHORE_FILE    "tmp/rightArmSemaphore"

//////////////////////////////////////////////////

// joints data
enum neck_id {
    NECK_HOR = 0, NECK_VER
};
enum arm_id {
    ARM_LEFT = 0, ARM_RIGHT
};

//////////////////////////////////////////////////

#define NECK                       "neck"
#define ARM                        "arm"

//////////////////////////////////////////////////

#define TEST_DEVICE	                0

//////////////////////////////////////////////////

typedef struct odometry {
        // current position in rad
        double pos;
        // current velocity in rad/second
        double vel;
        // instant current (mA)
        int cur;
} odo_t;

#endif
