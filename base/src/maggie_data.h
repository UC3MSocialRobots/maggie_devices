/**
 * @file        base_motor_node.cpp
 * @brief       Specific definitions for Maggie.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
 * @author      Rafael Rivas Estrada <rafael@ula.ve>
 * @date        2006-05
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

//TODO: convert this to rosparam

// max revolution speed of the motor (RPM = revolutions / min)
#define VELOCITY_MAX            "SP2500\n\r"

// reduction factor (engine revs / 1 wheel spin)
#define REDUCTION               36.0

// pulses per reduction
#define PULSES_PER_REVOLUTION   2000.0

// maximum linear velocity (mm / sec)
#define LINEAL_VELOCITY_MAX     858.17

// maximum angular velocity (degrees / sec), where 360 degrees = 1 revolution
#define ANGULAR_VELOCITY_MAX    360.0

// length of the axe between wheels (mm)
#define AXIS_LENGTH             307.0

// wheels diameter (mm)
#define WHEEL_DIAMETER          156.0

// parameters for the motor drivers

// acceleration ramp for accelerating (sec^-2)
#define ACCELERATION            "AC20\n\r"

// acceleration ramp for decelerating (sec^-2)
#define DECELERATION            "DEC25\n\r"
