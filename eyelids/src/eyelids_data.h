#ifndef __EYELIDS_DATA_H__
#define __EYELIDS_DATA_H__

/**
 * @file        eyelids_data.h
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

// communication
#define BAUDRATE    B9600
#define BAUDS       0xFF

//////////////////////////////////////////////////

// limits
#define LEFT_EYELID_MIN		    5
#define LEFT_EYELID_MAX		    140
#define LEFT_EYELID_NORMAL		100

#define RIGHT_EYELID_MIN	    220
#define RIGHT_EYELID_MAX	    80
#define RIGHT_EYELID_NORMAL 	127

//////////////////////////////////////////////////

// servomotor definitions
#define SERVO_LEFT  0x01
#define SERVO_RIGHT 0x00

//////////////////////////////////////////////////

// eyes id
#define LEFT        "left"
#define RIGHT       "right"

#endif
