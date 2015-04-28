#ifndef __EYELIDS_NODE_H__
#define __EYELIDS_NODE_H__

/**
 * @file        eyelids_node.h
 * @brief       Node for controlling the servomotors of the eyelids.
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

#include <termios.h>
#include <ros/ros.h>
#include <serial_communication_interface.h>
#include "eyelids_data.h"

// messages and services
#include <maggie_eyelids_msgs/EyelidPositions.h>
#include <maggie_eyelids_msgs/MoveStrPos.h>
#include <maggie_motor_controller_msgs/MoveAbsPos.h>

class EyelidsNode {
    public:
        /**
         * @brief Parameterized constructor.
         * @param serial_comm, the driver interface.
         */
        EyelidsNode(SerialCommunicationInterface *serial_comm);

        /**
         * @brief Destructor.
         */
        ~EyelidsNode();

        /**
         * @brief Initialize the eyelids node.
         * @param
         * @return
         */
        void init();

        /**
         * @brief Spin the node.
         * @param
         * @return
         */
        void spin();

        //////////////////////////////////////////////////
        // services
        //////////////////////////////////////////////////

        /**
         * @brief Callback to move motors by absolute position.
         * @param req MoveAbsPos type request.
         * @param resp MoveAbsPos type response.
         * @return true if there are no errors.
         */
        bool move_abs_pos(maggie_motor_controller_msgs::MoveAbsPos::Request & req, maggie_motor_controller_msgs::MoveAbsPos::Response & resp);

        /**
         * @brief Callback to move motors by string name position.
         * @param req MoveStrPos type request.
         * @param resp MoveStrPos type response.
         * @return true if there are no errors.
         */
        bool move_str_pos(maggie_eyelids_msgs::MoveStrPos::Request & req, maggie_eyelids_msgs::MoveStrPos::Response & resp);

        //////////////////////////////////////////////////
        // methods
        //////////////////////////////////////////////////

        /**
         * @brief Method to move the servomotors.
         * @param position, absolute position with values in the range between 0 and 254.
         * @return error: -1 or successful: 0.
         */
        int move(int position);

    private:
        // nodes
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        // publishers
        ros::Publisher _get_position_msg;

        // services
        ros::ServiceServer _move_abs_pos_srv;
        ros::ServiceServer _move_str_pos_srv;

        // spin rate
        ros::Rate _publish_rate;

        // serial communication
        SerialCommunicationInterface *_serial_comm;
        struct termios _oldtio;

        // left or right eye
        std::string _side;
};

#endif
