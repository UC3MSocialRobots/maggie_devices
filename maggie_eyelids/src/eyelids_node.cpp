/**
 * @file        eyelids_node.cpp
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

#include "eyelids_node.h"

//////////////////////////////////////////////////

EyelidsNode::EyelidsNode(SerialCommunicationInterface *serial_comm) :
    _nh_private("~"),
    _publish_rate(10),
    _serial_comm(serial_comm)
{
    // get ros params
    std::string searched_param;
    std::string port;

    // find the dev port name
    _nh_private.searchParam("port", searched_param);
    _nh_private.param(searched_param, port, std::string());

    // find the eye name
    _nh_private.searchParam("side", searched_param);
    _nh_private.param(searched_param, _side, std::string());

    // set full dev name
    _serial_comm->set_serial_device("/dev/" + port);

    _serial_comm->open_port(&_oldtio);

    // move eyelid to normal position
    if (_side == LEFT) {
        move(LEFT_EYELID_NORMAL);
    }
    else if (_side == RIGHT) {
        move(RIGHT_EYELID_NORMAL);
    }
}

//////////////////////////////////////////////////

EyelidsNode::~EyelidsNode()
{
    // move eyelid to normal position
    if (_side == LEFT) {
        move(LEFT_EYELID_NORMAL);
    }
    else if (_side == RIGHT) {
        move(RIGHT_EYELID_NORMAL);
    }

    _serial_comm->close_port(&_oldtio);
}

//////////////////////////////////////////////////

void EyelidsNode::init()
{
    _move_abs_pos_srv = _nh_private.advertiseService("move_abs_pos", &EyelidsNode::move_abs_pos, this);
    _move_str_pos_srv = _nh_private.advertiseService("move_str_pos", &EyelidsNode::move_str_pos, this);
}

//////////////////////////////////////////////////

void EyelidsNode::spin()
{
    while(_nh.ok()) {
        ros::spinOnce();
        _publish_rate.sleep();
    }
}

//////////////////////////////////////////////////

bool EyelidsNode::move_abs_pos(motor_controller_msgs::MoveAbsPos::Request & req,
                               motor_controller_msgs::MoveAbsPos::Response & resp)
{
    if (move(req.position) == -1) {
        return false;
    }

    return true;
}

//////////////////////////////////////////////////

bool EyelidsNode::move_str_pos(eyelids_msgs::MoveStrPos::Request & req, eyelids_msgs::MoveStrPos::Response & resp)
{
    if (req.position == "open") {
        if (_side == LEFT) {
            move(LEFT_EYELID_MAX);
        }
        else if (_side == RIGHT) {
            move(RIGHT_EYELID_MAX);
        }
    }
    else if (req.position == "close") {
        if (_side == LEFT) {
            move(LEFT_EYELID_MIN);
        }
        else if (_side == RIGHT) {
            move(RIGHT_EYELID_MIN);
        }
    }
    else if (req.position == "normal") {
        if (_side == LEFT) {
            move(LEFT_EYELID_NORMAL);
        }
        else if (_side == RIGHT) {
            move(RIGHT_EYELID_NORMAL);
        }
    }
    else {
        ROS_ERROR("[EYELIDS] Error: wrong string position (%s), not available", std::string(req.position).c_str());
        return false;
    }

    return true;
}

//////////////////////////////////////////////////

int EyelidsNode::move(int position)
{
    unsigned char bauds = BAUDS, motor_id, movement;
    int error = 0;

    if (_side == LEFT) {
        motor_id = SERVO_LEFT;
    }
    else if (_side == RIGHT) {
        motor_id = SERVO_RIGHT;
    }

    // it must be sended like a char, not an integer
    movement = (unsigned char) position;

    if (_serial_comm->send_character(&bauds) != 0) {
        ROS_ERROR("[EYELIDS] Error: sending bauds character");
        error = -1;
    }
    if (_serial_comm->send_character(&motor_id) != 0) {
        ROS_ERROR("[EYELIDS] Error: sending motor_id character");
        error = -1;
    }
    if (_serial_comm->send_character(&movement) != 0) {
        ROS_ERROR("[EYELIDS] Error: sending movement character");
        error = -1;
    }
    ROS_DEBUG("[EYELIDS] Command sent");

    return error;
}

////////////////////////////////////////////////////
