/**
 * @file        ir_controller_node.cpp
 * @brief       Node for controlling the IR controller for the TV.
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

#include "ir_controller_node.h"

//////////////////////////////////////////////////

IRControllerNode::IRControllerNode(IrDriverInterface *ir_driver) :
    _nh_private("~"),
    _publish_rate(10),
    _ir_driver(ir_driver),
    _is_on(false)
{
    // connect with the driver
    if (_ir_driver->connect() != 0) {
        ROS_ERROR("[IR_CONTROLLER_NODE] Error connecting to driver");
    }
}

//////////////////////////////////////////////////

IRControllerNode::~IRControllerNode()
{
    // disconnect the driver
    _ir_driver->disconnect();
}

//////////////////////////////////////////////////

void IRControllerNode::init()
{
    _action_srv = _nh_private.advertiseService("send_command", &IRControllerNode::send_command, this);
}

//////////////////////////////////////////////////

void IRControllerNode::spin()
{
    while(_nh.ok()) {
        ros::spinOnce();
        _publish_rate.sleep();
    }
}

//////////////////////////////////////////////////

bool IRControllerNode::send_command(maggie_ir_controller_msgs::SetTvAction::Request & req,
                                    maggie_ir_controller_msgs::SetTvAction::Response & resp)
{
    ROS_DEBUG("[IR_CONTROLLER_NODE] Sending IR command...");

    std::string remote = "TVlabo";
    std::string command = "";

    // select the command in function of the string received
    if (req.command == maggie_ir_controller_msgs::TvActions::ON
        || req.command == maggie_ir_controller_msgs::TvActions::OFF) {
        command = "OnOff";
    }
    else if (req.command == maggie_ir_controller_msgs::TvActions::VOL_UP) {
        command = "v+";
    }
    else if (req.command == maggie_ir_controller_msgs::TvActions::VOL_DOWN) {
        command = "v-";
    }
    else if (req.command == maggie_ir_controller_msgs::TvActions::CH_UP) {
        command = "up";
    }
    else if (req.command == maggie_ir_controller_msgs::TvActions::CH_DOWN) {
        command = "down";
    }
    else if (req.command == maggie_ir_controller_msgs::TvActions::TV) {
        command = "DTV";
    }
    else if (req.command == maggie_ir_controller_msgs::TvActions::RADIO) {
        command = "FM";
    }

    // set command
    if (_ir_driver->send_remote_command(remote, command) != 0) {
        ROS_ERROR("[IR_CONTROLLER_NODE] Error sending remote command");
        return false;
    }

    return true;
}

//////////////////////////////////////////////////
