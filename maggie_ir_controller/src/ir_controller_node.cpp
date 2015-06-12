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
    _action_pub = _nh_private.advertise<maggie_ir_controller_msgs::GetTvAction>("tv_action", 1);
    
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

    if (_ir_driver->send_remote_command(req.remote, req.command) != 0) {
        ROS_ERROR("[IR_CONTROLLER_NODE] Error sending remote command");
        return false;
    }

    // publish
    publish(req.command);

    return true;
}

//////////////////////////////////////////////////

void IRControllerNode::publish(std::string command)
{
    maggie_ir_controller_msgs::GetTvAction msg;

    msg.last_update = ros::Time::now();

    if (command == "OnOff") {
        // check if TV was ON or OFF
        if (_is_on) {
            msg.action = maggie_ir_controller_msgs::TvActions::OFF;
            _is_on = false;
        }
        else {
            msg.action = maggie_ir_controller_msgs::TvActions::ON;
            _is_on = true;
        }
    }
    else if (command == "v+") {
        msg.action = maggie_ir_controller_msgs::TvActions::VOL_UP;
    }
    else if (command == "v-") {
        msg.action = maggie_ir_controller_msgs::TvActions::VOL_DOWN;
    }
    else if (command == "up") {
        msg.action = maggie_ir_controller_msgs::TvActions::CH_UP;
    }
    else if (command == "down") {
        msg.action = maggie_ir_controller_msgs::TvActions::CH_DOWN;
    }
    else if (command == "DTV") {
        msg.action = maggie_ir_controller_msgs::TvActions::TV;
    }
    else if (command == "FM") {
        msg.action = maggie_ir_controller_msgs::TvActions::RADIO;
    }

    _action_pub.publish(msg);
}

//////////////////////////////////////////////////
