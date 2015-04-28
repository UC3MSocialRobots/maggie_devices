/**
 * @file        test_manual_ir_controller.cpp
 * @brief       Manual tests.
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

#include <iostream>
#include <ros/ros.h>

// service
#include <maggie_ir_controller_msgs/SetTvAction.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_manual_ir_controller");

    string remote("");
    string command("");

    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<maggie_ir_controller_msgs::SetTvAction>("ir_controller/ir_event_manager");
    maggie_ir_controller_msgs::SetTvAction srv;

    do {
        cout << "Select remote (TVLabo or Quit): ";
        cin >> remote;

        cout << "Select command (OnOff, up, down, v+, v-): ";
        cin >> command;

        // Send IR command
        srv.request.remote = remote;
        srv.request.command = command;

        if (!client.call(srv)) {
            ROS_ERROR("[IR_CONTROLLER] Error: calling service ir_event_manager");
            return 1;
        }
    }
    while(remote != "Quit");

    return 0;
}
