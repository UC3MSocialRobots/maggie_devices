#ifndef __IR_CONTROLLER_NODE_H__
#define __IR_CONTROLLER_NODE_H__

/**
 * @file        ir_controller_node.h
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

#include <ros/ros.h>
#include "ir_driver_interface.h"

// messages and services
#include <maggie_ir_controller_msgs/GetTvAction.h>
#include <maggie_ir_controller_msgs/TvActions.h>
#include <maggie_ir_controller_msgs/SetTvAction.h>

class IRControllerNode {
    public:
        /**
         * @brief Parameterized constructor.
         * @param ir_driver, the driver interface.
         */
        IRControllerNode(IrDriverInterface *ir_driver);

        /**
         * @brief Destructor.
         */
        ~IRControllerNode();

        /**
         * @brief Initialize the node.
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
        // publishers
        //////////////////////////////////////////////////

        /**
         * @brief Publish the tv action.
         */
        void publish(std::string command);

        //////////////////////////////////////////////////
        // services
        //////////////////////////////////////////////////

        /**
         * @brief Callback to manage the controller.
         * @param req SetTvActions type request.
         * @param resp SetTvActions type response.
         * @return true if there are no errors.
         */
        bool send_command(maggie_ir_controller_msgs::SetTvAction::Request & req,
                          maggie_ir_controller_msgs::SetTvAction::Response & resp);

    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        ros::Publisher _action_pub;

        ros::ServiceServer _action_srv;

        ros::Rate _publish_rate;

        IrDriverInterface *_ir_driver;

        bool _is_on;
};

#endif
