#ifndef __BASE_MOTOR_NODE_H__
#define __BASE_MOTOR_NODE_H__

/**
 * @file        base_motor_node.h
 * @brief       Node for the motors of the mobile base.
 *
 * Important data:
 *      A base controller that complies with:
 *      http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/#Base_Controller_.28base_controller.29
 *
 *      An odometry publisher that complies with:
 *      http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/#Odometry_Information_.28odometry_source.29
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

#include "base_motor_driver.h"
#include <tf/transform_broadcaster.h>

// messages
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class BaseMotorNode {
    public:
        /**
         * @brief Empty constructor.
         */
        BaseMotorNode();

        /**
         * @brief Destructor.
         */
        ~BaseMotorNode();

        /**
         * @brief Initialize the labjack node.
         */
        void init();

        /**
         * @brief Spin the node.
         */
        void spin();

        //////////////////////////////////////////////////
        // publishers and subscribers
        //////////////////////////////////////////////////

        /**
         * @brief launch the publication of odometry
         * @param
         * @return
         */
        void publish();

        /**
         * @brief
         * @param
         * @return
         */
        void twist_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel);

    private:
        // nodes
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        // publishers and subscribers
        ros::Publisher odom_pub;
        ros::Subscriber sub;

        // spin rate
        ros::Rate _publish_rate;

        ros::Time _current_time, _last_time;

        tf::TransformBroadcaster odom_broadcaster;

        BaseMotor _bm;
};

#endif
