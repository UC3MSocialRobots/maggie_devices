#ifndef __MCDC3006S_NODE_H__
#define __MCDC3006S_NODE_H__

/**
 * @file        motor_controller_node.h
 * @brief       Node for controlling the mcdc3006s motors (neck and arms).
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
 * @author      Javi F. Gorostiza <jgorosti@ing.uc3m.es>
 * @date        2008-07
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
#include "motor_controller_data.h"

// mcdc3006s driver
#include "mcdc3006s.h"

// messages and services
#include <motor_controller_msgs/Data.h>
#include <motor_controller_msgs/Odometry.h>
#include <motor_controller_msgs/Configuration.h>
#include <motor_controller_msgs/MoveAbsPos.h>
#include <motor_controller_msgs/Calibration.h>


class MotorControllerNode {
    public:
        /**
         * @brief Empty constructor.
         */
        MotorControllerNode(MotorDriverInterface *driver);

        /**
         * @brief Destructor.
         */
        ~MotorControllerNode();

        /**
         * @brief Initialize the mcdc3006s node.
         */
        void init();

        /**
         * @brief Spin the node.
         */
        void spin();

        //////////////////////////////////////////////////
        // publishers
        //////////////////////////////////////////////////

        /**
         * @brief Publish information from the motors.
         */
        void publish();

        //////////////////////////////////////////////////
        // services
        //////////////////////////////////////////////////

        /**
         * @brief Callback to set the maximum position of the motor.
         * @param req Configuration type request.
         * @param resp Configuration type response.
         * @return true if there are no errors.
         */
        bool set_max_pos(motor_controller_msgs::Configuration::Request & req,
                         motor_controller_msgs::Configuration::Response & resp);

        /**
         * @brief Callback to set the minimum position of the motor.
         * @param req Configuration type request.
         * @param resp Configuration type response.
         * @return true if there are no errors.
         */
        bool set_min_pos(motor_controller_msgs::Configuration::Request & req,
                         motor_controller_msgs::Configuration::Response & resp);

        /**
         * @brief Callback to set the maximum velocity of the motor.
         * @param req Configuration type request.
         * @param resp Configuration type response.
         * @return true if there are no errors.
         */
        bool set_max_vel(motor_controller_msgs::Configuration::Request & req,
                         motor_controller_msgs::Configuration::Response & resp);

        /**
         * @brief Callback to set the maximum acceleration of the motor.
         * @param req Configuration type request.
         * @param resp Configuration type response.
         * @return true if there are no errors.
         */
        bool set_max_acc(motor_controller_msgs::Configuration::Request & req,
                         motor_controller_msgs::Configuration::Response & resp);

        /**
         * @brief Callback to set the maximum deceleration of the motor.
         * @param req Configuration type request.
         * @param resp Configuration type response.
         * @return true if there are no errors.
         */
        bool set_max_dec(motor_controller_msgs::Configuration::Request & req,
                         motor_controller_msgs::Configuration::Response & resp);

        /**
         * @brief Callback to set the current limit of the motor.
         * @param req Configuration type request.
         * @param resp Configuration type response.
         * @return true if there are no errors.
         */
        bool set_cur_lim(motor_controller_msgs::Configuration::Request & req,
                         motor_controller_msgs::Configuration::Response & resp);

        /**
         * @brief Callback to move motors by absolute position.
         * @param req MoveAbsPos type request.
         * @param resp MoveAbsPos type response.
         * @return true if there are no errors.
         */
        bool move_abs_pos(motor_controller_msgs::MoveAbsPos::Request & req,
                          motor_controller_msgs::MoveAbsPos::Response & resp);

        /**
         * @brief Callback to move motors by relative position.
         * @param req MoveAbsPos type request.
         * @param resp MoveAbsPos type response.
         * @return true if there are no errors.
         */
        bool move_rel_pos(motor_controller_msgs::MoveAbsPos::Request & req,
                          motor_controller_msgs::MoveAbsPos::Response & resp);

        /**
         * @brief Callback to move motors in velocity.
         * @param req MoveAbsPos type request.
         * @param resp MoveAbsPos type response.
         * @return true if there are no errors.
         */
        bool move_vel(motor_controller_msgs::MoveAbsPos::Request & req,
                      motor_controller_msgs::MoveAbsPos::Response & resp);

		/**
         * @brief Callback to calibrate the joint.
         * @param req Calibration type request.
         * @param resp Calibration type response.
         * @return true if there are no errors.
         */
        bool joint_calibration(motor_controller_msgs::Calibration::Request & req,
						       motor_controller_msgs::Calibration::Response & resp);
                      
    private:
        // nodes
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        // publishers
        ros::Publisher _data_msg;
        ros::Publisher _odo_msg;

        // services
        ros::ServiceServer _max_pos_srv;
        ros::ServiceServer _min_pos_srv;
        ros::ServiceServer _max_vel_srv;
        ros::ServiceServer _max_acc_srv;
        ros::ServiceServer _max_dec_srv;
        ros::ServiceServer _cur_lim_srv;
        ros::ServiceServer _odo_srv;
        ros::ServiceServer _mov_abs_pos_srv;
        ros::ServiceServer _mov_rel_pos_srv;
        ros::ServiceServer _mov_vel_srv;
        ros::ServiceServer _calibration_srv;

        // spin rate
        ros::Rate _publish_rate;

        // ros params
        int _joint_id;
        std::string _serial_device, _joint_name;

        int _pos_factor, _vel_factor;
        float _calibration_home;

        MotorDriverInterface *_driver;
};

#endif
