/**
 * @file        motor_controller_node.cpp
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

#include "motor_controller_node.h"

//////////////////////////////////////////////////

MotorControllerNode::MotorControllerNode(MotorDriverInterface *driver) :
    _nh_private("~"),
    _publish_rate(10),
    _driver(driver)
{
    // get ros params

    std::string searched_param;
    std::string port;

    // find the dev port name
    _nh_private.searchParam("port", searched_param);
    _nh_private.param(searched_param, port, std::string());

    // find the joint name
    _nh_private.searchParam("joint_name", searched_param);
    _nh_private.param(searched_param, _joint_name, std::string());

    // find the joint id
    _nh_private.searchParam("joint_id", searched_param);
    _nh_private.param(searched_param, _joint_id, -1);

    // set full dev name
    _serial_device = "/dev/" + port;

    // set joint parameters

    int error = -1;
    std::string sem_file("/");

    if (_joint_name == NECK) {
        // neck
        switch(_joint_id) {
            case NECK_HOR:
                sem_file += HOR_NECK_SEMAPHORE_FILE;

                _pos_factor = TOTAL_NECK_HOR_REDUCTION;
                _vel_factor = NECK_HOR_REDUCTION_FACTOR;
                _calibration_home = NECK_HOR_HOME_POS;
                _tmp_min_pos = NECK_HOR_TMP_MIN_POS;
				_min_pos_after_calib = NECK_HOR_MIN_POS;
				_max_pos_after_calib = NECK_HOR_MAX_POS;
				
                ROS_INFO("[MOTOR_CONTROLLER] Neck: Horizontal joint chosen. factor_position = %d\n", _pos_factor);
                break;

            case NECK_VER:
                sem_file += VER_NECK_SEMAPHORE_FILE;

                _pos_factor = TOTAL_NECK_VER_REDUCTION;
                _vel_factor = NECK_VER_REDUCTION_FACTOR;
                _calibration_home = NECK_VER_HOME_POS;
                _tmp_min_pos = NECK_VER_TMP_MIN_POS;
                _min_pos_after_calib = NECK_VER_MIN_POS;
				_max_pos_after_calib = NECK_VER_MAX_POS;

                ROS_INFO("[MOTOR_CONTROLLER] Neck: Vertical joint chosen. factor_position = %d\n", _pos_factor);
                break;

            default:
                ROS_WARN("[MOTOR_CONTROLLER] You must select the axis of the neck.\n");
                exit(EXIT_FAILURE);
        }
    }
    else if (_joint_name == ARM) {
        // arms
        switch(_joint_id) {
            case ARM_LEFT:
                sem_file += LEFT_ARM_SEMAPHORE_FILE;

                ROS_INFO("[MOTOR_CONTROLLER] Arm: left joint chosen");
                break;

            case ARM_RIGHT:
                sem_file += RIGHT_ARM_SEMAPHORE_FILE;

                ROS_INFO("[MOTOR_CONTROLLER] Arm: right joint chosen");
                break;

            default:
                ROS_WARN("[MOTOR_CONTROLLER] You must select the correct arm id.\n");
                exit(EXIT_FAILURE);
        }
    }
    else {
        ROS_ERROR("[MOTOR_CONTROLLER] Error: no correct joint name param specified. Get: '%s', Expected: '%s' or '%s'", _joint_name.c_str(), NECK, ARM);
        exit(-1);
    }

    // trick for strings writable
    std::vector<char> wr_serial_device(_serial_device.begin(), _serial_device.end());
    wr_serial_device.push_back('\0');
    std::vector<char> wr_sem_file(sem_file.begin(), sem_file.end());
    wr_sem_file.push_back('\0');

    // init the communication
    if ((error = _driver->init(BAUDRATE, &wr_serial_device[0], &wr_sem_file[0])) != 0) {
        exit(error);
    }
}

//////////////////////////////////////////////////

MotorControllerNode::~MotorControllerNode()
{
    _driver->disable_driver();
}

//////////////////////////////////////////////////

void MotorControllerNode::init()
{
    _data_msg = _nh_private.advertise<maggie_motor_controller_msgs::Data>("data_pub", 100);
    _odo_msg = _nh_private.advertise<maggie_motor_controller_msgs::Odometry>("odo_pub", 100);

    _max_pos_srv = _nh_private.advertiseService("set_max_pos", &MotorControllerNode::set_max_pos, this);
    _min_pos_srv = _nh_private.advertiseService("set_min_pos", &MotorControllerNode::set_min_pos, this);
    _max_vel_srv = _nh_private.advertiseService("set_max_vel", &MotorControllerNode::set_max_vel, this);
    _max_acc_srv = _nh_private.advertiseService("set_max_acc", &MotorControllerNode::set_max_acc, this);
    _max_dec_srv = _nh_private.advertiseService("set_max_dec", &MotorControllerNode::set_max_dec, this);
    _cur_lim_srv = _nh_private.advertiseService("set_cur_lim", &MotorControllerNode::set_cur_lim, this);

    _mov_abs_pos_srv = _nh_private.advertiseService("mov_abs_pos", &MotorControllerNode::move_abs_pos, this);
    _mov_rel_pos_srv = _nh_private.advertiseService("mov_rel_pos", &MotorControllerNode::move_rel_pos, this);
    _mov_vel_srv = _nh_private.advertiseService("mov_vel", &MotorControllerNode::move_vel, this);

    _calibration_srv = _nh_private.advertiseService("joint_calibration", &MotorControllerNode::joint_calibration, this);

    // driver enable
    _driver->enable_driver();
}

//////////////////////////////////////////////////

void MotorControllerNode::spin()
{
    while(_nh.ok()) {
        ros::spinOnce();
        _publish_rate.sleep();

        // publish motors information
        publish();
    }
}

//////////////////////////////////////////////////

void MotorControllerNode::publish()
{
    maggie_motor_controller_msgs::Data msg_data;
    maggie_motor_controller_msgs::Odometry msg_odo;
    double factor;
    double joint_factor;

    // publish general data

    // get the factor
    joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    factor = (2. * M_PI) / joint_factor;

    // fill the message with positions
    msg_data.max_pos = _driver->get_max_pos() * factor;
    msg_data.min_pos = _driver->get_min_pos() * factor;

    if (_joint_name == NECK) {
        factor = (2. * M_PI) / _vel_factor / 60.;
    }
    else {
        factor = (2. * M_PI) * PULSES_PER_REV / ARMS_REDUCTION_FACTOR / 60. / 60.;
    }

    // fill the message with the rest of data
    msg_data.max_vel = _driver->get_max_vel() * factor;
    msg_data.max_acc = _driver->get_max_acc() * factor;
    msg_data.max_dec = _driver->get_max_dec() * factor;
    msg_data.max_cur_lim = _driver->get_cur_lim();

    // publish general data
    _data_msg.publish(msg_data);

    // publish odometry data

    double pos_factor, vel_factor;
    driverSensor_t dOdo;

    // get factors
    joint_factor = (_joint_name == NECK ? _pos_factor : ARMS_REDUCTION_FACTOR);
    pos_factor = (2. * M_PI) / joint_factor;

    joint_factor = (_joint_name == NECK ? _vel_factor : (PULSES_PER_REV / ARMS_REDUCTION_FACTOR));
    vel_factor = (2. * M_PI) / joint_factor / 60.;

    // get odometry information from motor driver
    _driver->get_sensor(&dOdo);

    // fill the message
    msg_odo.position = dOdo.p * pos_factor;
    msg_odo.velocity = dOdo.v * vel_factor;
    msg_odo.current = dOdo.i;

    // publish general data
    _odo_msg.publish(msg_odo);
}

//////////////////////////////////////////////////

bool MotorControllerNode::set_max_pos(maggie_motor_controller_msgs::Configuration::Request & req,
                                      maggie_motor_controller_msgs::Configuration::Response & resp)
{
    /* from rad of the DOF to pulses of the engine */
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    _driver->set_max_pos(req.max_pos * factor);

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::set_min_pos(maggie_motor_controller_msgs::Configuration::Request & req,
                                      maggie_motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    _driver->set_min_pos(req.min_pos * factor);

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::set_max_vel(maggie_motor_controller_msgs::Configuration::Request & req,
                                      maggie_motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _vel_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor * 60. / (2. * M_PI);

    _driver->set_max_vel(req.max_vel * factor);

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::set_max_acc(maggie_motor_controller_msgs::Configuration::Request & req,
                                      maggie_motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    _driver->set_max_acc(req.max_acc * factor);

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::set_max_dec(maggie_motor_controller_msgs::Configuration::Request & req,
                                      maggie_motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    _driver->set_max_dec(req.max_dec * factor);

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::set_cur_lim(maggie_motor_controller_msgs::Configuration::Request & req,
                                      maggie_motor_controller_msgs::Configuration::Response & resp)
{
    _driver->set_cur_lim(req.cur);

    return true;
}

////////////////////////
// moves
////////////////////////

bool MotorControllerNode::move_abs_pos(maggie_motor_controller_msgs::MoveAbsPos::Request & req,
                                       maggie_motor_controller_msgs::MoveAbsPos::Response & resp)
{
    ROS_DEBUG("[MOTOR_CONTROLLER] moveAbsPos pos = %f\n", req.position);

    long int joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    long int factor = joint_factor / (2. * M_PI);

    _driver->move_abs_pos(int(req.position * factor));

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::move_rel_pos(maggie_motor_controller_msgs::MoveAbsPos::Request & req,
                                       maggie_motor_controller_msgs::MoveAbsPos::Response & resp)
{
    ROS_DEBUG("[MOTOR_CONTROLLER] moveRelPos pos = %f\n", req.position);

    long int joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    long int factor = joint_factor / (2. * M_PI);

    _driver->move_rel_pos(int(req.position * factor));

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::move_vel(maggie_motor_controller_msgs::MoveAbsPos::Request & req,
                                   maggie_motor_controller_msgs::MoveAbsPos::Response & resp)
{
    ROS_DEBUG("[MOTOR_CONTROLLER] moveVel vel = %f\n", req.position);

    // change velocity from rad/sec to rpm
    long int joint_factor = (_joint_name == NECK ? _vel_factor : TOTAL_ARMS_REDUCTION / PULSES_PER_REV);
    long int factor = joint_factor * 60. / (2. * M_PI);

    _driver->move_vel(int(req.position * factor));

    return true;
}

//////////////////////////////////////////////////

bool MotorControllerNode::joint_calibration(maggie_motor_controller_msgs::MoveAbsPos::Request & req,
                                            maggie_motor_controller_msgs::MoveAbsPos::Response & resp)
{
    ROS_DEBUG("[MOTOR_CONTROLLER] joint calibration\n");

    long int joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    long int factor = joint_factor / (2. * M_PI);
	
	_driver->set_min_pos(_tmp_min_pos * factor);
  	_driver->set_max_pos(_tmp_max_pos * factor);

    _driver->calibrate(int(_calibration_home * factor));
	
	_driver->set_min_pos(_min_pos_after_calib * factor);
	_driver->set_max_pos(_max_pos_after_calib * factor);
	
    return true;
}

//////////////////////////////////////////////////

