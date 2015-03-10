/**
 * @file        mcdc3006s_node.cpp
 * @brief       Node for controlling the mcdc3006s motors (neck and arms).
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
 * @author      Javi F. Gorostiza <jgorosti@ing.uc3m.es<
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

Mcdc3006sNode::Mcdc3006sNode() :
    _nh_private("~"),
    _publish_rate(10),
    _RSd(-1)
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

                ROS_INFO("[MOTOR_CONTROLLER] Neck: Horizontal joint chosen. factor_position = %d\n", _pos_factor);
                break;

            case NECK_VER:
                sem_file += VER_NECK_SEMAPHORE_FILE;

                _pos_factor = TOTAL_NECK_VER_REDUCTION;
                _vel_factor = NECK_VER_REDUCTION_FACTOR;

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
    if ((error = initCommunication(BAUDRATE, &wr_serial_device[0], &_RSd, &wr_sem_file[0], &_semID)) < 0) {
        ROS_ERROR("[MOTOR_CONTROLLER] initCommunication with devices failed");
        ROS_WARN("[MOTOR_CONTROLLER] baudrate=%d, RSd=%d, semID=%d, serialDevice=%s, semFile=%s", BAUDRATE, _RSd, _semID, _serial_device.c_str(), sem_file.c_str());
        ROS_ERROR("[MOTOR_CONTROLLER] Error : %d\tRSd : %d", error, _RSd);
        exit(error);
    }
}

//////////////////////////////////////////////////

Mcdc3006sNode::~Mcdc3006sNode()
{
    disableDriver(_RSd, _semID);
    endCommunication(_RSd, _semID);
}

//////////////////////////////////////////////////

void Mcdc3006sNode::init()
{
    _data_msg = _nh_private.advertise<motor_controller_msgs::Data>("data_pub", 100);
    _odo_msg = _nh_private.advertise<motor_controller_msgs::Odometry>("odo_pub", 100);

    _max_pos_srv = _nh_private.advertiseService("set_max_pos", &Mcdc3006sNode::set_max_pos, this);
    _min_pos_srv = _nh_private.advertiseService("set_min_pos", &Mcdc3006sNode::set_min_pos, this);
    _max_vel_srv = _nh_private.advertiseService("set_max_vel", &Mcdc3006sNode::set_max_vel, this);
    _max_acc_srv = _nh_private.advertiseService("set_max_acc", &Mcdc3006sNode::set_max_acc, this);
    _max_dec_srv = _nh_private.advertiseService("set_max_dec", &Mcdc3006sNode::set_max_dec, this);
    _cur_lim_srv = _nh_private.advertiseService("set_cur_lim", &Mcdc3006sNode::set_cur_lim, this);
    _odo_srv = _nh_private.advertiseService("set_odo", &Mcdc3006sNode::set_odometry_calibration, this);

    _mov_abs_pos_srv = _nh_private.advertiseService("mov_abs_pos", &Mcdc3006sNode::move_abs_pos, this);
    _mov_rel_pos_srv = _nh_private.advertiseService("mov_rel_pos", &Mcdc3006sNode::move_rel_pos, this);
    _mov_vel_srv = _nh_private.advertiseService("mov_vel", &Mcdc3006sNode::move_vel, this);

    // driver enable
    enableDriver(_RSd, _semID);
}

//////////////////////////////////////////////////

void Mcdc3006sNode::spin()
{
    while(_nh.ok()) {
        ros::spinOnce();
        _publish_rate.sleep();

        // publish motors information
        publish();
    }
}

//////////////////////////////////////////////////

void Mcdc3006sNode::publish()
{
    motor_controller_msgs::Data msg_data;
    motor_controller_msgs::Odometry msg_odo;
    double factor;
    double joint_factor;

    // publish general data

    // get the factor
    joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    factor = (2. * M_PI) / joint_factor;

    // fill the message with positions
    msg_data.max_pos = getDriverMaxPos(_RSd, _semID) * factor;
    msg_data.min_pos = getDriverMinPos(_RSd, _semID) * factor;

    if (_joint_name == NECK) {
        factor = (2. * M_PI) / _vel_factor / 60.;
    }
    else {
        factor = (2. * M_PI) * PULSES_PER_REV / ARMS_REDUCTION_FACTOR / 60. / 60.;
    }

    // fill the message with the rest of data
    msg_data.max_vel = getDriverMaxVel(_RSd, _semID) * factor;
    msg_data.max_acc = getDriverMaxAcc(_RSd, _semID) * factor;
    msg_data.max_dec = getDriverMaxDec(_RSd, _semID) * factor;
    msg_data.max_cur_lim = getDriverCurLim(_RSd, _semID);

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
    getDriverSensor(_RSd, _semID, &dOdo);

    // fill the message
    msg_odo.position = dOdo.p * pos_factor;
    msg_odo.velocity = dOdo.v * vel_factor;
    msg_odo.current = dOdo.i;

    // publish general data
    _odo_msg.publish(msg_odo);
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::set_max_pos(motor_controller_msgs::Configuration::Request & req,
                                motor_controller_msgs::Configuration::Response & resp)
{
    /* from rad of the DOF to pulses of the engine */
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    setDriverMaxPos(_RSd, _semID, req.max_pos * factor);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::set_min_pos(motor_controller_msgs::Configuration::Request & req,
                                motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    setDriverMinPos(_RSd, _semID, req.min_pos * factor);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::set_max_vel(motor_controller_msgs::Configuration::Request & req,
                                motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _vel_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor * 60. / (2. * M_PI);

    setDriverMaxVel(_RSd, _semID, req.max_vel * factor);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::set_max_acc(motor_controller_msgs::Configuration::Request & req,
                                motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    setDriverMaxAcc(_RSd, _semID, req.max_acc * factor);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::set_max_dec(motor_controller_msgs::Configuration::Request & req,
                                motor_controller_msgs::Configuration::Response & resp)
{
    double joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    double factor = joint_factor / (2. * M_PI);

    setDriverMaxDec(_RSd, _semID, req.max_dec * factor);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::set_cur_lim(motor_controller_msgs::Configuration::Request & req,
                                motor_controller_msgs::Configuration::Response & resp)
{
    setDriverCurLim(_RSd, _semID, req.cur);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::set_odometry_calibration(motor_controller_msgs::Configuration::Request & req,
                                             motor_controller_msgs::Configuration::Response & resp)
{
    // neck
    setDriverHomePosition(_RSd, _semID, req.home * _pos_factor / (2. * M_PI));
    // arms
    //    return setDriverOdoCalibration(_RSd, _semID, home * TOTAL_ARMS_REDUCTION / (2. * M_PI));

    return true;
}

//////////////////////////////////////////////////

////////////////////////
// moves
////////////////////////

bool Mcdc3006sNode::move_abs_pos(motor_controller_msgs::MoveAbsPos::Request & req,
                                 motor_controller_msgs::MoveAbsPos::Response & resp)
{
    ROS_DEBUG("[MOTOR_CONTROLLER] moveAbsPos RSd = %d\t semID = %d\t pos = %d\n", _RSd, _semID, req.position);

    long int joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    long int factor = joint_factor / (2. * M_PI);

    moveDriverAbsPos(_RSd, _semID, req.position * factor);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::move_rel_pos(motor_controller_msgs::MoveAbsPos::Request & req,
                                 motor_controller_msgs::MoveAbsPos::Response & resp)
{
    ROS_DEBUG("[MOTOR_CONTROLLER] moveRelPos RSd = %d\t semID = %d\t pos = %d\n", _RSd, _semID, req.position);

    long int joint_factor = (_joint_name == NECK ? _pos_factor : TOTAL_ARMS_REDUCTION);
    long int factor = joint_factor / (2. * M_PI);

    moveDriverRelPos(_RSd, _semID, req.position * factor);

    return true;
}

//////////////////////////////////////////////////

bool Mcdc3006sNode::move_vel(motor_controller_msgs::MoveAbsPos::Request & req,
                             motor_controller_msgs::MoveAbsPos::Response & resp)
{
    ROS_DEBUG("[MOTOR_CONTROLLER] moveVel RSd = %d\t semID = %d\t vel = %d\n", _RSd, _semID, req.position);

    // change velocity from rad/sec to rpm
    long int joint_factor = (_joint_name == NECK ? _vel_factor : TOTAL_ARMS_REDUCTION / PULSES_PER_REV);
    long int factor = joint_factor * 60. / (2. * M_PI);

    moveDriverVel(_RSd, _semID, req.position * factor);

    return true;
}

//////////////////////////////////////////////////
