/**
 * @file        base_motor_node.cpp
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

#include "base_motor_node.h"

//////////////////////////////////////////////////

BaseMotorNode::BaseMotorNode() :
    _nh_private("~"), _publish_rate(10)
{
    _bm.init_communication();
}

//////////////////////////////////////////////////

BaseMotorNode::~BaseMotorNode()
{
    _bm.end_communication();
}

//////////////////////////////////////////////////

void BaseMotorNode::init()
{
    odom_pub = _nh.advertise<nav_msgs::Odometry>("odom", 50);
    sub = _nh.subscribe("cmd_vel", 1, &BaseMotorNode::twist_callback, this);
}

//////////////////////////////////////////////////

void BaseMotorNode::spin()
{
    ROS_DEBUG("[BASE_MOTOR_NODE] wait for /clock to emit...");
    while(_last_time.isZero()) {
        _current_time = ros::Time::now();
        _last_time = ros::Time::now();
    }
    ROS_DEBUG("[BASE_MOTOR_NODE] /clock time received OK.");

    while(_nh.ok()) {
        ros::spinOnce();
        _publish_rate.sleep();

        publish();
    }
}

//////////////////////////////////////////////////

void BaseMotorNode::publish()
{
    double x = 0, y = 0, th = 0;

    _current_time = ros::Time::now();

    // compute odometry in a typical way given the velocities of the robot
    double dt = (_current_time - _last_time).toSec();

    cinematic_data data;
    _bm.read_data_variable_time_diff(&data, dt);

    // speeds
    double vx = data.v / 1000.f; // mm/ sec -> m /sec
    double vy = 0; // the robot cannot have side speed
    double vth = data.w * M_PI / 180.f; // degrees /sec -> rad /sec

    // odometry

    // first method: trust the base primitive for odometry
    x = data.x / 1000.f; // mm -> m
    y = data.y / 1000.f; // mm -> m
    th = data.theta * M_PI / 180.f; // degrees -> rad

    // second method: first order integration
    // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // double delta_th = vth * dt;
    // x += delta_x;
    // y += delta_y;
    // th += delta_th;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = _current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = _current_time;
    odom.header.frame_id = "odom";

    // set the position
    std::ostringstream ans_position;
    ans_position << "(" << x << ", " << y << ", " << 0.0 << ")";
    std::ostringstream ans_orientation;
    ans_orientation << "(" << odom_quat.x << ", " << odom_quat.y << ", " << odom_quat.z << ")";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // print mgs
    ROS_DEBUG_NAMED(
        "base_motor_node",
        "[BASE_MOTOR_NODE] _current_time:%g (dt:%g) - position: %s, orientation :%s (theta:%g) vx:%g, vy:%g, vth:%g",
        _current_time.toSec(), dt, ans_position.str().c_str(), ans_orientation.str().c_str(), th, vx, vy, vth);

    // publish the message
    odom_pub.publish(odom);

    _last_time = _current_time;
}

//////////////////////////////////////////////////

void BaseMotorNode::twist_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    ROS_DEBUG_NAMED("base_motor_node", "[BASE_MOTOR_NODE] Linear (vx: %f, vy: %f, vz: %f)", cmd_vel->linear.x,
                    cmd_vel->linear.y, cmd_vel->linear.z);
    ROS_DEBUG_NAMED("base_motor_node", "[BASE_MOTOR_NODE] Angular (vx: %f, vy: %f, vtheta: %f)", cmd_vel->angular.x,
                    cmd_vel->angular.y, cmd_vel->angular.z);

    // convert linear speed: m/s -> mm/s
    double linear_speed_mm_s = cmd_vel->linear.x * 1000.f;

    // convert angular speed: radian/s -> degrees/s
    double angular_speed_deg_s = cmd_vel->angular.z * 180.f / M_PI;

    // set the new speeds
    _bm.set_velocity(linear_speed_mm_s, angular_speed_deg_s);
}

//////////////////////////////////////////////////
