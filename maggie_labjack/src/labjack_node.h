#ifndef __LABJACK_NODE_H__
#define __LABJACK_NODE_H__

/**
 * @file        labjack_node.h
 * @brief       Node for controlling the labjack controller.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
 * @author      Alvaro Castro Gonzalez <acgonzal@ing.uc3m.es>
 * @date        2007-12
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

#include "ljacklm_wrapper.h"
#include "labjack_data.h"

// messages and services
#include <basic_states_skill_msgs/States.h>
#include <basic_states_skill_msgs/GetLabjackState.h>
#include <basic_states_skill_msgs/SetLabjackState.h>
#include <batteries_skill_msgs/LabjackBatteryInfo.h>
#include <batteries_skill_msgs/LabjackPlugInfo.h>
#include <touch_skill_msgs/LabjackTouchInfo.h>

class LabjackNode {
    public:
        /**
         * @brief Empty constructor.
         */
        LabjackNode(LabjackDriverInterface *ljdriver);

        /**
         * @brief Destructor.
         */
        ~LabjackNode();

        /**
         * @brief Initialize the labjack node.
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
         * @brief Publish information from the touch sensors.
         */
        void publish_touch();

        /**
         * @brief Publish information from the battery.
         */
        void publish_battery();

        /**
         * @brief Publish information from the states.
         */
        void publish_state();

        //////////////////////////////////////////////////
        // services
        //////////////////////////////////////////////////

        /**
         * @brief Callback to get the information of the touch sensors.
         * @param req LabjackTouchInfo type request.
         * @param resp LabjackTouchInfo type response.
         * @return true if there are no errors.
         */
        bool getTouchSensors(touch_skill_msgs::LabjackTouchInfo::Request & req,
                             touch_skill_msgs::LabjackTouchInfo::Response & resp);

        /**
         * @brief Callback to get the information of the batteries.
         * @param req LabjackBatteryInfo type request.
         * @param resp LabjackBatteryInfo type response.
         * @return true if there are no errors.
         */
        bool getPower(batteries_skill_msgs::LabjackBatteryInfo::Request & req,
                      batteries_skill_msgs::LabjackBatteryInfo::Response & resp);

        /**
         * @brief Callback to get the information of the batteries.
         * @param req LabjackBatteryInfo type request.
         * @param resp LabjackBatteryInfo type response.
         * @return true if there are no errors.
         */
        bool getIsPlugged(batteries_skill_msgs::LabjackPlugInfo::Request & req,
                          batteries_skill_msgs::LabjackPlugInfo::Response & resp);

        /**
         * @brief Callback to get the information of the states.
         * @param req GetLabjackState type request.
         * @param resp GetLabjackState type response.
         * @return true if there are no errors.
         */
        bool getStates(basic_states_skill_msgs::GetLabjackState::Request & req,
                       basic_states_skill_msgs::GetLabjackState::Response & resp);

        /**
         * @brief Callback to set a state.
         * @param req SetLabjackState type request.
         * @param resp SetLabjackState type response.
         * @return true if there are no errors.
         */
        bool setStates(basic_states_skill_msgs::SetLabjackState::Request & req,
                       basic_states_skill_msgs::SetLabjackState::Response & resp);

        //////////////////////////////////////////////////
        // methods
        //////////////////////////////////////////////////

        /**
         * @brief Function which return all data relative to the labjack. Digital and analog lines.
         * These lines have connected: touch sensors, batteries, state of the emergency button RF,
         * state of the emergency button of Maggie, control signal of motor in the base and control
         * signal of the other motors.
         * @return if no error, return all data, if error, NULL
         */
        t_ljdata getAll();

        // low level methods

        /**
         * @brief Function which return the state of all D lines.
         * @return if error, return negative values.
         */
        float getStateDs();

        /**
         * @brief Function which assing a new state to all D lines.
         * @param stateD new state to set.
         * @return return 0 if no error, other thing if something happen.
         */
        int setStateDs(long stateD);

        /**
         * @brief Function which return the state of a specific D line.
         * @param stateD the line specified.
         * @return return 0 if no error, other thing if something happen.
         */
        int getStateD(long stateD);

        /**
         * @brief Function which assign a value of 1 to the D line specified in the parameter.
         * @param stateD the state of the line to set.
         * @return return 0 if no error, other thing if something happen.
         */
        int setStateD(long stateD);

        /**
         * @brief Function which assign a value of 0 to the D line specified in the parameter.
         * @param stateD the state of the line to clear.
         * @return return 0 if no error, other thing if something happen.
         */
        int clearStateD(long stateD);

        /**
         * @brief Function which return the directions of the D lines: 0 is an input and 1 an output.
         * @return if error, return negative values.
         */
        long getDsDirection();

        /**
         * @brief Function which get the state of the IO lines.
         * @return the state of the IOs.
         */
        float getStateIOs();

        /**
         * @brief Function which set the state of the IO lines.
         * @param stateIO the new state of IO line.
         * @return return 0 if no error, other thing if something happen.
         */
        int setStateIOs(long stateIO);

        /**
         * @brief Function which get the state of one IO line.
         * @param stateIO the state of the IO line.
         * @return return 0 if no error, other thing if something happen.
         */
        int getStateIO(long stateIO);

        /**
         * @brief Function which set the state of one IO line.
         * @param stateIO the of the line to set.
         * @return return 0 if no error, other thing if something happen.
         */
        int setStateIO(long stateIO);

        /**
         * @brief Function which clear the state of one IO line.
         * @param stateIO the state of the line to clear.
         * @return return 0 if no error, other thing if something happen.
         */
        int clearStateIO(long stateIO);

        /**
         * @brief Function which get the value of the AI lines.
         * @return return 0 if no error, other thing if something happen.
         */
        float8 getAIs();

        /**
         * @brief Function which get the value of one AI line.
         * @param channel the channel of the line to clear.
         * @return return 0 if no error, other thing if something happen.
         */
        float getAI(long channel);

        /**
         * @brief Function which assign a voltage. If only one parameter desire, pass negative value to the other.
         * @param voltageAO the voltage to set.
         * @return return 0 if no error, other thing if something happen.
         */
        int setAOs(float2 voltageAO);

        /**
         * @brief Function which write a pulse in a D line.
         * @param data the data to set.
         * @return return 0 if no error, other thing if something happen.
         */
        int setPulseD(t_pulseData data);

        /**
         * @brief Function which write a pulse in a IO line.
         * @param data the data to set.
         * @return return 0 if no error, other thing if something happen.
         */
        int setPulseIO(t_pulseData data);

        // states methods

        /**
         * @brief Function which get the current state.
         * @return the current state of the labjack.
         */
        int getState();

        /**
         * @brief Function which set the state.
         * @param state the new state.
         * @return return 0 if no error, other thing if something happen.
         */
        int setState(int state);

        /**
         * @brief Function which enable the base to move.
         * @param enable true or false depending on the case.
         * @return return 0 if no error, other thing if something happen.
         */
        int enableBase(bool enable);

        /**
         * @brief Function which enable the body to move.
         * @param enable true or false depending on the case.
         * @return return 0 if no error, other thing if something happen.
         */
        int enableBody(bool enable);

        /**
         * @brief Function which set the watchdog.
         * @return return 0 if no error, other thing if something happen.
         */
        int setWatchdog();

        /**
         * @brief Function which clear the watchog.
         * @return return 0 if no error, other thing if something happen.
         */
        int clearWatchdog();

        /**
         * @brief Function which get if the labjack is in emergency state.
         * @return true or false depending on the state of the IO line.
         */
        int isEmergency();

        /**
         * @brief Function which emergency by software.
         * @return return 0 if no error, other thing if something happen.
         */
        int setEmergency();

        // sensors methods

        /**
         * @brief Function which return the value in volts of the battery.
         * @return the voltage of the battery.
         */
        float getVoltage();

        /**
         * @brief Function which return the state of the robot connection.
         * @return true if robot is plugged, false in other case.
         */
        bool isPlugged();

        /**
         * @brief Function which get the value of the touch sensors.
         * @return the current value of the touch sensors.
         */
        touch getTouch();

    private:
        // nodes
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        // services
        ros::ServiceServer _touch_srv;
        ros::ServiceServer _get_voltage_srv;
        ros::ServiceServer _get_is_plugged_srv;
        ros::ServiceServer _get_state_srv;
        ros::ServiceServer _set_state_srv;

        // spin rate
        ros::Rate _publish_rate;

        t_ljdata _labjack_data;
        LabjackDriverInterface *_ljdriver;
};

#endif
