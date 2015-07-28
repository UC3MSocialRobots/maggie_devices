/**
 * @file        labjack_node.cpp
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

#include "labjack_node.h"

//////////////////////////////////////////////////

LabjackNode::LabjackNode(LabjackDriverInterface *ljdriver) :
    _nh_private("~"),
    _publish_rate(10),
    _ljdriver(ljdriver)
{
    long error = 0;
    bool connected = false;

    while(!connected) {
        error = _ljdriver->config();
        if (error != 0) {
            ROS_WARN("[LABJACK] Trying after 5 seconds...");
            usleep(5000 * 1000);
        }
        else {
            connected = true;

            getAll();
        }
    }

    ROS_DEBUG("[LABJACK] Labjack detected. Waiting for a command...");
}

//////////////////////////////////////////////////

LabjackNode::~LabjackNode()
{
}

//////////////////////////////////////////////////

void LabjackNode::init()
{
    _touch_srv = _nh_private.advertiseService("get_touch_sensors", &LabjackNode::getTouchSensors, this);
    _get_voltage_srv = _nh_private.advertiseService("get_voltage", &LabjackNode::getPower, this);
    _get_is_plugged_srv = _nh_private.advertiseService("is_plugged", &LabjackNode::getIsPlugged, this);
    _get_state_srv = _nh_private.advertiseService("get_state", &LabjackNode::getStates, this);
    _set_state_srv = _nh_private.advertiseService("set_state", &LabjackNode::setStates, this);
}

//////////////////////////////////////////////////

void LabjackNode::spin()
{
    while(_nh.ok()) {
        ros::spinOnce();
        _publish_rate.sleep();
    }
}

//////////////////////////////////////////////////

bool LabjackNode::getTouchSensors(touch_skill_msgs::LabjackTouchInfo::Request & req,
                                  touch_skill_msgs::LabjackTouchInfo::Response & resp)
{
    touch aux = getTouch();

    for (int i = 0; i < touch_skill_msgs::MaggieTouchParts::NUMBER_SENSORS; i++) {
        resp.sensors_status[i] = aux.sensor[i];
    }

    return true;
}

//////////////////////////////////////////////////

bool LabjackNode::getPower(batteries_skill_msgs::LabjackBatteryInfo::Request & req,
                           batteries_skill_msgs::LabjackBatteryInfo::Response & resp)
{
    resp.voltage = getVoltage();

    return true;
}

//////////////////////////////////////////////////

bool LabjackNode::getIsPlugged(batteries_skill_msgs::LabjackPlugInfo::Request & req,
                               batteries_skill_msgs::LabjackPlugInfo::Response & resp)
{
    resp.is_plugged = isPlugged();

    return true;
}

//////////////////////////////////////////////////

bool LabjackNode::getStates(basic_states_skill_msgs::GetLabjackState::Request & req,
                            basic_states_skill_msgs::GetLabjackState::Response & resp)
{
    resp.state = getState();

    return true;
}

//////////////////////////////////////////////////

bool LabjackNode::setStates(basic_states_skill_msgs::SetLabjackState::Request & req,
                            basic_states_skill_msgs::SetLabjackState::Response & resp)
{
    if ((setState(req.state)) < 0) {
        ROS_ERROR("[LABJACK] Error setting new state: %d", req.state);
        return false;
    }

    return true;
}

//////////////////////////////////////////////////

t_ljdata LabjackNode::getAll()
{
    t_ljdata data;
    int error = 0;

    long stateDs = 0, stateIOs = 0;
    if ((error = _ljdriver->readDIOs(&stateDs, &stateIOs)) != 0) {
        ROS_ERROR("[LABJACK] Error reading D/IO lines");
        stateDs = stateIOs = -1;
    }
    _labjack_data.DsState = stateDs;
    _labjack_data.IOsState = stateIOs;

    long lines03[] = {0,
                      1,
                      2,
                      3};
    long lines47[] = {4,
                      5,
                      6,
                      7};
    float voltages03[] = {0,
                          0,
                          0,
                          0};
    float voltages47[] = {0,
                          0,
                          0,
                          0};
    if ((error = _ljdriver->readAIs(4, lines03, voltages03)) != 0) {
        ROS_ERROR("[LABJACK] Error reading AI0..AI3 lines");
        voltages03[0] = voltages03[1] = voltages03[2] = voltages03[3] = -1;
    }
    if ((error = _ljdriver->readAIs(4, lines47, voltages47)) != 0) {
        ROS_ERROR("[LABJACK] Error reading AI4..AI7 lines");
        voltages47[0] = voltages47[1] = voltages47[2] = voltages47[3] = -1;
    }
    for (short i = 0; i < 4; i++) {
        _labjack_data.AIsState[i] = voltages03[i];
        _labjack_data.AIsState[i + 4] = voltages47[i];
    }

    long DsDirection = 0;
    if ((error = _ljdriver->readDirectionDs(&DsDirection)) != 0) {
        ROS_ERROR("[LABJACK] Error reading Ds directions");
        DsDirection = -1;
    }
    _labjack_data.DsDirection = DsDirection;

    data.DsState = _labjack_data.DsState;
    data.IOsState = _labjack_data.IOsState;
    for (short i = 0; i < NUM_LIN_AI; i++) {
        data.AIsState[i] = _labjack_data.AIsState[i];
    }

    return data;
}

//////////////////////////////////////////////////

float LabjackNode::getStateDs()
{
    long stateDs = 0, stateIOs = 0;

    if (_ljdriver->readDIOs(&stateDs, &stateIOs) != 0) {
        ROS_ERROR("[LABJACK] Error reading D/IO lines");
        stateDs = stateIOs = -1;
    }
    _labjack_data.DsState = stateDs;

    return stateDs;
}

//////////////////////////////////////////////////

int LabjackNode::setStateDs(long stateD)
{
    int error = 0;
    long stateIOs = 0;

    stateIOs = _labjack_data.IOsState;
    error = (int) _ljdriver->writeDIOsCONFIG(&stateD, &stateIOs);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::getStateD(long stateD)
{
    int result = 0, error = 0;
    long mask = 0x0001;

    if (stateD < 0 || stateD >= NUM_LIN_D) {
        result = -1;
    }
    else {
        long stateDs = 0, stateIOs = 0;
        if ((error = _ljdriver->readDIOs(&stateDs, &stateIOs)) != 0) {
            ROS_ERROR("[LABJACK] Error reading D/IO lines");
            stateDs = stateIOs = -1;
        }
        _labjack_data.DsState = stateDs;

        mask <<= stateD;
        if (stateDs & mask) {
            result = 1;
        }
        else {
            result = 0;
        }
    }

    return result;
}

//////////////////////////////////////////////////

int LabjackNode::setStateD(long stateD)
{
    int error = 0;

    // write D line and reconfig the directions
    error = (int) _ljdriver->writeDCONFIG(stateD, 1);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::clearStateD(long stateD)
{
    int error = 0;

    error = (int) _ljdriver->writeDCONFIG(stateD, 0);

    return error;
}

//////////////////////////////////////////////////

long LabjackNode::getDsDirection()
{
    int error = 0;
    long DsDirection = 0;

    if ((error = _ljdriver->readDirectionDs(&DsDirection)) != 0) {
        ROS_ERROR("[LABJACK] Error reading Ds directions");
        DsDirection = -1;
    }
    _labjack_data.DsDirection = DsDirection;

    return DsDirection;
}

//////////////////////////////////////////////////

float LabjackNode::getStateIOs()
{
    int error = 0;
    long stateDs = 0, stateIOs = 0;

    if ((error = _ljdriver->readDIOs(&stateDs, &stateIOs)) != 0) {
        ROS_ERROR("[LABJACK] Error reading D/IO lines");
        stateDs = stateIOs = -1;
    }
    _labjack_data.IOsState = stateIOs;

    return stateIOs;
}

//////////////////////////////////////////////////

int LabjackNode::setStateIOs(long stateIO)
{
    int error;
    long stateD;

    stateD = _labjack_data.DsState;

    error = (int) _ljdriver->writeDIOsCONFIG(&stateD, &stateIO);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::getStateIO(long stateIO)
{
    int result = 0;
    long mask = 0x0001;

    if (stateIO < 0 || stateIO >= NUM_LIN_IO) {
        result = -1;
    }
    else {
        int error = 0;
        long stateDs = 0, stateIOs = 0;

        if ((error = _ljdriver->readDIOs(&stateDs, &stateIOs)) != 0) {
            ROS_ERROR("[LABJACK] Error reading D/IO lines");
            stateDs = stateIOs = -1;
        }
        _labjack_data.IOsState = stateIOs;

        mask <<= stateIO;
        if (stateIOs & mask)
            result = 1;
        else
            result = 0;
    }

    return result;
}

//////////////////////////////////////////////////

int LabjackNode::setStateIO(long stateIO)
{
    int error = 0;

    error = (int) _ljdriver->writeIO(stateIO, 1);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::clearStateIO(long stateIO)
{
    int error = 0;

    error = (int) _ljdriver->writeIO(stateIO, 0);

    return error;
}

//////////////////////////////////////////////////

float8 LabjackNode::getAIs()
{
    float8 result;

    long lines03[] = {0,
                      1,
                      2,
                      3};
    long lines47[] = {4,
                      5,
                      6,
                      7};
    float voltages03[] = {0,
                          0,
                          0,
                          0};
    float voltages47[] = {0,
                          0,
                          0,
                          0};

    if (_ljdriver->readAIs(4, lines03, voltages03) != 0) {
        ROS_ERROR("[LABJACK] Error reading AI0..AI3 lines");
        voltages03[0] = voltages03[1] = voltages03[2] = voltages03[3] = -1;
    }
    if (_ljdriver->readAIs(4, lines47, voltages47) != 0) {
        ROS_ERROR("[LABJACK] Error reading AI4..AI7 lines");
        voltages47[0] = voltages47[1] = voltages47[2] = voltages47[3] = -1;
    }
    for (short i = 0; i < 4; i++) {
        _labjack_data.AIsState[i] = voltages03[i];
        _labjack_data.AIsState[i + 4] = voltages47[i];
    }

    for (short i = 0; i < NUM_LIN_AI; i++) {
        result.v_float8[i] = _labjack_data.AIsState[i];
    }

    return result;
}

//////////////////////////////////////////////////

float LabjackNode::getAI(long channel)
{
    float result = 0.0;
    int error = 0;

    if (channel < 0 || channel >= NUM_LIN_AI) {
        result = -1;
    }
    else {
        if ((error = _ljdriver->readAI(channel, &result)) != 0) {
            ROS_ERROR("[LABJACK] Error reading AI%ld line", channel);
            result = -1;
        }
        _labjack_data.AIsState[channel] = result;
    }

    return result;
}

//////////////////////////////////////////////////

int LabjackNode::setAOs(float2 voltageAO)
{
    int error = 0;

    error = (int) _ljdriver->writeAOs(voltageAO.v_float2[0], voltageAO.v_float2[1]);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::setPulseD(t_pulseData data)
{
    int error = 0;
    long mask = 1 << (int) data.line;
    long directions = 0;

    if (_ljdriver->readDirectionDs(&directions) == 0) {
        // line is configured as output
        if ((directions & mask) != 0) {
            error = (int) _ljdriver->writePulse(data.level, data.line, data.microseconds, 0);
        }
        else {
            // line is configured as input
            ROS_WARN("[LABJACK] Impossible to write a pulse to an input line");
            error = -1;
        }
    }
    else {
        ROS_ERROR("[LABJACK] Error reading line direction");
        error = -1;
    }

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::setPulseIO(t_pulseData data)
{
    int error = 0;
    long mask = 1 << (int) data.line;
    long directions = 0;

    if (_ljdriver->readDirectionDs(&directions) == 0) {
        // line is configured as output
        if ((directions & mask) != 0) {
            error = (int) _ljdriver->writePulse(data.level, data.line, data.microseconds, 1);
        }
        else {
            // line is configured as input
            ROS_WARN("[LABJACK] Impossible to write a pulse to an input line");
            error = -1;
        }
    }
    else {
        ROS_ERROR("[LABJACK] Error reading line direction");
        error = -1;
    }

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::getState()
{
    int state = 0;
    float stateIO = getStateIOs();

    if (!((long) (stateIO) & EMERGENCY)) {
        state = basic_states_skill_msgs::States::EMERGENCY_STATE;
    }
    else if (!((long) (stateIO) & ENABLE_BASE) && !((long) (stateIO) & ENABLE_BODY)) {
        // sleep state => !base & !body (low level closed by rele)
        state = basic_states_skill_msgs::States::SLEEP_STATE;
    }
    else if (!((long) (stateIO) & ENABLE_BASE) && ((long) (stateIO) & ENABLE_BODY)) {
        // stop state => !base & body (low level closed by rele)
        state = basic_states_skill_msgs::States::STOP_STATE;
    }
    else if (((long) (stateIO) & ENABLE_BASE) && ((long) (stateIO) & ENABLE_BODY)) {
        // active state => base & body (low level closed by rele)
        state = basic_states_skill_msgs::States::ACTIVE_STATE;
    }
    else {
        state = basic_states_skill_msgs::States::UNDEFINED_STATE;
    }

    return state;
}

//////////////////////////////////////////////////

int LabjackNode::setState(int state)
{
    int error = 0;

    switch(state) {
        case basic_states_skill_msgs::States::SLEEP_STATE:
            ROS_DEBUG("[LABJACK] SLEEP STATE");
            if (_ljdriver->writeIO(IO_ENABLE_BASE, DISABLE) != 0
                || _ljdriver->writeIO(IO_ENABLE_OTHERS, DISABLE) != 0) {
                ROS_ERROR("[LABJACK] Error in setState: changing state to SLEEP");
                error = -1;
            }
            break;
        case basic_states_skill_msgs::States::STOP_STATE:
            ROS_DEBUG("[LABJACK] STOP STATE");
            if (_ljdriver->writeIO(IO_ENABLE_BASE, DISABLE) != 0 || _ljdriver->writeIO(IO_ENABLE_OTHERS, ENABLE) != 0) {
                ROS_ERROR("[LABJACK] Error in setState: changing state to STOP");
                error = -1;
            }
            ROS_DEBUG("[LABJACK] Checking state: %d == %d", basic_states_skill_msgs::States::STOP_STATE, getState());
            break;
        case basic_states_skill_msgs::States::ACTIVE_STATE:
            ROS_DEBUG("[LABJACK] ACTIVE STATE");
            if (_ljdriver->writeIO(IO_ENABLE_BASE, ENABLE) != 0 || _ljdriver->writeIO(IO_ENABLE_OTHERS, ENABLE) != 0) {
                ROS_ERROR("[LABJACK] Error in setState: changing state to ACTIVE");
                error = -1;
            }
            break;
        case basic_states_skill_msgs::States::EMERGENCY_STATE:
            ROS_DEBUG("[LABJACK] EMERGENCY STATE");
            if (setEmergency()) {
                ROS_ERROR("[LABJACK] Error in setState: changing state to EMERGENCY");
                error = -1;
            }
            break;
        default:
            ROS_DEBUG("[LABJACK] UNDEFINED STATE");
            error = -1;
    }

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::enableBase(bool enable)
{
    int error = 0;
    long line_enable_base = ENABLE_BASE;

    error = (enable) ? clearStateIO(line_enable_base) : setStateIO(line_enable_base);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::enableBody(bool enable)
{
    int result = 0;
    long line_enable_body = ENABLE_BODY;

    result = (enable) ? clearStateIO(line_enable_body) : setStateIO(line_enable_body);

    return result;
}

//////////////////////////////////////////////////

int LabjackNode::setWatchdog()
{
    int error = 0;
    long timeout = TIMEOUT_WD;
    int lineD_WD = D_WATCHDOG;
    long state = 0;

    error = (int) _ljdriver->enableWatchdog(timeout, lineD_WD, state);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::clearWatchdog()
{
    int error = 0;

    error = (int) _ljdriver->disableWatchdog(D_WATCHDOG);

    return error;
}

//////////////////////////////////////////////////

int LabjackNode::isEmergency()
{
    int result = 0;
    long emergency_line = IO_EMERGENCY;

    result = getStateIO(emergency_line);

    return result;
}

//////////////////////////////////////////////////

int LabjackNode::setEmergency()
{
    int error = 0;

    // send a low level pulse of 10ms
    if ((_ljdriver->writePulse(0, IO_SETA_SF, 10000, 1)) != 0) {
        error = -1;
    }

    if (!error) {
        // disable lines for the base and the body
        if (_ljdriver->writeIO(IO_ENABLE_BASE, DISABLE) != 0 || _ljdriver->writeIO(IO_ENABLE_OTHERS, DISABLE) != 0) {
            error = -1;
        }
    }

    return error;
}

//////////////////////////////////////////////////

float LabjackNode::getVoltage()
{
    float result = 0.0;
    float v = 0.0;
    long ai_voltage_base = AI_VOLTAGE_BASE;

    v = getAI(ai_voltage_base);
    if (v == -1) {
        ROS_ERROR("[LABJACK] Error getting the voltage");
        result = v;
    }
    else {
        result = v * GAIN_VOLT;
    }

    return result;
}

//////////////////////////////////////////////////

bool LabjackNode::isPlugged()
{
    bool result = true;
    float v = 0.0;
    long ai_plugged = AI_PLUGGED;

    v = getAI(ai_plugged);
    if (v == -1) {
        ROS_ERROR("[LABJACK] Error reading the charger");
        result = false;
    }
    else {
        (v > 3.0) ? result = true : result = false;
    }

    return result;
}

//////////////////////////////////////////////////

touch LabjackNode::getTouch()
{
    static touch result;
    long mask = 0;
    int error = 0;

    long stateDs = 0, stateIOs = 0;
    if ((error = _ljdriver->readDIOs(&stateDs, &stateIOs)) != 0) {
        ROS_ERROR("[LABJACK] Error reading D/IO lines");
        stateDs = stateIOs = -1;
    }
    _labjack_data.DsState = stateDs;

    // touch sensors are activated to low level (0- touched, 1- non touched)
    mask = HEAD; // point to D1
    // check lines D1 to D3, touch sensors associated
    for (short i = 1; i <= 3; i++) {
        if ((~stateDs) & mask) {
            result.sensor[i - 1] = TOUCHED;
        }
        else {
            result.sensor[i - 1] = UNTOUCHED;
        }
        mask <<= 1;
    }

    mask = LEFT_SHOULDER; // point to D5
    // check lines D5 to D7, touch sensor associated
    for (short i = 5; i <= 7; i++) {
        if ((~stateDs) & mask) {
            result.sensor[i - 2] = TOUCHED;
        }
        else {
            result.sensor[i - 2] = UNTOUCHED;
        }
        mask <<= 1;
    }

    mask = RIGHT_FOREARM; // point to D11
    // check lines D11 to D15, touch sensors associated
    for (short i = 11; i <= 15; i++) {
        if ((~stateDs) & mask) {
            result.sensor[i - 5] = TOUCHED;
        }
        else {
            result.sensor[i - 5] = UNTOUCHED;
        }
        mask <<= 1;
    }

    return result;
}

//////////////////////////////////////////////////
