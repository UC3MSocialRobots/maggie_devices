/**
 * @file        gtest_labjack_node.cpp
 * @brief       Unit tests.
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

#include <gtest/gtest.h>
#include "ljacklm_mock.h"
#include "labjack_node.h"
#include "labjack_data.h"

using testing::_;
using testing::DoAll;
using testing::SetArgumentPointee;
using testing::AnyNumber;
using testing::Return;

// Test getStateDs()
TEST(LabjackNode, GetStateDs)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_FLOAT_EQ(0, labjack_node.getStateDs());

    // fail
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.getStateDs());
}

// Test setStateDs()
TEST(LabjackNode, SetStateDs)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeDIOsCONFIG(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.setStateDs(0));

    // fail
    EXPECT_CALL(mock, writeDIOsCONFIG(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setStateDs(0));
}

// Test getStateD()
TEST(LabjackNode, GetStateD)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.getStateD(0));

    // fail
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(1, labjack_node.getStateD(0));

    ASSERT_EQ(-1, labjack_node.getStateD(-1));
}

// Test setStateD()
TEST(LabjackNode, SetStateD)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeDCONFIG(_, 1))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.setStateD(0));

    // fail
    EXPECT_CALL(mock, writeDCONFIG(_, 1))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setStateD(0));
}

// Test clearStateD()
TEST(LabjackNode, ClearStateD)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeDCONFIG(_, 0))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.clearStateD(0));

    // fail
    EXPECT_CALL(mock, writeDCONFIG(_, 0))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.clearStateD(0));
}

// Test getDsDirection()
TEST(LabjackNode, GetDsDirection)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.getDsDirection());

    // fail
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.getDsDirection());
}

// Test getStateIOs()
TEST(LabjackNode, GetStateIOs)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.getStateIOs());

    // fail
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.getStateIOs());
}

// Test getDsDirection()
TEST(LabjackNode, SetStateIOs)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeDIOsCONFIG(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.setStateIOs(0));

    // fail
    EXPECT_CALL(mock, writeDIOsCONFIG(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setStateIOs(0));
}

// Test getStateIO()
TEST(LabjackNode, GetStateIO)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.getStateIO(0));

    // fail
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(1, labjack_node.getStateIO(0));

    ASSERT_EQ(-1, labjack_node.getStateIO(-1));
    ASSERT_EQ(-1, labjack_node.getStateIO(NUM_LIN_IO + 1));
}

// Test setStateIO()
TEST(LabjackNode, SetStateIO)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeIO(_, 1))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.setStateIO(0));

    // fail
    EXPECT_CALL(mock, writeIO(_, 1))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setStateIO(0));
}

// Test clearStateIO()
TEST(LabjackNode, ClearStateIO)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeIO(_, 0))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.clearStateIO(0));

    // fail
    EXPECT_CALL(mock, writeIO(_, 0))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.clearStateIO(0));
}

// Test getAIs()
TEST(LabjackNode, GetAI)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readAI(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0.0, labjack_node.getAI(0));

    // fail
    EXPECT_CALL(mock, readAI(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_FLOAT_EQ(-1, labjack_node.getAI(0));

    ASSERT_FLOAT_EQ(-1, labjack_node.getAI(-1));
    ASSERT_FLOAT_EQ(-1, labjack_node.getAI(NUM_LIN_AI + 1));
}

// Test setAOs()
TEST(LabjackNode, SetAOs)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    float2 data;

    // success
    EXPECT_CALL(mock, writeAOs(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.setAOs(data));

    // fail
    EXPECT_CALL(mock, writeAOs(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setAOs(data));
}

// Test setPulseD()
TEST(LabjackNode, SetPulseD)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    t_pulseData data;

    // fail
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(-1, labjack_node.setPulseD(data));

    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setPulseD(data));
}

// Test setPulseIO()
TEST(LabjackNode, SetPulseIO)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    t_pulseData data;

    // fail
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(-1, labjack_node.setPulseIO(data));

    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setPulseIO(data));
}

// Test setState()
TEST(LabjackNode, SetState)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeIO(_, _))
        .Times(2)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.setState(basic_states_skill_msgs::States::SLEEP_STATE));

    EXPECT_CALL(mock, writeIO(_, _))
        .Times(2)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.setState(basic_states_skill_msgs::States::STOP_STATE));

    EXPECT_CALL(mock, writeIO(_, _))
        .Times(2)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.setState(basic_states_skill_msgs::States::ACTIVE_STATE));

    // fail
    EXPECT_CALL(mock, writeIO(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setState(basic_states_skill_msgs::States::SLEEP_STATE));

    ASSERT_EQ(-1, labjack_node.setState(10));
}

// Test enableBase()
TEST(LabjackNode, EnableBase)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeIO(_, 0))
        .Times(1)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.enableBase(true));

    EXPECT_CALL(mock, writeIO(_, 1))
        .Times(1)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.enableBase(false));

    // fail
    EXPECT_CALL(mock, writeIO(_, 0))
        .Times(1)
        .WillRepeatedly(Return(-1));

    ASSERT_EQ(-1, labjack_node.enableBase(true));

    EXPECT_CALL(mock, writeIO(_, 1))
        .Times(1)
        .WillRepeatedly(Return(-1));

    ASSERT_EQ(-1, labjack_node.enableBase(false));
}

// Test enableBody()
TEST(LabjackNode, EnableBody)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writeIO(_, 0))
        .Times(1)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.enableBody(true));

    EXPECT_CALL(mock, writeIO(_, 1))
        .Times(1)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.enableBody(false));

    // fail
    EXPECT_CALL(mock, writeIO(_, 0))
        .Times(1)
        .WillRepeatedly(Return(-1));

    ASSERT_EQ(-1, labjack_node.enableBody(true));

    EXPECT_CALL(mock, writeIO(_, 1))
        .Times(1)
        .WillRepeatedly(Return(-1));

    ASSERT_EQ(-1, labjack_node.enableBody(false));
}

// Test setWatchdog()
TEST(LabjackNode, setWatchdog)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, enableWatchdog(_, _, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.setWatchdog());

    // fail
    EXPECT_CALL(mock, enableWatchdog(_, _, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setWatchdog());
}

// Test clearWatchdog()
TEST(LabjackNode, ClearWatchdog)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, disableWatchdog(_))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.clearWatchdog());

    // fail
    EXPECT_CALL(mock, disableWatchdog(_))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.clearWatchdog());
}

// Test isEmergency()
TEST(LabjackNode, IsEmergency)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.isEmergency());

    // fail
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(1, labjack_node.isEmergency());
}

// Test setEmergency()
TEST(LabjackNode, SetEmergency)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, writePulse(_, _, _, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, writeIO(_, _))
        .Times(2)
        .WillRepeatedly(Return(0));

    ASSERT_EQ(0, labjack_node.setEmergency());

    // fail
    EXPECT_CALL(mock, writePulse(_, _, _, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setEmergency());

    EXPECT_CALL(mock, writePulse(_, _, _, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, writeIO(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_EQ(-1, labjack_node.setEmergency());
}

// Test getVoltage()
TEST(LabjackNode, GetVoltage)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // success
    EXPECT_CALL(mock, readAI(_, _))
        .Times(1)
        .WillOnce(Return(0));

    ASSERT_EQ(0, labjack_node.getVoltage());

    // fail
    EXPECT_CALL(mock, readAI(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_FLOAT_EQ(-1, labjack_node.getVoltage());
}

// Test isPlugged()
TEST(LabjackNode, IsPlugged)
{
    MockLjacklmWrapper mock;

    EXPECT_CALL(mock, config())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readDIOs(_, _))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, readAIs(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(0));
    EXPECT_CALL(mock, readDirectionDs(_))
        .Times(1)
        .WillOnce(Return(0));

    LabjackNode labjack_node(&mock);

    // fail
    EXPECT_CALL(mock, readAI(_, _))
        .Times(1)
        .WillOnce(Return(-1));

    ASSERT_FALSE(labjack_node.isPlugged());
}

////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "labjack_node_tests");

    testing::InitGoogleMock(&argc, argv);

    return RUN_ALL_TESTS();
}
