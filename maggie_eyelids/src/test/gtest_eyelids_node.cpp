/**
 * @file        gtest_eyelids_node.cpp
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
#include <maggie_serial_comm_drivers/serial_communication_mock.h>
#include "eyelids_node.h"

using testing::_;
using testing::DoAll;
using testing::SetArgumentPointee;
using testing::AnyNumber;
using testing::Return;

TEST(Eyelids, MoveFail_OpenPort)
{
    MockSerialCommunication mock;

    EXPECT_CALL(mock, set_serial_device(_))
        .Times(1);
    EXPECT_CALL(mock, open_port(_))
        .Times(1)
        .WillOnce(Return(-1));
    EXPECT_CALL(mock, close_port(_))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, send_character(_))
        .Times(AnyNumber())
        .WillRepeatedly(Return(-1));

    EyelidsNode eyelids_node(&mock);

    // fail case
    ASSERT_EQ(-1, eyelids_node.move(0));
}

TEST(Eyelids, MoveSuccess_ClosePort)
{
    MockSerialCommunication mock;

    EXPECT_CALL(mock, set_serial_device(_))
        .Times(1);
    EXPECT_CALL(mock, open_port(_))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, close_port(_))
        .Times(1)
        .WillOnce(Return(-1));
    EXPECT_CALL(mock, send_character(_))
        .Times(AnyNumber())
        .WillRepeatedly(Return(0));

    EyelidsNode eyelids_node(&mock);

    // fail case
    ASSERT_EQ(0, eyelids_node.move(0));
}

TEST(Eyelids, MoveFail_SendChar)
{
    MockSerialCommunication mock;

    EXPECT_CALL(mock, set_serial_device(_))
        .Times(1);
    EXPECT_CALL(mock, open_port(_))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, close_port(_))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, send_character(_))
        .Times(AnyNumber())
        .WillRepeatedly(Return(-1));

    EyelidsNode eyelids_node(&mock);

    // fail case
    ASSERT_EQ(-1, eyelids_node.move(0));
}

TEST(Eyelids, MoveSuccess)
{
    MockSerialCommunication mock;

    EXPECT_CALL(mock, set_serial_device(_))
        .Times(1);
    EXPECT_CALL(mock, open_port(_))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, close_port(_))
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, send_character(_))
        .Times(AnyNumber())
        .WillRepeatedly(Return(0));

    EyelidsNode eyelids_node(&mock);

    // success cases
    for (int j = 0; j < 255; j += 50) {
        ASSERT_EQ(0, eyelids_node.move(j));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eyelids_tests");

    ::testing::InitGoogleMock(&argc, argv);

    return RUN_ALL_TESTS();
}
