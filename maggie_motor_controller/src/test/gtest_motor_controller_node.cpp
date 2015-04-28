/**
 * @file        gtest_motor_controller_node.cpp
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
#include "mcdc3006s_mock.h"
#include "motor_controller_node.h"

using testing::_;
using testing::DoAll;
using testing::SetArgumentPointee;
using testing::AnyNumber;
using testing::Return;

class CallbackCounter : public testing::Test {
    protected:
        // Remember that SetUp() is run immediately before a test starts.
        virtual void SetUp()
        {
            _client = _nh.serviceClient<maggie_motor_controller_msgs::MoveAbsPos>("move_abs_pos");
        }

        // TearDown() is invoked immediately after a test finishes.
        virtual void TearDown()
        {
            _client.shutdown();
        }

        ros::NodeHandle _nh;

        ros::ServiceClient _client;

    public:

};

// Test set_max_pos() method.
TEST_F(CallbackCounter, TestCounter_Node)
{
//    std_msgs::Int16 msg;
//    ros::Rate loop_rate(1);
//    msg.data = 0;
//
//    counter_pub_.publish(msg);
//    loop_rate.sleep();      // Give some time the message to arrive
//    ros::spinOnce();
//    EXPECT_EQ(0, this->get_count());
//
//    counter_pub_.publish(msg);
//    loop_rate.sleep();
//    ros::spinOnce();
//    EXPECT_EQ(1, this->get_count());
//
//    counter_pub_.publish(msg);
//    loop_rate.sleep();
//    ros::spinOnce();
//    EXPECT_EQ(2, this->get_count());
}

// Test set_max_pos()
TEST(MotorControllerNode, SetMaxPos)
{
    MockMcdc3006s mock;

//    EXPECT_CALL(mock, config())
//        .Times(1)
//        .WillOnce(Return(0));
//    EXPECT_CALL(mock, readDIOs(_, _))
//        .Times(1)
//        .WillOnce(Return(0));
//    EXPECT_CALL(mock, readAIs(_, _, _))
//        .Times(2)
//        .WillRepeatedly(Return(0));
//    EXPECT_CALL(mock, readDirectionDs(_))
//        .Times(1)
//        .WillOnce(Return(0));
//
//    LabjackNode labjack_node(&mock);
//
//    // success
//    EXPECT_CALL(mock, readDIOs(_, _))
//        .Times(1)
//        .WillOnce(Return(0));
//
//    ASSERT_FLOAT_EQ(0, labjack_node.getStateDs());
//
//    // fail
//    EXPECT_CALL(mock, readDIOs(_, _))
//        .Times(1)
//        .WillOnce(Return(-1));
//
//    ASSERT_EQ(-1, labjack_node.getStateDs());
}

////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller_node_tests");

    testing::InitGoogleMock(&argc, argv);

    return RUN_ALL_TESTS();
}
