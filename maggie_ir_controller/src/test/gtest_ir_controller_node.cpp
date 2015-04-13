/**
 * @file        gtest_ir_controller_node.cpp
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

#include "gtest/gtest.h"
#include "irtrans_mock.h"
#include "ir_controller_node.h"

using testing::_;
using testing::DoAll;
using testing::SetArgumentPointee;
using testing::Return;

TEST(IrController, Connect_Disconnect)
{
    MockIrTransWrapper mock;

    EXPECT_CALL(mock, connect())
        .Times(1)
        .WillOnce(Return(0));
    EXPECT_CALL(mock, disconnect())
        .Times(1);

    IRControllerNode ir_controller_node(&mock);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ir_controller_node_tests");

    ::testing::InitGoogleMock(&argc, argv);

    return RUN_ALL_TESTS();
}
