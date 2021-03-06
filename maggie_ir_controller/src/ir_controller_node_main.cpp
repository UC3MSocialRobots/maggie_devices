/**
 * @file        ir_controller_node_main.cpp
 * @brief       Main of the node.
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

#include <maggie_ir_drivers/irtrans_wrapper.h>
#include "ir_controller_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ir_controller_node");

    IrDriverInterface *ir_driver = new IrTransWrapper();

    IRControllerNode node(ir_driver);

    node.init();
    node.spin();

    delete ir_driver;

    return 0;
}
