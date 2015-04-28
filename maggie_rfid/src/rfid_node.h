#ifndef __RFID_NODE_H__
#define __RFID_NODE_H__

/**
 * @file        rfid_node.h
 * @brief       Node for controlling the rfid.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
 * @author      Ana Corrales <>
 * @date        2011-07
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

#include <iostream>
#include <ros/ros.h>
#include <maggie_rfid_drivers/rfid_driver_interface.h>

// services
#include <maggie_rfid_msgs/RfidTag.h>
#include <maggie_rfid_msgs/WriteCard.h>

using namespace std;

class RfidNode {
    public:
        /**
         * @brief Parameterized constructor.
         * @param rfid_driver, the driver interface.
         */
        RfidNode(RfidDriverInterface *rfid_driver);

        /**
         * @brief Destructor.
         */
        ~RfidNode();

        /**
         * @brief Initialize the node.
         * @param
         * @return
         */
        void init();

        /**
         * @brief Spin the node.
         * @param
         * @return
         */
        void spin();

        //////////////////////////////////////////////////
        // services
        //////////////////////////////////////////////////

        /**
         * @brief Callback to manage the rfid.
         * @param req RfidTag type request.
         * @param resp RfidTag type response.
         * @return true if there are no errors.
         */
        bool write_card(maggie_rfid_msgs::WriteCard::Request & req, maggie_rfid_msgs::WriteCard::Response & resp);

        //////////////////////////////////////////////////
        // class methods
        //////////////////////////////////////////////////
        /**
         * @brief .
         * @return 0 if no error, -1 if error.
         */
        int read_card();

        /**
         * @brief Method to set the data in the struct.
         * @param card_data Data to set in the struct.
         */
        void set_data(card_data);

        /**
         * @brief Method to read data from a card.
         * @param num_blocks Number of blocks to read.
         * @return card_data
         */
        card_data read_tag_data(int num_blocks);

        /**
         * @brief Method to write the data in a card.
         * @param card_data Data to write in the card.
         */
        void write_tag_data(card_data card);

    private:
        /**
         * @brief Method to get the number of blocks with data in the card.
         * @return Number of blocks with data.
         */
        int get_num_blocks();

        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        ros::Publisher _write_pub;

        ros::ServiceServer _rfid_write_srv;

        ros::Rate _publish_rate;

        RfidDriverInterface *_rfid_driver;
        card_data _rfid_card;

        int _num_device;
};

#endif
