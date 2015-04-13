/**
 * @file        rfid_node.cpp
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

#include "rfid_node.h"

//////////////////////////////////////////////////

RfidNode::RfidNode(RfidDriverInterface *rfid_driver) :
    _nh_private("~"), _publish_rate(5), _rfid_driver(rfid_driver)
{
    // get ros params
    std::string searched_param;

    // find the dev port name
    _nh_private.searchParam("num_device", searched_param);
    _nh_private.param(searched_param, _num_device, HEAD_READER_ID);

    // connect with the driver
    if (_rfid_driver->open_device(_num_device) != 0) {
        ROS_ERROR("[RFID_NODE] Error connecting to driver. Device: %d", _num_device);
        exit(-1);
    }

    // set the maximum number of labels to detect in each inventory, default 1
    _rfid_driver->set_num_labels(1);
}

//////////////////////////////////////////////////

RfidNode::~RfidNode()
{
    // disconnect the driver
    if (_rfid_driver->close_device() != 0) {
        ROS_ERROR("[RFID_NODE] Error disconnecting to the driver");
    }
}

//////////////////////////////////////////////////

void RfidNode::init()
{
    _write_pub = _nh_private.advertise<rfid_msgs::RfidTag>("rfid_write", 1);
    _rfid_write_srv = _nh_private.advertiseService("rfid_write", &RfidNode::write_card, this);
}

//////////////////////////////////////////////////

void RfidNode::spin()
{
    while(_nh.ok()) {
        ros::spinOnce();
        _publish_rate.sleep();

        // detect and read rfid cards
        read_card();
    }
}

//////////////////////////////////////////////////

bool RfidNode::write_card(rfid_msgs::WriteCard::Request & req, rfid_msgs::WriteCard::Response & resp)
{
    // copy data from message
    card_data card;

    for (int i = 0; i < req.card.id.size(); ++i) {
        card.UID[i] = req.card.id[i];
    }

    card.data.length = req.card.data.size();

    for (int i = 0; i < req.card.data.size(); ++i) {
        card.data.value[i] = req.card.data[i];
    }

    // check type of reader
    if (strcmp(_rfid_driver->get_type_reader().c_str(), HF_NAME) == 0) {
        // write data
        write_tag_data(card);

        // wait for writing
        sleep(3);
    }
    else {
        ROS_ERROR("[RFID_NODE] Error: cannot write rfid card");

        return false;
    }

    return true;
}

//////////////////////////////////////////////////

int RfidNode::read_card()
{
    int error = 0;
    int num_cards = _rfid_driver->inventory();

    rfid_msgs::RfidTag msg;

    // if there are cards near, read them
    if (num_cards > 0) {
        // check type of reader
        if (strcmp(_rfid_driver->get_type_reader().c_str(), HF_NAME) == 0) {
            // get the uid label
            string id = _rfid_driver->get_uid_label();

            if (id != "0") {
                set_data(read_tag_data(get_num_blocks()));

                // publish data
                ROS_INFO("[RFID_NODE] RFID id: %s", _rfid_card.UID);
                ROS_INFO("[RFID_NODE] RFID data (size: %d): %s", _rfid_card.data.length, _rfid_card.data.value);

                // fill the message
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "/rfid";
                for (int i = 0; i < strlen(_rfid_card.UID); ++i) {
                    msg.id.push_back(_rfid_card.UID[i]);
                }
                for (int i = 0; i < strlen((char *) _rfid_card.data.value); ++i) {
                    msg.data.push_back(_rfid_card.data.value[i]);
                }

                _write_pub.publish(msg);
            }
        }
        else {
            ROS_ERROR("[RFID_NODE] Error: Unknown RFID reader");
        }
    }
    else {
        ROS_DEBUG("[RFID_NODE] No cards detected");

        error = -1;
    }

    return error;
}

//////////////////////////////////////////////////

void RfidNode::set_data(card_data card)
{
    bcopy((const void*) &card, (void*) &_rfid_card, sizeof(card_data));
}

//////////////////////////////////////////////////

card_data RfidNode::read_tag_data(int num_blocks)
{
    unsigned char data_aux[MAX_LENGTH] = "";
    unsigned char data_read[BLOCK_SIZE_HF];

    int iterator = 0;
    for (int i = 0; i <= num_blocks; i++) {
        _rfid_driver->read_hf(data_read, (unsigned char) i);

        for (int j = 0; j < BLOCK_SIZE_HF; j++) {
            data_aux[iterator + j] = data_read[j];
        }

        iterator += BLOCK_SIZE_HF;
    }

    _rfid_card.data.length = strlen((char*) data_aux);

    for (int i = 0; i < MAX_LENGTH; i++) {
        _rfid_card.data.value[i] = data_aux[i];
    }
    ROS_DEBUG("[RFID_NODE] HF data read: %s", _rfid_card.data.value);

    // get the uid label
    string ID_aux = _rfid_driver->get_uid_label();
    int len = ID_aux.length();

    char* ID_tag = new char[len + 1];
    ID_aux.copy(ID_tag, len, 0);
    ID_tag[len] = '\0';

    strcpy(_rfid_card.UID, ID_tag);

    return _rfid_card;
}

//////////////////////////////////////////////////

void RfidNode::write_tag_data(card_data card)
{
    int dir = 0;
    unsigned char datos_aux[BLOCK_SIZE_HF];
    string uid = "";

    // wait until read at least a card
    while(_rfid_driver->inventory() == 0) {
        ROS_INFO("[RFID_NODE] Waiting card to write");
    }

    // get the uid label
    uid = _rfid_driver->get_uid_label();
    int len = strlen((char*) card.data.value);

    // write data
    int i = 0;
    while(i < len) {
        for (int j = 0; j < BLOCK_SIZE_HF; j++) {
            if (i < len) {
                datos_aux[j] = card.data.value[i];
                i++;
            }
        }
        _rfid_driver->write_hf(datos_aux, (unsigned char) dir, uid);
        dir++;
    }
}

//////////////////////////////////////////////////

int RfidNode::get_num_blocks()
{
    int num_blocks = 0;
    unsigned char data[BLOCK_SIZE_HF] = "";

    _rfid_driver->read_hf(data, (unsigned char) 0);

    num_blocks = atoi((char*) data);

    return num_blocks;
}

//////////////////////////////////////////////////
