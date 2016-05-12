/**
 * @file        test_manual_base_motors.cpp
 * @brief       Manual tests.
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

#include "base_motor_driver.h"
#include <iostream>

using namespace std;

int menu()
{
    int option = -1;

    cout << "1.  Set velocities" << endl;
    cout << "2.  Print data" << endl;
    cout << "3.  Move ahead a distance" << endl;
    cout << "4.  Turn an angle" << endl;
    cout << "5.  Move a distance" << endl;
    cout << "6.  Reset odometry" << endl;
    cout << "7.  Update odometry" << endl;
    cout << "8.  Enable motors" << endl;
    cout << "9.  Disable motors" << endl;
    cout << "10. Set estimate position" << endl;
    cout << "11. Get estimate position" << endl;
    cout << "0.  Quit" << endl;

    cout << "Choose option: " << endl;
    cin >> option;

    return option;
}

int main()
{
    BaseMotor bm;

    double x = 1000, y = 0, theta = 0;
    int linear_velocity = 0, angular_velocity = 0, distance = 0, angle = 0;
    cinematic_data *data;

    bm.init_communication();

    int option = -1;
    do {
        option = menu();

        switch(option) {
            case 1:
                cout << "Set linear velocity (degrees/sec): " << endl;
                cin >> linear_velocity;

                cout << "Set angular velocity (degrees/sec): " << endl;
                cin >> angular_velocity;

                if (bm.set_velocity(linear_velocity, angular_velocity) == 0) {
                    cout << "OK" << endl;
                }
                else {
                    cout << "ERROR" << endl;
                }

                break;

            case 2:
                if (bm.read_data(data) == 0) {
                    cout << "Linear velocity: " << data->v << endl;
                    cout << "Angular velocity: " << data->w << endl;
                    cout << "X position: " << data->x << endl;
                    cout << "Y position: " << data->y << endl;
                    cout << "Theta: " << data->theta << endl;
                }
                else {
                    cout << "ERROR" << endl;
                }

                break;

            case 3:
                cout << "Set distance (mm) to move ahead: " << endl;
                cin >> distance;

                if (bm.move_ahead(distance) == 0) {
                    cout << "OK" << endl;

                }
                else {
                    cout << "ERROR" << endl;
                }

                break;

            case 4:
                cout << "Set angle to turn: " << endl;
                cin >> angle;

                if (bm.relative_turn(angle) == 0) {
                    cout << "OK" << endl;
                }
                else {
                    cout << "ERROR" << endl;
                }

                break;

            case 5:
                cout << "Set linear velocity (mm/sec): " << endl;
                cin >> linear_velocity;

                cout << "Set angular velocity (degrees/sec): " << endl;
                cin >> angular_velocity;

                cout << "Set distance to move (mm): " << endl;
                cin >> distance;

                if (bm.set_displacement_velocity(distance, linear_velocity, angular_velocity) == 0) {
                    cout << "OK" << endl;
                }
                else {
                    cout << "ERROR" << endl;
                }

                break;

            case 6:
                bm.reset_odometry();

                break;

            case 7:
                cout << "x: " << endl;
                cin >> x;
                cout << "y: " << endl;
                cin >> y;
                cout << "theta: " << endl;
                cin >> theta;

                bm.update_odometry(x, y, theta);

                break;

            case 8:
                if (bm.enable_motors()) {
                    cout << "ERROR" << endl;
                }

                break;

            case 9:
                if (bm.disable_motors()) {
                    cout << "ERROR" << endl;
                }

                break;

            case 10:
                x = 0;
                y = 0;
                theta = 0;

                cout << "x: " << endl;
                cin >> x;
                cout << "y: " << endl;
                cin >> y;
                cout << "theta: " << endl;
                cin >> theta;

                bm.set_estimate_position(x, y, theta);

                break;

            case 11:
                data = bm.get_estimate_position();

                cout << "Velocidad Lineal: " << data->v << endl;
                cout << "Velocidad Angular: " << data->w << endl;
                cout << "Posicion estimada x: " << data->x << endl;
                cout << "Posicion estimada y: " << data->y << endl;
                cout << "Posicion angular estimada : " << data->theta << endl;

                break;
        }
    }
    while(option != 0);

    bm.end_communication();

    return 0;
}
