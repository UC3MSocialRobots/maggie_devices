/**
 * @file        test_manual_primitives_labjack.cpp
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

#include <iostream>
#include "ljacklm_wrapper.h"

using namespace std;

int main(int argc, char **argv)
{
    int op = 1;
    long error = 0;
    float v, v0, v1;
    int ch;
    long n, m, cd, cio;
    short i;
    long *lineas;
    float *voltajes;
    char stringError[50];

    LabjackDriverInterface * lj = new LjacklmWrapper();

    if ((error = lj->config()) != 0) {
        GetErrorString(error, stringError);
        printf("Error configuring Labjack. %s\n", stringError);
        return -1;
    }

    while(op != 0) {
        cout << "\n Options: \n";
        cout << "\t 1.Reset\n";
        cout << "\t 2.getAI\n";
        cout << "\t 3.setAI\n";
        cout << "\t 4.getD\n";
        cout << "\t 5.setD\n";
        cout << "\t 6.getIO\n";
        cout << "\t 7.setIO\n";
        cout << "\t 8.getDIOs\n";
        cout << "\t 9.setDIOs\n";
        cout << "\t 10.set&confDIOs\n";
        cout << "\t 11.enableWatchdog\n";
        cout << "\t 12.disableWatchdog\n";
        cout << "\t 13.getAIs\n";
        cout << "\t 14.setAOs\n";
        cout << "\t 15.setPulso\n";
        cout << "\t 16.getDireccionDs\n";
        cout << "\t 17.setD&confDIOs\n";
        cout << "\t 18.setDIOsCONFIG\n";
        cout << "\t 0.FIN\n";
        cin >> op;

        switch(op) {
            case 0:
                printf("End communication with labjack\n");
                break;
            case 1:
                printf("Reseting labjack...");

                if ((error = lj->reset()) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error reseting Labjack. %s\n", stringError);
                }
                break;
            case 2:
                printf("Reading AnalogInput...\n");

                printf("Channel(0..7) = ");
                cin >> ch;

                if ((error = lj->readAI((long) ch, &v)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in readAI. %s\n", stringError);
                }
                else
                    printf("Voltage in AI%d=%f\n", ch, v);
                break;
            case 3:
                printf("Writing AnalogOutput..\n");

                printf("Channel(0..2) = ");
                cin >> ch;
                printf("Voltage(0.0..5.0) = ");
                cin >> v;

                if ((error = lj->writeAO(ch, v)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeAI. %s\n", stringError);
                }
                break;
            case 4:
                printf("Reading DigitalInput...\n");

                printf("Channel(0..15) = ");
                cin >> ch;

                if ((error = lj->readD((long) ch, &n)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in readD. %s\n", stringError);
                }
                else
                    printf("Level in D%d=%li\n", ch, n);
                break;
            case 5:
                printf("Writing DigitalInput..\n");

                printf("Channel(0..15) = ");
                cin >> ch;
                printf("Level(0-1) = ");
                cin >> n;

                if ((error = lj->writeD(ch, n)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeD. %s\n", stringError);
                }
                break;
            case 6:
                printf("Reading digital InputOutput...\n");

                printf("Channel(0..3) = ");
                cin >> ch;

                if ((error = lj->readIO((long) ch, &n)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in readIO. %s\n", stringError);
                }
                else
                    printf("Level in IO%d=%li\n", ch, n);
                break;
            case 7:
                printf("Writing digital InputOutput..\n");

                printf("Channel(0..3) = ");
                cin >> ch;
                printf("Level(0-1) = ");
                cin >> n;

                if ((error = lj->writeIO(ch, n)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeIO. %s\n", stringError);
                }
                break;
            case 8:
                printf("Reading D and IO...\n");

                if ((error = lj->readDIOs(&n, &m)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in readDIOs. %s\n", stringError);
                }
                else {
                    printf("Levels D = %li | %li\n", n, n);
                    printf("Levels IO = %li | %li\n", m, m);
                }
                break;
            case 9:
                printf("Writing a D y IO...\n");

                printf("Level of signals D(binary: 0=level low, 1=level high) = ");
                cin >> n;
                printf("Level of signals IO(binary: 0=level low, 1=level high) = ");
                cin >> m;

                if ((error = lj->writeDIOsCONFIG(&n, &m)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeDIOsCONFIG. %s\n", stringError);
                }
                else {
                    printf("Levels D = %li | %li\n", n, n);
                    printf("Levels IO = %li | %li\n", m, m);
                }
                break;
            case 10:
                printf("Writing and configuring D and IO...\n");

                printf("Direction of Lines D (binary: 0=input, 1=output) = ");
                cin >> cd;
                printf("Direction of Lines IO (binary: 0=input, 1=output) = ");
                cin >> cio;
                printf("Level of signals D(binary: 0=level low, 1=level high) = ");
                cin >> n;
                printf("Level of signals IO(binary: 0=level low, 1=level high) = ");
                cin >> m;

                if ((error = lj->writeDIOs(&cd, &cio, &n, &m)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeDIOs. %s\n", stringError);
                }
                else {
                    printf("Direction D = %li | %li\n", cd, cd);
                    printf("Levels D = %li | %li\n", n, n);
                    printf("Levels IO = %li | %li\n", m, m);
                }
                break;
            case 11:
                printf("Enabling Watchdog...\n");

                printf("Timeout(1..715) = ");
                cin >> m;
                printf("Line D(0,1,8) = ");
                cin >> ch;
                printf("Level D%d(0-1) = ", ch);
                cin >> n;

                if ((error = lj->enableWatchdog(m, ch, n)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in setWatchdog. %s\n", stringError);
                }
                break;
            case 12:
                printf("Disabling Watchdog...\n");

                printf("Line D(0,1,8) = ");
                cin >> ch;

                if ((error = lj->disableWatchdog(ch)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in unsetWatchdog. %s\n", stringError);
                }
                break;
            case 13:
                printf("Reading Analog Inputs...\n");

                printf("Number of Lines to read (1, 2, 4) = ");
                cin >> n;
                lineas = (long *) calloc(n, sizeof(long));
                voltajes = (float *) calloc(n, sizeof(float));
                for (i = 0; i < n; i++) {
                    printf("Lines %d AI (0..7) = ", i + 1);
                    cin >> lineas[i];
                }

                if ((error = lj->readAIs(n, lineas, voltajes)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in readAIs. %s\n", stringError);
                }
                else {
                    for (i = 0; i < n; i++) {
                        printf("Voltage in AI%li=%f\n", lineas[i], voltajes[i]);
                    }
                }
                free(lineas);
                free(voltajes);
                break;
            case 14:
                printf("Writing Analog Outs...\n");

                printf("Voltage AO0 = ");
                cin >> v0;
                printf("Voltage AO1 = ");
                cin >> v1;

                if ((error = lj->writeAOs(v0, v1)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeAOs. %s\n", stringError);
                }
                break;
            case 15:
                printf("Writing Pulses...\n");

                printf("Level of pulse (<=0 -> low, >0 -> high) = ");
                cin >> n;
                printf("Type Line (0->Lines D, 1->Lines IO) = ");
                cin >> cd;
                if (cd == 0)
                    printf("Lines D (0..15) = ");
                else
                    printf("Lines IO (0..3)");
                cin >> m;
                printf("Pulse duration [us] (1..2000000) = ");
                cin >> cio;

                if ((error = lj->writePulse(n, m, cio, cd)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writePulse. %s\n", stringError);
                }
                break;
            case 16:
                printf("Reading D directions...\n");

                if ((error = lj->readDirectionDs(&m)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in readDirectionDs. %s\n", stringError);
                }
                else
                    printf("Direction Ds = %li | %li\n", m, m);
                break;
            case 17:
                printf("Writing D...\n");

                printf("Channel(0..15) = ");
                cin >> m;
                printf("Level(0-1) = ");
                cin >> n;

                if ((error = lj->writeDCONFIG(m, n)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeDCONFIG. %s\n", stringError);
                }
                break;
            case 18:
                // establecemos la configuración por defecto of Lines y les asignamos un level
                printf("Writing D and IO\n");

                long stateD = CONFIG_D; //level of Lines D
                long stateIO = CONFIG_IO; //level of Lines IO

                if ((error = lj->writeDIOsCONFIG(&stateD, &stateIO)) != 0) {
                    GetErrorString(error, stringError);
                    printf("Error in writeDIOsCONFIG. %s\n", stringError);
                }
                break;
        }
    }

    delete lj;

    return 0;
}
