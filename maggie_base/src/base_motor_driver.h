#ifndef __BASE_MOTOR_DRIVER_H__
#define __BASE_MOTOR_DRIVER_H__

/**
 * @file        base_motor_driver.h
 * @brief       Driver for the motors of the mobile base.
 *
 * Important data:
 *      Max velocity: 3600 rpm, with a reduction of 36:1 -> 100 rpm, to seconds is 1.667 rps.
 *      Wheel diameter: 163.9 mm, max velocity -> 163.9 mm * PI / rev * 1.667 rev/sec = 858.17 mm/seg
 *      Wheels distance: 30cm, max velocity of rotation -> 13 rad/sec or 745 degrees/sec
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-02
 * @author      Rafael Rivas Estrada <rafael@ula.ve>
 * @date        2006-05
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

#include <fcntl.h>      // open modes devices
#include <cmath>
#include <cstring>
#include <sys/sem.h>    // IPC semaphores
#include <termios.h>    // struct termios
#include <ros/ros.h>
#include "maggie_data.h"

// device names
//TODO: convert this to rosparam
#define DEVICE_NAME1    "/dev/motor_base2"
#define DEVICE_NAME2    "/dev/motor_base1"

// semaphore name
#define SEM_FILENAME    "/tmp/sem"
key_t SEMFDID;

// baud rate
#define BAUDRATE        B19200

// period between samples in seconds
#define PERIOD          0.060

// length for GST command response
#define SIZE_REPORT_GST 8

typedef struct {
        double v;       // linear velocity (mm / sec)
        double w;       // angular velocity (degrees / sec)
        double x;       // x position (mm)
        double y;       // y position (mm)
        double theta;   // angle (degrees)
} cinematic_data;

class BaseMotor {
    public:
        /**
         * @brief Constructor
         */
        BaseMotor();

        /**
         * @brief Destructor
         */
        ~BaseMotor();

        /**
         * @brief inicio de la comunicacion con los motores
         * @return 0 Inicializacion exitosa, en caso contrario:
         *         1 Error al abrir el puerto.
         *         2 Error al asignar los atributos al puerto.
         *         3 Error al crear el semaforo del puerto.
         */
        int init_communication();

        /**
         * @brief fin de la comunicacion con los motores
         * @return 0 Finalizacionxitosa, en caso contrario Error.
         */
        int end_communication();

        /**
         * @brief Habilita los motores
         * @return 0 operación exitosa,
         *         -1 error escribiendo o leyendo del puerto, \
         *         -2 número de intentos de la operación superados,
         *         -3 orden no correcta \
         *         -4 error con el semáforo
         */
        int enable_motors();

        /**
         * @brief Inhabilita los motores, es decir, no van a realizar ningún movimiento.
         * @return 0 operación exitosa,
         *         -1 error escribiendo o leyendo del puerto, \
         *         -2 número de intentos de la operación superados,
         *         -3 orden no correcta \
         *         -4 error con el semáforo
         */
        int disable_motors();

        /**
         * @brief Habilita o inhabilita el motor referido por el manejador que se la pasa como parámetro
         * @param fd manejador del motor sobre el que se va a operar
         * @param order comando que queremos enviar al driver para habilitarlo/deshabilitarlo(EN ó DI)
         * @return 0 operación exitosa,
         *         -1 error escribiendo o leyendo del puerto, \
         *         -2 número de intentos de la operación superados,
         *         -3 orden no correcta \
         *         -4 error con el semáforo
         */
        int enable_disable_motor(int *fd, const char *order);

        /**
         * @brief
         * @param data
         * @return
         */
        int read_data(cinematic_data *data);

        /**
         * @brief actualizar lectura velocidad lineal, velocidad angular, angulo y posicion
         * @param data puntero a la direcciones de memoria donde se encuentran los data.
         * @param time_since_last_call the time that has passed since the last call of this function, in seconds
         * @return 0 operacion exitosa.
         *         1 Error al adquirir semaforo.
         */
        int read_data_variable_time_diff(cinematic_data *data, double time_since_last_call);

        /**
         * @brief actualizar lectura velocidad lineal, velocidad angular, angulo y posicion
         * @param v Velocidad lineal en mm/seg
         * @param w Velocidad angular en grados/seg
         * @return 0 operacion exitosa.
         */
        int set_velocity(double v, double w);

        /**
         * @brief movimiento de una distancia en línea recta a una velocidad determinada
         * @param d distancia a recorrer en cm
         * @param v velocidad lineal en mm/seg
         * @param w velocidad angular en grados/seg
         * @return 0 operación exitosa
         */
        int set_displacement_velocity(double d, double v, double w);

        /**
         * @brief solicitud de movimiento en linea recta relativo
         * @param d distance
         * @return 0 operacion exitosa.
         */
        int move_ahead(double distance);

        /**
         * @brief solicitud de giro relativo
         * @param teta  angulo a girar en radianes.
         * @return  0 operacion exitosa.
         */
        int relative_turn(double theta);

        /**
         * @brief comando para resetear la odometria
         * @return 0 operación exitosa
         */
        int reset_odometry();

        /**
         * @brief comando para actualizar la odometria. Esto es si se tiene un loclaizador o controlador en lazo cerrado que
         *        corrige la odometria del  robot.
         * @return 0 operación exitosa
         */
        int update_odometry(double x, double y, double theta);

        /**
         * @brief
         * @param data
         * @return
         */
        cinematic_data * get_estimate_position();

        /**
         * @brief
         * @param x
         * @param y
         * @param theta
         * @return
         */
        int set_estimate_position(double x, double y, double theta);

        std::string motor_order;

    private:
        // file descriptors
        int _fd_motor1, _fd_motor2;

        // motors odometry
        double _distance1, _distance2, _before_distance1, _before_distance2;
        cinematic_data _current_cinematic, _last_cinematic;

        // aggregated variable for estimated position
        int _corrected_data;
        cinematic_data _current_estimated, _before_estimated;
};

#endif
