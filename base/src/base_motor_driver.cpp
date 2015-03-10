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

#include "base_motor_driver.h"

//////////////////////////////////////////////////

BaseMotor::BaseMotor() :
    motor_order("M\n\r"), _corrected_data(0), _distance1(0.0), _distance2(0.0), _before_distance1(0.0),
    _before_distance2(0.0), _fd_motor1(-1), _fd_motor2(-1)
{

}

//////////////////////////////////////////////////

BaseMotor::~BaseMotor()
{
}

//////////////////////////////////////////////////

int BaseMotor::init_communication()
{
    int result;
    struct termios newtio1, newtio2;
    char DEFINE0[] = "HO\n\r";

    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};

    // Semaforo para el puerto de comuniciaciones
    int semFD;

    _fd_motor1 = open(DEVICE_NAME1, O_RDWR | O_NOCTTY);
    if (_fd_motor1 < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error while opening puerto serie 0 del motor %s:'%s'", DEVICE_NAME1,
                 strerror(errno));
        return (1);
    }

    _fd_motor2 = open(DEVICE_NAME2, O_RDWR | O_NOCTTY);
    if (_fd_motor2 < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error while opening puerto serie 0 del motor %s:'%s'", DEVICE_NAME2,
                 strerror(errno));
        return (1);
    }

    // MOTOR 1
    bzero((void *) &newtio1, sizeof(newtio1));

    // Velocidad, paridad, stop, bit, echo ..
    newtio1.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio1.c_iflag = IGNPAR | ICRNL;
    newtio1.c_oflag = 0;
    newtio1.c_lflag = ECHO | ICANON;

    // Caracteres de control
    newtio1.c_cc[VINTR] = 0;
    newtio1.c_cc[VQUIT] = 0;
    newtio1.c_cc[VERASE] = 0;
    newtio1.c_cc[VKILL] = 0;
    newtio1.c_cc[VEOF] = 4;
    newtio1.c_cc[VSWTC] = 0;
    newtio1.c_cc[VSTART] = 0;
    newtio1.c_cc[VSTOP] = 0;
    newtio1.c_cc[VSUSP] = 0;
    newtio1.c_cc[VEOL] = 0;
    newtio1.c_cc[VREPRINT] = 0;
    newtio1.c_cc[VDISCARD] = 0;
    newtio1.c_cc[VWERASE] = 0;
    newtio1.c_cc[VLNEXT] = 0;
    newtio1.c_cc[VEOL2] = 0;

    // Bloqueo por caracter con tiempo de espera infinito.
    newtio1.c_cc[VTIME] = 0;
    newtio1.c_cc[VMIN] = 1;

    // Activacion de configuracion.
    tcflush(_fd_motor1, TCIFLUSH);
    result = tcsetattr(_fd_motor1, TCSANOW, &newtio1);

    if (result) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Configuando puerto serie del motor %s:'%s'", DEVICE_NAME1, strerror(errno));
        return (2);
    }

    // MOTOR 2
    bzero((void *) &newtio2, sizeof(newtio2));

    // Velocidad, paridad, stop, bit, echo ..
    newtio2.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio2.c_iflag = IGNPAR | ICRNL;
    newtio2.c_oflag = 0;
    newtio2.c_lflag = ECHO | ICANON;

    // Caracteres de control
    newtio2.c_cc[VINTR] = 0;
    newtio2.c_cc[VQUIT] = 0;
    newtio2.c_cc[VERASE] = 0;
    newtio2.c_cc[VKILL] = 0;
    newtio2.c_cc[VEOF] = 4;
    newtio2.c_cc[VSWTC] = 0;
    newtio2.c_cc[VSTART] = 0;
    newtio2.c_cc[VSTOP] = 0;
    newtio2.c_cc[VSUSP] = 0;
    newtio2.c_cc[VEOL] = 0;
    newtio2.c_cc[VREPRINT] = 0;
    newtio2.c_cc[VDISCARD] = 0;
    newtio2.c_cc[VWERASE] = 0;
    newtio2.c_cc[VLNEXT] = 0;
    newtio2.c_cc[VEOL2] = 0;

    // Bloqueo por caracter con tiempo de espera infinito.
    newtio2.c_cc[VTIME] = 0;
    newtio2.c_cc[VMIN] = 1;

    // Activacion de configuracion.
    tcflush(_fd_motor2, TCIFLUSH);
    result = tcsetattr(_fd_motor2, TCSANOW, &newtio2);

    if (result) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Configuando puerto serie del motor %s:'%s'", DEVICE_NAME2, strerror(errno));
        return (2);
    }

    // create the key SEMFDID for the semaphore
    int returnValue = access(SEM_FILENAME, F_OK);
    if (returnValue != 0) {
        ROS_DEBUG("[BASE_MOTOR_DRIVER] access() returned an error %i='%s'", returnValue, strerror(errno));
        if (creat(SEM_FILENAME, S_IRUSR | S_IWUSR) == -1) {
            ROS_INFO("[BASE_MOTOR_DRIVER] ACCESS:Error creando el fichero de claves:'%s'", strerror(errno));
        }
    }
    SEMFDID = ftok(SEM_FILENAME, 'S');
    if (SEMFDID == (key_t) -1) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Errro while creating the key SEMFDID:'%s'", strerror(errno));
        exit(EXIT_FAILURE);
    }

    // Creacion del semaforo de control de acceso del puerto de comunicaciones.
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Creacion del semaforo de control de acceso del puerto de comunicaciones:SEMFDID:%i",
              SEMFDID);
    semFD = semget(SEMFDID, 1, IPC_CREAT | 0666);
    if (semFD < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Errro while creating el semaforo para el puerto de comunicaciones:semFD:%i, '%s'",
                 semFD, strerror(errno));
        return (3);
    }

    returnValue = semctl(semFD, 0, SETVAL, 1);
    if (returnValue != 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error while semctl:semFD:%i, '%s'", semFD, strerror(errno));
    }

    bzero((void *) &_current_cinematic, sizeof(cinematic_data));
    bzero((void *) &_last_cinematic, sizeof(cinematic_data));
    bzero((void *) &_current_estimated, sizeof(cinematic_data));
    bzero((void *) &_before_estimated, sizeof(cinematic_data));

    _distance1 = _distance2 = _before_distance1 = _before_distance2 = 0.0;

    // INCIO ZONA EXCLUSIVA

    semctl(semFD, 0, SETVAL, 1);

    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en inicioComunicacionMotores: %d", semctl( semFD, 0,GETVAL,0));

    semop(semFD, sem_in, 1);

    write(_fd_motor1, DEFINE0, 4);
    write(_fd_motor2, DEFINE0, 4);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", DEFINE0, DEFINE0);

    write(_fd_motor1, ACCELERATION, 6);
    write(_fd_motor2, ACCELERATION, 6);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", ACCELERATION, ACCELERATION);

    write(_fd_motor1, DECELERATION, 7);
    write(_fd_motor2, DECELERATION, 7);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", DECELERATION, DECELERATION);

    write(_fd_motor1, VELOCITY_MAX, 8);
    write(_fd_motor2, VELOCITY_MAX, 8);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", VELOCITY_MAX, VELOCITY_MAX);

#ifdef DEBUG_ODO
    char buf1[64];
    int res;
    int i;
    static const char POSM1[] = "GSP\n\r";
    bzero((void *) buf1, 64);
    write(_fd_motor1, POSM1, 5);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Leido: ");
    i = 0;
    res = read(_fd_motor1, buf1, 62);
    read(_fd_motor1, &buf1[63], 1);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Velocidad máxima :'%s':%d", buf1, res);
#endif

    // FIN ZONA EXCLUSIVA
    semop(semFD, sem_out, 1);

    // El puerto acepta la configuracion y el semaforo esestavo.
    return (0);
}

//////////////////////////////////////////////////

int BaseMotor::end_communication()
{
    int semFD;
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf arg[] = {0, 0, 0};
    char buf[] = "V0\n\r";

    if ((semFD = semget(SEMFDID, 1, 0)) == -1) {
        ROS_INFO("[BASE_MOTOR_DRIVER] finComunicacionesMotores semget:'%s'", strerror(errno));
        return (1);
    }

    // INCIO ZONA EXCLUSIVA
    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en finComunicacionMotores: %d", semctl( semFD, 0,GETVAL,0));
    semop(semFD, sem_in, 1);

    write(_fd_motor1, buf, 4);
    write(_fd_motor2, buf, 4);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", buf, buf);

    // FIN ZONA EXCLUSIVA

    close(_fd_motor1);
    close(_fd_motor2);

    /* remove it: */
    if (semctl(semFD, 0, IPC_RMID, arg) == -1) {
        ROS_INFO("[BASE_MOTOR_DRIVER] finComunicacionesMotores semctl:'%s'", strerror(errno));

        return (2);
    }
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Saliendo de fin comunicaciones");

    return 0;
}

//////////////////////////////////////////////////

int BaseMotor::enable_motors()
{
    int r;
    if ((r = enable_disable_motor(&_fd_motor1, "EN\r")) != 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error habilitando motor 1! Error %d", r);
        return r;
    }
    if ((r = enable_disable_motor(&_fd_motor2, "EN\r")) != 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error habilitando motor 2! Error %d", r);
        return r;
    }

    return 0;
}

//////////////////////////////////////////////////

int BaseMotor::disable_motors()
{
    int r;
    if ((r = enable_disable_motor(&_fd_motor1, "DI\r")) != 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error inhabilitando motor 1! Error %d", r);
        return r;
    }
    if ((r = enable_disable_motor(&_fd_motor2, "DI\r")) != 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error inhabilitando motor 2! Error %d", r);
        return r;
    }

    return 0;
}

//////////////////////////////////////////////////

int BaseMotor::enable_disable_motor(int *fd, const char *order)
{
    char bufOut[64], bufIn[64];
    // size 4 because commands to send: EN\n\r or DI\n\r
    char comando[4];
    const char comandoGST[] = "GST\n\r";
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};
    int res, l, semFD;
    // times to try to send the command
    int intentos = 3;
    char c;

    if (strlen(order) > 3) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error en la longitud del comando EN/DI");
        return -3;
    }
    bzero((void *) comando, 4);
    strcat(comando, order);
    if (comando[2] != '\r') {
        comando[2] = '\r';
    }
    l = strlen(comando);

    bzero((void *) bufOut, 64);
    bzero((void *) bufIn, 64);

    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error adquiriendo el semaforo en enable_disable_motor:'%s'", strerror(errno));
        return (-4);
    }

    while(intentos >= 0) {
        intentos--;
        l = strlen(comando);

        // escribimos el comando al driver
        if (semop(semFD, sem_in, 1) != 0) {
            ROS_INFO("[BASE_MOTOR_DRIVER] Error inicio zona exclusiva write enable_disable_motor -> (%i) \"'%s'\"",
                     errno, strerror(errno));
            return -4;
        }

        res = write(*fd, comando, l);
        if (res != l) {
            ROS_INFO("[BASE_MOTOR_DRIVER] Error escribiendo a _fd_motor1: escritos %d bytes de %d", res, l);
            ROS_DEBUG("[BASE_MOTOR_DRIVER] (%i) \"'%s'\"/n/n", errno, strerror(errno));
            continue;
        }

        // nos aseguramos que lo ha escrito correctamente preguntando al driver su estado...
        l = strlen(comandoGST);
        res = write(*fd, comandoGST, l);
        if (res != l) {
            ROS_INFO("[BASE_MOTOR_DRIVER] Error escribiendo a _fd_motor1: escritos %d bytes de %d", res, l);
            ROS_DEBUG("[BASE_MOTOR_DRIVER] (%i) \"'%s'\"", errno, strerror(errno));
            continue;
        }

        // reading the answer

        res = read(*fd, bufIn, 62);
        read(*fd, &bufIn[63], 1);
        if (semop(semFD, sem_out, 1) != 0) {
            ROS_INFO("[BASE_MOTOR_DRIVER] Error fin zona exclusiva read enable_disable_motor -> (%i) \"'%s'\"", errno,
                     strerror(errno));
            return -4;
        }

        if (res < SIZE_REPORT_GST) {
            ROS_INFO("[BASE_MOTOR_DRIVER] Error leyendo a fd: leidos %d bytes", res);
            ROS_DEBUG("[BASE_MOTOR_DRIVER] (%i) \"'%s'\"", errno, strerror(errno));
            continue;
        }

        // en c se guarda el estado del driver
        c = bufIn[3];
        // encuentra "EN" => hemos mandado habilitar el motor
        if (strstr(comando, "EN") != NULL) {
            // operación realizada correctamente
            if (c == '1') {
                return 0;
            }
            else {
                ROS_DEBUG("[BASE_MOTOR_DRIVER] Intento fallido. Restan %d intentos", intentos);
            }
        }
        // encuentra "DI" => hemos mandado inhabilitar el motor
        else if (strstr(comando, "DI") != NULL) {
            // operación realizada correctamente
            if (c == '0') {
                return 0;
            }
            else {
                ROS_DEBUG("[BASE_MOTOR_DRIVER] Intento fallido. Restan %d intentos", intentos);
            }
        }
        // hemos mandado una cosa rara
        else {
            ROS_INFO("[BASE_MOTOR_DRIVER] Error comprobando el éxito de nuestro comando inapropiado: \"'%s'\"",
                     comando);
            return -3;
        }
    }

    ROS_DEBUG("[BASE_MOTOR_DRIVER] ¡INTENTOS AGOTADOS!");
    return -2;
}

//////////////////////////////////////////////////

int BaseMotor::read_data(cinematic_data* data)
{
    return read_data_variable_time_diff(data, PERIOD);
}

//////////////////////////////////////////////////

int BaseMotor::read_data_variable_time_diff(cinematic_data *data, double time_since_last_call)
{
    static const char GNM1[] = "GN\n\r";
    static const char GNM2[] = "GN\n\r";
    static const char POSM1[] = "POS\n\r";
    static const char POSM2[] = "POS\n\r";

    char buf1[64], buf2[64], buf3[64], buf4[64];

    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};

    double v, w, deltaD1, deltaD2, deltaD, deltaTheta;
    int semFD;

    bzero((void *) buf1, 64);
    bzero((void *) buf2, 64);
    bzero((void *) buf3, 64);
    bzero((void *) buf4, 64);

    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Adquiriendo el semaforo en leeDatosMotores:'%s'", strerror(errno));
        return (1);
    }

    // INCIO ZONA EXCLUSIVA
    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en leeDatosMotores: %d", semctl( semFD, 0,GETVAL,0));
    int returnValue = semop(semFD, sem_in, 1);
    if (returnValue != 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error while semop:'%s'", strerror(errno));
    }

    // POS
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Sending POSM1:'%s'", POSM1);
    write(_fd_motor1, POSM1, 5);

    // buf1 :ticks
    read(_fd_motor1, buf1, 62);
    read(_fd_motor1, &buf1[63], 1);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Received buf1:'%s'", buf1);

    ROS_DEBUG("[BASE_MOTOR_DRIVER] Sending POSM2:'%s'", POSM2);
    write(_fd_motor2, POSM2, 5);

    // buf2 :ticks
    read(_fd_motor2, buf2, 62);
    read(_fd_motor2, &buf2[63], 1);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Received buf2:'%s'", buf2);

    // VELOCIDAD
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Sending GNM1:'%s'", GNM1);
    write(_fd_motor1, GNM1, 4);

    // buf3 :RPM (radians / sec)
    read(_fd_motor1, buf3, 62);
    read(_fd_motor1, &buf3[63], 1);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Received buf3:'%s'", buf3);

    ROS_DEBUG("[BASE_MOTOR_DRIVER] Sending GNM2:'%s'", GNM2);
    write(_fd_motor2, GNM2, 4);

    // buf4 :RPM (radians / sec)
    read(_fd_motor2, buf4, 62);
    read(_fd_motor2, &buf4[63], 1);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Received buf4:'%s'", buf4);

    semop(semFD, sem_out, 1);

    // PULSES_PER_REVOLUTION                 ticks -> 1 rotation of motor
    // so PULSES_PER_REVOLUTION * REDUCTION  ticks -> 1 rotation of wheel ( 2 PI )
    // so if we have buf1 ticks, the wheel angle is
    // angle = (buf1 * 2 PI) / (PULSES_PER_REVOLUTION * REDUCTION)

    // and so the distance is
    // d = angle * radius
    //   = (buf1 * 2 PI) / (PULSES_PER_REVOLUTION * REDUCTION) * radius
    //   = (diameter * buf1 * PI) / (PULSES_PER_REVOLUTION * REDUCTION)
    _before_distance1 = _distance1;
    // dimension check: (tick * no_dim * mm) / (tick * no_dim * no_dim) = mm
    _distance1 = (atof(buf1) * M_PI * WHEEL_DIAMETER) / (PULSES_PER_REVOLUTION * REDUCTION);
    _before_distance2 = _distance2;
    _distance2 = (atof(buf2) * M_PI * WHEEL_DIAMETER) / (PULSES_PER_REVOLUTION * REDUCTION);

    deltaD1 = _distance1 - _before_distance1; // mm
    deltaD2 = _distance2 - _before_distance2; // mm

    deltaD = (deltaD1 + deltaD2) * 0.5; // mm
    deltaTheta = (deltaD2 - deltaD1) / (AXIS_LENGTH); // no_dim

    // mm . sec-1
    v = deltaD / time_since_last_call;

    // rad. sec-1
    w = deltaTheta / time_since_last_call;

    _current_cinematic.theta = _last_cinematic.theta + deltaTheta;
    _current_cinematic.x = _last_cinematic.x + deltaD * cos(_current_cinematic.theta);
    _current_cinematic.y = _last_cinematic.y + deltaD * sin(_current_cinematic.theta);
    _current_cinematic.v = v;
    _current_cinematic.w = w;

    ROS_DEBUG("[BASE_MOTOR_DRIVER] x:%.4f mm   y:%.4f mm   theta:%.4f rad   v:%.4f  mm.sec-1   w:%.4f rad.sec-1",
              _current_cinematic.x, _current_cinematic.y, _current_cinematic.theta, _current_cinematic.v,
              _current_cinematic.w);

    // copy actual -> anterior
    _last_cinematic.theta = _current_cinematic.theta;
    _last_cinematic.x = _current_cinematic.x;
    _last_cinematic.y = _current_cinematic.y;
    _last_cinematic.v = _current_cinematic.v;
    _last_cinematic.w = _current_cinematic.w;

    // copy actual -> data
    data->theta = _current_cinematic.theta;
    data->x = _current_cinematic.x;
    data->y = _current_cinematic.y;
    data->v = _current_cinematic.v;
    data->w = _current_cinematic.w;

    // convertimos a grados
    data->w = data->w * 360.0 * (0.5) / M_PI; // deg . sec-1
    data->theta = data->theta * 360.0 * (0.5) / M_PI; // deg

    ROS_DEBUG("[BASE_MOTOR_DRIVER] data: x=%.4f y=%.4f  theta=%.4f grados, v=%.4f w=%.4f", data->x, data->y,
              data->theta, data->v, data->w);

    if (_corrected_data) {
        _current_estimated.theta = _before_estimated.theta + deltaTheta;
        _current_estimated.x = _before_estimated.x + deltaD * cos(_current_estimated.theta);
        _current_estimated.y = _before_estimated.y + deltaD * sin(_current_estimated.theta);
        _current_estimated.v = v;
        _current_estimated.w = w;

        _before_estimated.theta = _current_estimated.theta;
        _before_estimated.x = _current_estimated.x;
        _before_estimated.y = _current_estimated.y;
        _before_estimated.v = _current_estimated.v;
        _before_estimated.w = _current_estimated.w;
    }
    else {
        _current_estimated.theta = _current_cinematic.theta;
        _current_estimated.x = _current_cinematic.x;
        _current_estimated.y = _current_cinematic.y;
        _current_estimated.v = v;
        _current_estimated.w = w;

        _before_estimated.theta = _last_cinematic.theta;
        _before_estimated.x = _last_cinematic.x;
        _before_estimated.y = _last_cinematic.y;
        _before_estimated.v = _last_cinematic.v;
        _before_estimated.w = _last_cinematic.w;
    }

    return (0);
}

//////////////////////////////////////////////////

int BaseMotor::set_velocity(double v, double w)
{
    char buf1[64], buf2[64];
    long int vm1, vm2;
    int l1, l2;
    int semFD;
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};

    // conversion w a rad/seg
    w = w * 2.0 * M_PI / (360.0);

    // Modificados los signos para ajustar esquema con player
    double v1 = v - w * AXIS_LENGTH * 0.5;
    double v2 = v + w * AXIS_LENGTH * 0.5;

    // El 60 es para llevar segundos a minutos
    vm1 = (long int) ((v1 * 60 * REDUCTION) / (M_PI * WHEEL_DIAMETER));
    vm2 = (long int) ((v2 * 60 * REDUCTION) / (M_PI * WHEEL_DIAMETER));
    ROS_DEBUG_NAMED("base_motor_driver", "[BASE_MOTOR_DRIVER] Motor instructions: %li, %li", vm1, vm2);

    bzero((void *) buf1, 64);
    bzero((void *) buf2, 64);

    sprintf(buf1, "V%ld\n\r", vm1);
    sprintf(buf2, "V%ld\n\r", vm2);

    // find the end of the buf1 buffer
    for (l1 = 0; (buf1[l1] != '\0') && (l1 < 63); l1++) {
    }
    // find the end of the buf2 buffer
    for (l2 = 0; (buf2[l2] != '\0') && (l2 < 63); l2++) {
    }

    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Adquiriendo el semaforo en leeDatosMotores:'%s'", strerror(errno));
        return (1);
    }

    // INCIO ZONA EXCLUSIVA
    ROS_DEBUG_NAMED("base_motor_driver", "[BASE_MOTOR_DRIVER] SEM en colocaVelocidadMotores: %d",
                    semctl( semFD, 0,GETVAL,0));
    semop(semFD, sem_in, 1);

    // recuperamos la velocidad máxima
    write(_fd_motor1, VELOCITY_MAX, 9);
    write(_fd_motor2, VELOCITY_MAX, 9);
    ROS_DEBUG_NAMED("base_motor_driver", "[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", VELOCITY_MAX,
                    VELOCITY_MAX);

    write(_fd_motor1, buf1, l1);
    write(_fd_motor2, buf2, l2);
    ROS_DEBUG_NAMED("base_motor_driver", "[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", buf1, buf2);

    int returnValue = semop(semFD, sem_out, 1);
    if (returnValue != 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Error while semop:semFD:%i, '%s'", semFD, strerror(errno));
    }

    ROS_DEBUG_NAMED("base_motor_driver", "[BASE_MOTOR_DRIVER] v %f w %f vm1 %ld vm2 %ld, v * w: %f", v, w, vm1, vm2,
                    v * w);

    return (0);
}

//////////////////////////////////////////////////

int BaseMotor::set_displacement_velocity(double d, double v, double w)
{
    char buf[64], buf1[64], buf2[64];
    long int dm;
    int l, l1, l2;
    double v1, v2;
    long int vm1, vm2;
    int grados;
    int semFD;
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};
    cinematic_data dVelocidad;

    // Desplazamiento:
    grados = (int) ((d * PULSES_PER_REVOLUTION * REDUCTION) / (360));
    dm = (long int) (grados * 0.713);
    bzero((void *) buf, 64);
    sprintf(buf, "LR%ld\n\r", dm);
    for (l = 0; (buf[l] != '\0') && (l < 63); l++)
        ;

    // Velocidad:
    w = w * 2.0 * M_PI / (360.0);    // convertimos w a rad/seg

    // Cambia los signos de la velocidad para ajustar con player
    v1 = v - w * AXIS_LENGTH * 0.5;
    v2 = v + w * AXIS_LENGTH * 0.5;
    vm1 = (long int) ((v1 * 60 * REDUCTION) / (M_PI * WHEEL_DIAMETER));    // El 60 es para llevar segundos a minutos
    vm2 = (long int) ((v2 * 60 * REDUCTION) / (M_PI * WHEEL_DIAMETER));
    bzero((void *) buf1, 64);
    bzero((void *) buf2, 64);
    sprintf(buf1, "SP%ld\n\r", vm1);
    sprintf(buf2, "SP%ld\n\r", vm2);
    for (l1 = 0; (buf1[l1] != '\0') && (l1 < 63); l1++)
        ;
    for (l2 = 0; (buf2[l2] != '\0') && (l2 < 63); l2++)
        ;

    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Adquiriendo el semaforo en leeDatosMotores:'%s'", strerror(errno));
        return (1);
    }

    // INCIO ZONA EXCLUSIVA
    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en colocaVelocidadDesplazamiento: %d", semctl( semFD, 0,GETVAL,0));
    semop(semFD, sem_in, 1);

    //Ajustamos las velocidades máximas
    write(_fd_motor1, buf1, l1);
    write(_fd_motor2, buf2, l2);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", buf1, buf2);

    //Indicamos cuanto va a moverse
    write(_fd_motor1, buf, l);
    write(_fd_motor2, buf, l);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", buf, buf);

    //Comienza a moverse
    write(_fd_motor1, motor_order.c_str(), 3);
    write(_fd_motor2, motor_order.c_str(), 3);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", motor_order.c_str(), motor_order.c_str());

    semop(semFD, sem_out, 1);

    ROS_DEBUG("[BASE_MOTOR_DRIVER] colocaVelocidadDesplazamiento");
    ROS_DEBUG("[BASE_MOTOR_DRIVER] d=%f  dm=%ld l=%i ", d, dm, l);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] v %f w %f v1 %.4f v2 %.4f vm1 %ld vm2 %ld", v, w, v1, v2, vm1, vm2);

    return (0);
}

//////////////////////////////////////////////////

int BaseMotor::move_ahead(double distance)
{
    char buf[64];
    long int dm;
    int l;
    int semFD;
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};

    dm = (long int) ((distance * PULSES_PER_REVOLUTION * REDUCTION) / (M_PI * WHEEL_DIAMETER));

    bzero((void *) buf, 64);

    sprintf(buf, "LR%ld\n\r", dm);

    for (l = 0; (buf[l] != '\0') && (l < 63); l++)
        ;

    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Adquiriendo el semaforo en leeDatosMotores:'%s'", strerror(errno));
        return (1);
    }

    // INCIO ZONA EXCLUSIVA
    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en moverLineaRecta: %d", semctl( semFD, 0,GETVAL,0));
    semop(semFD, sem_in, 1);

    write(_fd_motor1, VELOCITY_MAX, 9);
    write(_fd_motor2, VELOCITY_MAX, 9);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", VELOCITY_MAX, VELOCITY_MAX);

    write(_fd_motor1, buf, l);
    write(_fd_motor2, buf, l);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", buf, buf);

    write(_fd_motor1, motor_order.c_str(), 3);
    write(_fd_motor2, motor_order.c_str(), 3);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", motor_order.c_str(), motor_order.c_str());

    semop(semFD, sem_out, 1);

    ROS_DEBUG("[BASE_MOTOR_DRIVER] distance: %f, dm: %ld, l: %i ", distance, dm, l);

    return (0);
}

//////////////////////////////////////////////////

int BaseMotor::relative_turn(double theta)
{
    char buf1[64], buf2[64];
    char buf3[] = "M\n\r";
    long int dm;
    int l1, l2;
    int semFD;
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};
    cinematic_data dVelocidad;

    ROS_DEBUG("[BASE_MOTOR_DRIVER] Motores: Me piden girar %f grados", theta);

    theta = theta * REDUCTION * PULSES_PER_REVOLUTION / 360.0;

    dm = (long int) theta * AXIS_LENGTH / WHEEL_DIAMETER;
    bzero((void *) buf1, 64);
    bzero((void *) buf2, 64);

    sprintf(buf1, "LR%ld\n\r", (-1) * dm);
    sprintf(buf2, "LR%ld\n\r", dm);

    for (l1 = 0; (buf1[l1] != '\0') && (l1 < 63); l1++)
        ;
    for (l2 = 0; (buf2[l2] != '\0') && (l2 < 63); l2++)
        ;

    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Adquiriendo el semaforo en leeDatosMotores:'%s'", strerror(errno));
        return (1);
    }

    // INCIO ZONA EXCLUSIVA
    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en giroRelativo: %d", semctl( semFD, 0,GETVAL,0));
    semop(semFD, sem_in, 1);

    write(_fd_motor1, VELOCITY_MAX, 9);
    write(_fd_motor2, VELOCITY_MAX, 9);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", VELOCITY_MAX, VELOCITY_MAX);

    write(_fd_motor1, buf1, l1);
    write(_fd_motor2, buf2, l2);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", buf1, buf2);

    write(_fd_motor2, buf3, 3);
    write(_fd_motor1, buf3, 3);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", buf3, buf3);

    semop(semFD, sem_out, 1);

    ROS_DEBUG("[BASE_MOTOR_DRIVER] theta %.4f   dm %ld l1 %i b1 '%s' b2 '%s' '%s'", theta, dm, l1, buf1, buf2, buf3);

    return (0);
}

//////////////////////////////////////////////////

int BaseMotor::reset_odometry()
{
    char DEFINE0[] = "HO\n\r";
    int semFD;
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};
    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Adquiriendo el semaforo en resetOdometry:'%s'", strerror(errno));
        return (1);
    }

    bzero((void *) &_current_cinematic, sizeof(cinematic_data));
    bzero((void *) &_last_cinematic, sizeof(cinematic_data));
    _distance1 = _distance2 = _before_distance1 = _before_distance2 = 0.0;

    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en resetOdometry: %d", semctl(semFD, 0,GETVAL,0));

    // INCIO ZONA EXCLUSIVA
    semop(semFD, sem_in, 1);
    write(_fd_motor1, DEFINE0, 4);
    write(_fd_motor2, DEFINE0, 4);
    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", DEFINE0, DEFINE0);
    // FIN ZONA EXCLUSIVA
    semop(semFD, sem_out, 1);

    return (0);
}

//////////////////////////////////////////////////

int BaseMotor::update_odometry(double x, double y, double theta)
{
    char DEFINE0[] = "HO\n\r";
    int semFD;
    struct sembuf sem_in[] = {0, -1, 0};
    struct sembuf sem_out[] = {0, 1, 0};

    if ((semFD = semget(SEMFDID, 1, 0)) < 0) {
        ROS_INFO("[BASE_MOTOR_DRIVER] Adquiriendo el semaforo en actualizarOdometria:'%s'", strerror(errno));
        return (1);
    }

    _distance1 = _distance2 = _before_distance1 = _before_distance2 = 0.0;

    _current_cinematic.x = x;
    _current_cinematic.y = y;
    _current_cinematic.theta = theta;
    // guess v and w are global variables

    _last_cinematic.x = _current_cinematic.x;
    _last_cinematic.y = _current_cinematic.y;
    _last_cinematic.theta = _current_cinematic.theta;

    ROS_DEBUG("[BASE_MOTOR_DRIVER] SEM en resetOdometry: %d", semctl( semFD, 0,GETVAL,0));

    // start exclusive zone
    semop(semFD, sem_in, 1);

    write(_fd_motor1, DEFINE0, 4);
    write(_fd_motor2, DEFINE0, 4);

    ROS_DEBUG("[BASE_MOTOR_DRIVER] Enviado M1: '%s' Enviado M2: '%s'", DEFINE0, DEFINE0);

    semop(semFD, sem_out, 1);
    // end exclusive zone

    return (0);
}

//////////////////////////////////////////////////

cinematic_data * BaseMotor::get_estimate_position()
{
    cinematic_data *data;

    data->x = _current_estimated.x;
    data->y = _current_estimated.y;
    data->theta = _current_estimated.theta;
    data->v = _current_estimated.v;
    data->w = _current_estimated.w;

    return data;
}

//////////////////////////////////////////////////

int BaseMotor::set_estimate_position(double x, double y, double theta)
{
    _current_estimated.x = x;
    _current_estimated.y = y;
    _current_estimated.theta = theta;
    // guess v and w are global variables

    _before_estimated.x = _current_estimated.x;
    _before_estimated.y = _current_estimated.y;
    _before_estimated.theta = _current_estimated.theta;
    _corrected_data = 1;

    return (0);
}

//////////////////////////////////////////////////
