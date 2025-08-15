/**
 *****************************************************************************
 * @file            KMR_dxlP1_structures.hpp
 * @brief           File defining the structures used in this library
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 08/2025
 *****************************************************************************
 */

#pragma once

namespace KMR::dxlP1
{
const int UNDEF = -1;


/**
 * @brief       Hardware abstraction layer for Dynamixel motors
 */
enum class ControlMode {
    CURRENT, SPEED, POSITION, MULTITURN, HYBRID, PWM, UNDEF_CTRL
};


/**
 * @brief   Structure saving the info of a single motor: both config-wise (ID, model...)
 *          and specific to the project (reset status...)
 */
struct Motor {
    int id;                 // ID of the motor
    int model;              // Model of the motor

    // Multiturn variables
    bool multiturn = 0;     // Flag for using multiturn mode
    int toReset = 0;        // Flag for multiturn requiring a reset

    Motor(int id, int model)
    {
        this->id = id;
        this->model = model;
    }; 
};

/**
 * @brief   Enumerate of all data fields in a Dynamixel motor in protocol 1
 */
enum class ControlTableItem
{
    MODEL_NBR, FIRMWARE, ID, BAUDRATE, RETURN_DELAY, CW_ANGLE_LIMIT, CCW_ANGLE_LIMIT, TEMPERATURE_LIMIT,
    MIN_VOLTAGE_LIMIT, MAX_VOLTAGE_LIMIT, MAX_TORQUE, STATUS_RETURN, ALARM_LED, SHUTDOWN, MULTITURN_OFFSET, 
    RESOLUTION_DIVIDER,

    TORQUE_ENABLE, LED, D_GAIN, I_GAIN, P_GAIN, GOAL_POSITION, MOVING_SPEED, TORQUE_LIMIT, PRESENT_POSITION,
    PRESENT_VELOCITY, PRESENT_LOAD, PRESENT_VOLTAGE, PRESENT_TEMPERATURE, REGISTERED, MOVING, LOCK, PUNCH,
    REALTIME_TICK, CONSUMED_CURRENT, ENABLE_TORQUE_MODE, GOAL_TORQUE, GOAL_ACCELERATION,

    NBR_FIELDS, UNDEF
};

/**
 * @brief   Structure of each field in a Dynamixel control table
 */
struct Field {
    float unit;
    int length;
    int addr;

    Field() {
        length = UNDEF;
    }
};

/**
 * @brief   General structure of a Dynamixel control table
 */
struct ControlTable {
    Field modelNumber;
    Field firmware;
    Field id;
    Field baudrate;
    Field returnDelayTime;
    Field CW_angleLimit;
    Field CCW_angleLimit;
    Field temperatureLimit;
    Field minVoltageLimit;
    Field maxVoltageLimit;
    Field maxTorque;
    Field statusReturn;
    Field alarmLed;
    Field shutdown;
    Field multiturnOffset;
    Field resolutionDivider;

    Field torqueEnable;
    Field LED;
    Field D_gain;
    Field I_gain;
    Field P_gain;
    Field goalPosition;
    Field movingSpeed;
    Field torqueLimit;
    Field presentPosition;
    Field presentVelocity;
    Field presentLoad;
    Field presentVoltage;
    Field presentTemperature;
    Field registered;
    Field moving;
    Field lock;
    Field punch;
    Field realtimeTick;
    Field consumedCurrent;
    Field enableTorqueMode;
    Field goalTorque;
    Field goalAcceleration;
};

}