/**
 ******************************************************************************
 * @file            AX_12A.hpp
 * @brief           Configuration file of AX-12A motors
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include "../structures.hpp"

namespace KMR::dxlP1
{

/**
 * @brief   Control table of AX-12A motors
 */
struct AX_12A_P1 : ControlTable {
	AX_12A_P1 ()
    {
        bulkRead = 0;
        jointValueMin = 1;
        jointValueMax = 1023;

		// EEPROM
        modelNumber.addr = 0;
        modelNumber.length = 2;
        modelNumber.unit = 1;
        firmware.addr = 2;
        firmware.length = 1;
        firmware.unit = 1;
        id.addr = 3;
        id.length = 1;
        id.unit = 1;
        baudrate.addr = 4;
        baudrate.length = 1;
        baudrate.unit = 1;
        returnDelayTime.addr = 5;
        returnDelayTime.length = 1;
        returnDelayTime.unit = 0.000002;
        CW_angleLimit.addr = 6;
        CW_angleLimit.length = 2;
        CW_angleLimit.unit = 0.005061;
        CCW_angleLimit.addr = 8;
        CCW_angleLimit.length = 2;
        CCW_angleLimit.unit = 0.005061;
        temperatureLimit.addr = 11;
        temperatureLimit.length = 1;
        temperatureLimit.unit = 1;
        minVoltageLimit.addr = 12;
        minVoltageLimit.length = 1;
        minVoltageLimit.unit = 0.1;
        maxVoltageLimit.addr = 13;
        maxVoltageLimit.length = 1;
        maxVoltageLimit.unit = 0.1;
        maxTorque.addr = 14;
        maxTorque.length = 2;
        maxTorque.unit = 0.1;
        statusReturn.addr = 16;
        statusReturn.length = 1;
        statusReturn.unit = 1;
        alarmLed.addr = 17;
        alarmLed.length = 1;
        alarmLed.unit = 1;
        shutdown.addr = 18;
        shutdown.length = 1;
        shutdown.unit = 1;

        // RAM
        torqueEnable.addr = 24;
        torqueEnable.length = 1;
        torqueEnable.unit = 1;
        LED.addr = 25;
        LED.length = 1;
        LED.unit = 1;
        goalPosition.addr = 30;
        goalPosition.length = 2;
        goalPosition.unit = 0.005061;
        movingSpeed.addr = 32;
        movingSpeed.length = 2;
        movingSpeed.unit = 0.0116;
        torqueLimit.addr = 34;
        torqueLimit.length = 2;
        torqueLimit.unit = 0.1;
        presentPosition.addr = 36;
        presentPosition.length = 2;
        presentPosition.unit = 0.005061;
        presentVelocity.addr = 38;
        presentVelocity.length = 2;
        presentVelocity.unit = 0.0116;
        presentLoad.addr = 40;
        presentLoad.length = 2;
        presentLoad.unit = 0.1;
        presentVoltage.addr = 42;
        presentVoltage.length = 1;
        presentVoltage.unit = 0.1;
        presentTemperature.addr = 43;
        presentTemperature.length = 1;
        presentTemperature.unit = 1;
        registered.addr = 44;
        registered.length = 1;
        registered.unit = 1;
        moving.addr = 46;
        moving.length = 1;
        moving.unit = 1;
        lock.addr = 47;
        lock.length = 1;
        lock.unit = 1;
        punch.addr = 48;
        punch.length = 2;
        punch.unit = 1;
    }
};


}
