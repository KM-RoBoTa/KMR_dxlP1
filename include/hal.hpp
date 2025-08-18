/**
 *****************************************************************************
 * @file            hal.hpp
 * @brief           Define the Hal class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include <string>
#include <iostream>
#include <cstdint>
#include <vector>

#include "../config/motor_models.hpp"

namespace KMR::dxlP1
{

/**
 * @brief       Hardware abstraction layer for Dynamixel motors
 * @details     The lowest-level element in the library. The Hal class serves as
 *              an abstraction layer, providing high-level functions to get the Dynamixel control
 *              table addresses and byte sizes
 */
class Hal {
public:
    Hal(std::vector<int> ids, int nbrMotors, std::vector<int> models);
    ~Hal();

    // Get hardware information

    float getPositionOffset(int modelNumber);
    Field getControlFieldFromModel(int modelNumber, ControlTableItem item);
    Motor getMotorFromID(int id);
    
    // Multiturn functionalities
    
    void setMultiturnMode(int id);
    void updateResetStatus(int id, int status);


private:
    ControlTable* AX_12A = nullptr;
    ControlTable* MX_64 = nullptr;

    int m_nbrMotors = -1;
    std::vector<int> m_ids;             // IDs of all motors
    std::vector<int> m_models;          // Models of all motors
	std::vector<Motor> m_motorsList;    // Parameters per specific motor

    // Get hardware information, in private scope
    Field getControlField(ControlTable motor, ControlTableItem item);
    ControlTable getControlTable(int modelNumber);
};

}
