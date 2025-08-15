/**
 ******************************************************************************
 * @file            motor_models.hpp
 * @brief           Header file including all motor models config files
 ******************************************************************************
 * @copyright
 * Copyright 2021-2054 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include "structures.hpp"

#include "motor_models/AX_12A.hpp"

namespace KMR::dxlP1
{

// (Note: const int is preferred over #define, which is a preprocessor directive and 
// thus does not live inside namespaces) 

const int MODEL_NBR_AX_12A      = 12;

}
