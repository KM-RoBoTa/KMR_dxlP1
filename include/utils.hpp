/**
 ******************************************************************************
 * @file            utils.hpp
 * @brief           Header for the utils.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <unistd.h> // Provides the usleep function

namespace KMR::dxlP1
{

// Trigonometry
float deg2rad(float deg);
float rad2deg(float rad);

// Time-measuring functions
timespec time_s();
double get_delta_us(struct timespec t2, struct timespec t1);

// Conversions
std::string convertToHex(int dec) ;

/**
 * @brief       Saturate an input value between a min and max
 * @tparam      T Number type (int, float, etc)
 * @param min   Lower bound for saturation
 * @param max   Upper bound for saturation
 * @param val   Value to be saturated
 * @return      Saturated value between min and max
 */
template<typename T>
T saturate(T min, T max, T val)
{
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}

/**
 * @brief   Get the index of an element in a vector
 * @tparam  T Number
 * @param   v Vector in which to search
 * @param   k Element searched for in the vector
 * @return  Index of the element in the vector. Equal to -1 if not in the vector
 */
template<typename T>
int getIndex(std::vector<T> v, T k)
{
    auto it = std::find(v.begin(), v.end(), k); 
    int index = -1;
  
    // If element was found 
    if (it != v.end())  
        index = it - v.begin(); 
    return index;    
}

}