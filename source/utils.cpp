/**
 ******************************************************************************
 * @file            utils.cpp
 * @brief           Useful miscellaneous functions
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <cmath>
#include <sstream> // Convert dec-hex
#include <cstring>

#include "utils.hpp"

using namespace std;

namespace KMR::dxlP1
{

/**
 * @brief       Convert degrees to radians
 * @param[in]   deg Angle in degrees
 * @return      Angle in radians
 */
float deg2rad(float deg)
{
    return (deg * M_PI / 180.0);
}

/**
 * @brief       Convert radians to degrees
 * @param[in]   rad Angle in radians
 * @return      Angle in degrees
 */
float rad2deg(float rad)
{
    return (rad * 180 / M_PI);
}


/**
 * @brief   Get current time structure
 * @return  Current time structure
 */
timespec time_s()
{
    struct timespec real_time;

    if (clock_gettime(CLOCK_REALTIME, &real_time) == -1 )
    {
        perror("clock gettime");
        exit( EXIT_FAILURE );
    }

    return real_time;
}

/**
 * @brief       Get elapsed time, in microseconds
 * @param[in]   t2 End time structure (gotten with time_s)
 * @param[in]   t2 Start time structure (gotten with time_s)
 * @return      Elapsed time between t1 and t2, in us
 */
double get_delta_us(struct timespec t2, struct timespec t1)
{
    struct timespec td;
    td.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td.tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td.tv_sec > 0 && td.tv_nsec < 0)
    {
        td.tv_nsec += 1000000000;
        td.tv_sec--;
    }

    return(td.tv_sec*1000000 + td.tv_nsec/1000);
}

/**
 * @brief   Convert an int to hex to be able to print it
 * @param   dec Integer in decimal
 * @return  String in hex
 */
std::string convertToHex(int dec) 
{
    std::stringstream ss;
    ss << std::hex << dec; // int decimal_value
    std::string res ( ss.str() );

    return res;
}

}