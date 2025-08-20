/**
 *****************************************************************************
 * @file    ex5_multiturn.cpp
 * @brief   Example for multiturn control
 * @details The motors are controlled in multiturn position control.
 *          At first, they will turn positively (counterclockwise), then 
 *          negatively (clockwise), while resetting when necessary.
 *          Please note the goal position adjustements in the code
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

#include "../include/KMR_dxlP1.hpp"

#define BAUDRATE        1000000
#define PORTNAME        "/dev/ttyUSB0"
#define INCREMENT       0.02
#define MAX_CTR         2000
#define CTRL_PERIOD_US  5000

using namespace std;


// --------------------------------------------------------------------------- //
// Id(s) of motor(s)
vector<int> ids = {1};  // EDIT HERE FOR YOUR MOTOR(s)
// --------------------------------------------------------------------------- //

int nbrMotors = ids.size();
KMR::dxlP1::MotorHandler robot(ids, PORTNAME, BAUDRATE);

int main()
{
    cout << endl << endl << " ---------- MULTITURN CONTROL ---------" << endl;
    robot.disableMotors();

    // Set the motors to multiturn mode
    KMR::dxlP1::ControlMode mode = KMR::dxlP1::ControlMode::MULTITURN;
    robot.setControlModes(mode);
    sleep(1);

    // Set the return delay time to 0
    robot.setReturnDelayTime(0);
    sleep(1);

    // Prepare required variables for the control
    vector<float> goalPositions(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors, 0);

    robot.enableMotors();    
    robot.getPositions(fbckPositions);
    float goalAngle = fbckPositions[0];
    int ctr = 0;
    bool toRecenter = 0;

    // -------- Main loop -------- //
    while (ctr < MAX_CTR) {
        timespec start = KMR::dxlP1::time_s();
        robot.resetMultiturnMotors();  // Resets the motors marked by internal flag

        // --- Update the goal angle --- // 

        // If goal angle was outside of [-2PI; 2PI] the previous step, recenter it
        if (toRecenter) {
            if (goalAngle > 2*M_PI)
                goalAngle -= 2*M_PI;
            else if (goalAngle < -2*M_PI)
                goalAngle += 2*M_PI;
            toRecenter = false;
        }

        robot.getPositions(fbckPositions);
        cout << "Positions: "; 
        for (int i=0; i<nbrMotors; i++) {
            cout << "Goal position: " << goalPositions[i] << ", fbck: " << fbckPositions[i] << " rad";
            if (i != (nbrMotors-1))
                cout << ", ";
        }
        cout << endl;        

        // Update the goal angle
        if (ctr < MAX_CTR/2)
            goalAngle += INCREMENT;
        else 
            goalAngle -= INCREMENT;

        // If the new goal angle is outside of [-2PI; 2PI], it needs to be recentered next step
        if (goalAngle > 2*M_PI || goalAngle < -2*M_PI)
            toRecenter = true;

        // --- Send new goal positions --- //
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = goalAngle;
        robot.setPositions(goalPositions);

        // --- Increment counter and set the control loop to 5ms --- // 
        ctr++;

        timespec end = KMR::dxlP1::time_s();
        double elapsed = KMR::dxlP1::get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    robot.disableMotors();
    cout << "Example finished, exiting" << endl;
}