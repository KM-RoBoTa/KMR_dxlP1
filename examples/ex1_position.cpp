/**
 ********************************************************************************************
 * @file    ex1_position.cpp
 * @brief   Example for position control
 * @details This example is designed for 2 motors to showcase the effect of setting
 *          the min and max positions. \n
 *          Those motors can be any model, in protocol 2 (IDs 1 and 2 by default). \n \n
 *          The motors' goal positions oscillate during -pi and +pi throughout the 
 *          program. However, the second's motor limits are set at lower values, which
 *          results in the second motor not being able to execute the full motion due to 
 *          these constraints.
 ********************************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 ********************************************************************************************
 */

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

#include "../include/KMR_dxlP1.hpp"
#include "../include/utils.hpp"

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
    cout << endl << endl << " ---------- POSITION CONTROL ---------" << endl;
    robot.disableMotors();

    // Set the control mode
    //KMR::dxl::ControlMode mode = KMR::dxl::ControlMode::POSITION;
    //robot.setControlModes(mode);
    //sleep(1);

    // Set min/max positions
    //vector<float> minPositions = {-M_PI, -2*M_PI/3};
    //vector<float> maxPositions = {+M_PI, +M_PI/2};
    //robot.setMinPosition(minPositions);
    //usleep(50*1000);
    //robot.setMaxPosition(maxPositions);
    //usleep(50*1000);
    robot.enableMotors();    
    
    // Required variables
    vector<float> goalPositions(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors, 0);

    cout << endl << endl << " ---------- Showcase of ignored commands when above set limits ---------" << endl;
    sleep(3);
    robot.setPositions(goalPositions);
    sleep(1);
    goalPositions[0] = -M_PI;
    robot.setPositions(goalPositions);
    cout << "Going to -PI" << endl;
    sleep(2);

    goalPositions[0] = -2*M_PI/3;
    robot.setPositions(goalPositions);
    cout << "Going to -2*PI/3" << endl;
    sleep(2);

    goalPositions[0] = 0;
    robot.setPositions(goalPositions);
    cout << "Going to 0" << endl;
    sleep(2);

    goalPositions[0] = M_PI;
    robot.setPositions(goalPositions);
    cout << "Going to PI" << endl;
    sleep(2);

    goalPositions[0] = M_PI/2;
    robot.setPositions(goalPositions);
    cout << "Going to PI/2" << endl;
    sleep(2);

    goalPositions[0] = 0;
    robot.setPositions(goalPositions);
    cout << "Going to 0" << endl;
    sleep(2);


    cout << endl << endl << " ---------- Smooth position command, also with set limits ---------" << endl;
    sleep(3);
    float angle = 0;
    bool forward = 1;
    int ctr = 0;

    // Main loop
    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = KMR::dxlP1::time_s();
        //robot.getPositions(fbckPositions);

        //cout << "Positions: "; 
        //for (int i=0; i<nbrMotors; i++) {
        //    cout << fbckPositions[i] << " rad";
        //    if (i != (nbrMotors-1))
        //        cout << ", ";
        //}
        //cout << endl;

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = angle;
        robot.setPositions(goalPositions);

        // Update the goal angle for next loop
        if (forward) {
            angle += INCREMENT;

            if (angle > M_PI) {
                angle = M_PI;
                forward = false;
            }
        }
        else {
            angle -= INCREMENT;

            if (angle < -M_PI) {
                angle = -M_PI;
                forward = true;
            }
        }

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = KMR::dxlP1::time_s();
        double elapsed = KMR::dxlP1::get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    sleep(1);
    robot.disableMotors();

    // Reset the limits
    //for (int i=0; i<nbrMotors; i++) {
    //    minPositions[i] = -M_PI;
    //    maxPositions[i] = +M_PI;
    //}
    //robot.setMinPosition(minPositions);
    //usleep(50*1000);
    //robot.setMaxPosition(maxPositions);
    //usleep(50*1000);

    cout << "Example finished, exiting" << endl;
}