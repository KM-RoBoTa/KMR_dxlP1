/**
 *****************************************************************************
 * @file            KMR_dxl_motor_handler.hpp
 * @brief           Declare the MotorHandler class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include "writer.hpp"
#include "reader.hpp"

namespace KMR::dxlP1
{

/**
 * @brief   Highest-level class that manages all communication with the motors
 * @details This class contains everything necessary for handling dynamixel motors. \n 
 *          On creation, it opens the port, initializes communication, pings and detects motors. \n 
 *          It provides out-of-the-box many most-used functions, such as enabling/disabling motors,
 *          setting control modes, setting different limits, sending control commands and getting
 *          most common feedbacks. \n  
 *          In case the user wants to create custom readers/writers (for example indirect ones),
 *          this class provides functions to easily create and destroy them.
 */
class MotorHandler {
public:

    MotorHandler(std::vector<int> ids, const char *port_name, int baudrate);
    ~MotorHandler();

    // Easy handlers creations
    
    Writer* getNewWriter(ControlTableItem field, std::vector<int> ids);
    Reader* getNewReader(ControlTableItem field, std::vector<int> ids);
    void deleteWriter(Writer* writer);
    void deleteReader(Reader* reader);
    
    // Enable/disable motor torque and rebooting

    bool enableMotors();
    bool enableMotors(std::vector<int> ids);
    bool enableMotor(int id);
    bool disableMotors();
    bool disableMotors(std::vector<int> ids);
    bool disableMotor(int id);
    //void reboot(int id);
    //void reboot();

    // Motor setup

    //void setControlModes(std::vector<ControlMode> controlModes);  
    //void setControlModes(ControlMode controlMode);  
    bool setReturnDelayTime(float val); 
    bool setMinVoltage(std::vector<float> maxVoltages);   
    bool setMinVoltage(float minVoltage);   
    bool setMaxVoltage(std::vector<float> maxVoltages);  
    bool setMaxVoltage(float maxVoltage);  


    // Set limits for different operating modes

    bool setMinPosition(std::vector<float> minPositions);
    bool setMinPosition(float minPosition);
    bool setMaxPosition(std::vector<float> maxPositions);
    bool setMaxPosition(float maxPosition);

    // Control commands

    bool setPositions(std::vector<float> positions);
    bool setPositions(std::vector<int> ids, std::vector<float> positions);
    bool setPosition(int id, float position);


    bool setTorques(std::vector<float> torques);
    bool setTorques(std::vector<int> ids, std::vector<float> torques);
    bool setTorque(int id, float torque);


    // Feedback functions

    bool getPositions(std::vector<float>& positions);
    bool getPositions(std::vector<int> ids, std::vector<float>& positions);
    bool getPosition(int id, float position);

    bool getSpeeds(std::vector<float>& speeds);
    bool getSpeeds(std::vector<int> ids, std::vector<float>& speeds);
    bool getSpeed(int id, float speed);

    // Multiturn

    void resetMultiturnMotors();

private:
    std::vector<int> m_ids;     // List of all motor IDs
    int m_nbrMotors;            // Number of motors
    std::vector<int> m_models;  // Models of the motors

    dynamixel::PortHandler   *portHandler_ = nullptr;
    dynamixel::PacketHandler *packetHandler_ = nullptr;
    Hal* m_hal = nullptr;

    Writer* m_motorEnableWriter = nullptr;

    // Base controls
    
    Writer* m_positionWriter = nullptr;
    Writer* m_torqueWriter = nullptr;
    Reader* m_positionReader = nullptr;
    Reader* m_speedReader = nullptr;

    void init_comm(const char *port_name, int baudrate, float protocol_version);
    void check_comm();
};

}
