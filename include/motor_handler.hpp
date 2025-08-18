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
//#include "reader.hpp"

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
    std::vector<int> m_ids;     // List of all motor IDs
    int m_nbrMotors;            // Number of motors
    std::vector<int> m_models;  // Models of the motors

    MotorHandler(std::vector<int> ids, const char *port_name, int baudrate);
    ~MotorHandler();

    // Easy handlers creations
    
    Writer* getNewWriter(ControlTableItem field, std::vector<int> ids);
    //Reader* getNewReader(std::vector<ControlTableItem> fields, std::vector<int> ids);
    void deleteWriter(Writer* writer);
    //void deleteReader(Reader* reader);
    
    // Enable/disable motor torque and rebooting

    void enableMotors();
    void disableMotors();
    //void reboot(int id);
    //void reboot();

    // Motor setup

    //void setControlModes(std::vector<ControlMode> controlModes);  
    //void setControlModes(ControlMode controlMode);  
    //void setReturnDelayTime(float val); 
    //void setMinVoltage(std::vector<float> maxVoltages);   
    //void setMinVoltage(float minVoltage);   
    //void setMaxVoltage(std::vector<float> maxVoltages);  
    //void setMaxVoltage(float maxVoltage);  


    // Set limits for different operating modes

    //void setMinPosition(std::vector<float> minPositions);
    //void setMinPosition(float minPosition);
    //void setMaxPosition(std::vector<float> maxPositions);
    //void setMaxPosition(float maxPosition);
    //void setMaxSpeed(std::vector<float> maxSpeeds);
    //void setMaxSpeed(float maxSpeed);
    //void setMaxCurrent(std::vector<float> maxCurrents);
    //void setMaxCurrent(float maxCurrent);
    //void setMaxPWM(std::vector<float> maxPWMs);
    //void setMaxPWM(float maxPWM);

    // Control and feedback commands

    void setPositions(std::vector<float> positions);
    //bool getPositions(std::vector<float>& positions);
    //void setSpeeds(std::vector<float> speeds);
    //bool getSpeeds(std::vector<float>& speeds);
    //void setCurrents(std::vector<float> currents);
    //bool getCurrents(std::vector<float>& currents);
    //void setPWMs(std::vector<float> pwms);
    //bool getPWMs(std::vector<float>& pwms);

    //void setHybrid(std::vector<float> positions, std::vector<float> currents);
    //bool getHybrid(std::vector<float>& positions, std::vector<float>& currents);

    // Multiturn

    void resetMultiturnMotors();

private:
    dynamixel::PortHandler   *portHandler_ = nullptr;
    dynamixel::PacketHandler *packetHandler_ = nullptr;
    Hal* m_hal = nullptr;

    Writer* m_motorEnableWriter = nullptr;

    // Base controls
    
    Writer* m_positionWriter = nullptr;
    Writer* m_speedWriter = nullptr;
    Writer* m_currentWriter = nullptr;
    Writer* m_PWMWriter = nullptr;
    //Reader* m_positionReader = nullptr;
    //Reader* m_speedReader = nullptr;
    //Reader* m_currentReader = nullptr;
    //Reader* m_PWMReader = nullptr;

    void init_comm(const char *port_name, int baudrate, float protocol_version);
    void check_comm();
};

}
