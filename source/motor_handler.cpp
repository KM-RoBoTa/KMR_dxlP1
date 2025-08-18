/**
 *****************************************************************************
 * @file            KMR_dxl_motor_handler.cpp
 * @brief           Define the MotorHandler class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <cstdint>
#include <iostream>
#include <unistd.h>  // Provides sleep function for linux

#include "motor_handler.hpp"

using namespace std;

namespace KMR::dxlP1
{

const int PROTOCOL_VERSION = 1;
const int ENABLE  = 1;
const int DISABLE = 0;

/**
 * @brief       Constructor for MotorHandler
 * @param[in]   ids List of IDs of all the motors
 * @param[in]   port_name Serial port handling the motors, of the type "/dev/ttyUSBx"
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 */
MotorHandler::MotorHandler(vector<int> ids, const char *port_name, int baudrate)
{
    m_ids = ids;
    m_nbrMotors = ids.size();
    m_models = vector<int>(m_nbrMotors);

    // Connect U2D2
    init_comm(port_name, baudrate, PROTOCOL_VERSION);

    // Ping each motor to validate the communication is working
    check_comm();

    // Initialize Hal
    m_hal = new Hal(m_ids, m_nbrMotors, m_models);

    // 2 integrated handlers: motor enabling and mode setter
    {
        //using enum ControlTableItem;

        m_motorEnableWriter = getNewWriter(ControlTableItem::TORQUE_ENABLE, m_ids);

        // Integrated base command handlers
        m_positionWriter = getNewWriter(ControlTableItem::GOAL_POSITION, m_ids);
        //m_speedWriter = getNewWriter(vector<ControlTableItem>{GOAL_VELOCITY}, m_ids);
        //m_currentWriter = getNewWriter(vector<ControlTableItem>{GOAL_CURRENT}, m_ids);
        //m_PWMWriter = getNewWriter(vector<ControlTableItem>{GOAL_PWM}, m_ids);
        //m_positionReader = getNewReader(vector<ControlTableItem>{PRESENT_POSITION}, m_ids);
        //m_speedReader = getNewReader(vector<ControlTableItem>{PRESENT_VELOCITY}, m_ids);
        //m_currentReader = getNewReader(vector<ControlTableItem>{PRESENT_CURRENT}, m_ids);
        //m_PWMReader = getNewReader(vector<ControlTableItem>{PRESENT_PWM}, m_ids);
    }
}


/**
 * @brief Destructor. Also closes the serial port
 */
MotorHandler::~MotorHandler()
{
    // Delete handlers created on heap
    deleteWriter(m_motorEnableWriter);
    deleteWriter(m_positionWriter); 
    deleteWriter(m_speedWriter);
    deleteWriter(m_currentWriter);
    deleteWriter(m_PWMWriter);
    //deleteReader(m_speedReader);
    //deleteReader(m_positionReader);
    //deleteReader(m_currentReader);
    //deleteReader(m_PWMReader);

    // Security against double freeing
    m_motorEnableWriter = nullptr;
    m_positionWriter = nullptr;
    m_speedWriter = nullptr;
    m_currentWriter = nullptr;
    m_PWMWriter = nullptr;
    //m_positionReader = nullptr;
    //m_speedReader = nullptr;
    //m_currentReader = nullptr;
    //m_PWMReader = nullptr;

    // Delete the Hal
    delete m_hal;
    m_hal = nullptr;

    // Close port
    portHandler_->closePort();
}


/**
 * @brief       Initialize the serial communication
 * @param[in]   port_name Name of the port handling communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   protocol_version Protocol version, for the communication (U2D2)
 */
void MotorHandler::init_comm(const char *port_name, int baudrate, float protocol_version)
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name);
    if (!portHandler_->openPort())
        exit(1);
    else
        cout<< "Succeeded to open the motors port!" <<endl;

    if (!portHandler_->setBaudRate(baudrate)) {
        cout<< "Failed to set baudrate!" <<endl;
        exit(1) ;
    }
    else
        cout << "Succeeded to change the baudrate!" <<endl;

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

/**
 * @brief   Ping each motor to validate the communication is working
 * @note    Also populates the models vector, required to initialize Hal
 */
void MotorHandler::check_comm()
{
    cout << "Pinging motors...." << endl;

    for (int i=0; i<m_nbrMotors; i++) {
        int id = m_ids[i];
        uint16_t model_number = 0;
        uint8_t dxl_error = 0;

        bool result = packetHandler_->ping(portHandler_, id, &model_number, &dxl_error);
        if (result != COMM_SUCCESS) {
            cout << "Failed to ping, check config file and motor ID: " << id << endl;
            cout << "Check also the power source and the cabling ;) " << endl;
            cout << packetHandler_->getTxRxResult(result) << endl;
            exit(1);
        }
        else {
            cout << "id: " << id << ", model number : " << model_number << endl;
            m_models[i] = (int)model_number;
        }
    }
}

/*
******************************************************************************
 *                     Easy handlers creation and deletion
 ****************************************************************************/

/**
 * @brief   Create a new Writer object
 * @note    The new Writer is created on the heap.
 *          Make sure to use the provided deleteWriter() method to clean the memory
 * @param   fields Control table fields to be handled by the new writer 
 * @param   ids IDs of motors to be handled by the new writer
 * @return  New Writer object
 */
Writer* MotorHandler::getNewWriter(ControlTableItem field, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Writer creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Writer* writer = new Writer(field, ids, models, portHandler_, packetHandler_, m_hal);
    return writer;
}

/**
 * @brief   Create a new Reader object
 * @note    The new Reader is created on the heap.
 *          Make sure to use the provided deleteReader() method to clean the memory
 * @param   fields Control table fields to be handled by the new reader 
 * @param   ids IDs of motors to be handled by the new reader
 * @return  New Reader object
 */
/*Reader* MotorHandler::getNewReader(vector<ControlTableItem> fields, vector<int> ids)
{
    // Get the list of models corresponding to the ids
    vector<int> models(ids.size());
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        if (idx < 0) {
            cout << "Error! Unknown ID during Reader creation. Exiting" << endl;
            exit(1);
        }
        models[i] = m_models[i];
    }

    Reader* reader = new Reader(fields, ids, models, portHandler_, packetHandler_, m_hal, 0);
    return reader;
}*/

/**
 * @brief   Delete a Writer object, previously created with getNewWriter()
 * @param   writer Writer object to be deleted
 */
void MotorHandler::deleteWriter(Writer* writer)
{
    delete writer;
    writer = nullptr; // Security in case of double freeing
}

/**
 * @brief   Delete a Reader object, previously created with getNewReader()
 * @param   reader Reader object to be deleted
 */
/*void MotorHandler::deleteReader(Reader* reader)
{
    delete reader;
    reader = nullptr; // Security in case of double freeing
}*/


/*
******************************************************************************
*                         Enable/disable motors
****************************************************************************/

/**
 * @brief   Enable all the motors
 */
void MotorHandler::enableMotors()
{
    m_motorEnableWriter->send(vector<int>{ENABLE});
}

/**
 * @brief   Disable all the motors
 */
void MotorHandler::disableMotors()
{
    m_motorEnableWriter->send(vector<int>{DISABLE});
}

/**
 * @brief       Reboot a specific motor
 * @note        Make sure to give the motor enough time to reboot (~100ms).
 *              After the reboot, the motor's torque is disabled
 * @param[in]   id Motor to be rebooted
 */
/*void MotorHandler::reboot(int id)
{
    packetHandler_->reboot(portHandler_, id);
}/*

/**
 * @brief   Reboot all motors
 * @note    Make sure to give the motors enough time to reboot (~100ms).
 *          After the reboot, the motors' torque is disabled
 */
/*void MotorHandler::reboot()
{
    for (int i=0; i<m_nbrMotors; i++)
        packetHandler_->reboot(portHandler_, m_ids[i]);
}*/


/****************************************************************************
*                     EEPROM settings writing
****************************************************************************/

/**
 * @brief       Set the control modes of motors
 * @note        If all motors have the same control mode, you can use the overload function
 * @param[in]   controlModes Control modes to be set to motors
 */
/*
void MotorHandler::setControlModes(std::vector<ControlMode> controlModes)
{
    using enum ControlMode;

    Writer writer(vector<ControlTableItem>{ControlTableItem::OPERATING_MODE}, m_ids, m_models,
                                            portHandler_, packetHandler_, m_hal, 0);

    if (controlModes.size() != m_nbrMotors) {
        cout << "Error! Not all motors have their control modes assigned. Exiting" << endl; 
        cout << endl;
    }

    vector<int> controlModes_int(m_nbrMotors);
    for (int i=0; i<m_nbrMotors; i++) {
        switch (controlModes[i])
        {
        case CURRENT:
            controlModes_int[i] = CTRL_CURRENT;
            break;
        case SPEED:
            controlModes_int[i] = CTRL_SPEED;
            break;
        case POSITION:
            controlModes_int[i] = CTRL_POSITION;
            break;
        case MULTITURN:
            controlModes_int[i] = CTRL_MULTITURN;
            m_hal->setMultiturnMode(m_ids[i]);
            break;
        case HYBRID:
            controlModes_int[i] = CTRL_HYBRID;
            break;
        case PWM:
            controlModes_int[i] = CTRL_PWM;
            break;
        
        default:
            cout << "Error! Trying to assign an unknown control mode. Exiting" << endl;
            exit(1);
            break;
        }
    }

    writer.addDataToWrite(controlModes_int);
    writer.syncWrite();
}
*/

/**
 * @brief       Set the same control mode to all motors
 * @param[in]   controlMode Control mode to be set to all motors
 */
/*
void MotorHandler::setControlModes(ControlMode controlMode)
{
    vector<ControlMode> controlModes(m_nbrMotors, controlMode);
    setControlModes(controlModes);
}
*/


/**
 * @brief       Set the return delay to all motors
 * @param[in]   val Return delay time [s]
 */
/*
void MotorHandler::setReturnDelayTime(float val)
{
    Writer writer(vector<ControlTableItem>{ControlTableItem::RETURN_DELAY}, m_ids, m_models,
                                            portHandler_, packetHandler_, m_hal, 0);

    vector<float> vals(m_nbrMotors, val);
    writer.addDataToWrite(vals);
    writer.syncWrite();
}
*/

/**
 * @brief       Set the minimum voltages to all motors
 * @param[in]   minVoltages Min. allowed voltages in motors [V]
 */      
/*                           
void MotorHandler::setMinVoltage(std::vector<float> minVoltages)
{
    Writer writer(vector<ControlTableItem>{ControlTableItem::MIN_VOLTAGE_LIMIT}, m_ids, m_models,
                                            portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(minVoltages);
    writer.syncWrite();
}
*/

/**
 * @brief       Set the same minimum voltage to all motors
 * @param[in]   minVoltage Min. allowed voltage in motors [V]
 */  
/*                               
void MotorHandler::setMinVoltage(float minVoltage)
{
    vector<float> minVoltages(m_nbrMotors, minVoltage);
    setMinVoltage(minVoltages);
}
*/

/**
 * @brief       Set the maximum voltage to all motors
 * @param[in]   maxVoltages Max. allowed voltages in motors [V]
 */   
/*                              
void MotorHandler::setMaxVoltage(std::vector<float> maxVoltages)
{
    Writer writer(vector<ControlTableItem>{ControlTableItem::MAX_VOLTAGE_LIMIT}, m_ids, m_models,
                                            portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxVoltages);
    writer.syncWrite();
}
*/

/**
 * @brief       Set the same maximum voltage to all motors
 * @param[in]   maxVoltage Max. allowed voltage in motors [V]
 */
/*                              
void MotorHandler::setMaxVoltage(float maxVoltage)
{
    vector<float> maxVoltages(m_nbrMotors, maxVoltage);
    setMaxVoltage(maxVoltages);
}
*/

/****************************************************************************
*                  Setting limits in different operating modes
****************************************************************************/

/**
 * @brief       Set the minimum position to all motors
 * @note        If all motors have the same min. position, you can use the overload
 * @param[in]   minPositions Min. positions for motors [rad]
 */  
/*                               
void MotorHandler::setMinPosition(std::vector<float> minPositions)
{
    if (minPositions.size() != m_nbrMotors) {
        cout << "Error! The min. position values do not coincide with the number of motors" << endl;
        exit(1);
    }
    
    Writer writer(vector<ControlTableItem>{ControlTableItem::MIN_POSITION_LIMIT}, m_ids, m_models,
                                            portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(minPositions);
    writer.syncWrite();
}
*/

/**
 * @brief       Set the same minimum position to all motors
 * @param[in]   minPosition Min. position for all motors [rad]
 */   
/*
void MotorHandler::setMinPosition(float minPosition)
{
    vector<float> minPositions(m_nbrMotors, minPosition);
    setMinPosition(minPositions);
}
*/

/**
 * @brief       Set the maximum position to all motors
 * @note        If all motors have the same max. position, you can use the overload
 * @param[in]   maxPositions Max. positions for motors [rad]
 */   
/*                              
void MotorHandler::setMaxPosition(std::vector<float> maxPositions)
{
    if (maxPositions.size() != m_nbrMotors) {
        cout << "Error! The max. position values do not coincide with the number of motors" << endl;
        exit(1);
    }

    Writer writer(vector<ControlTableItem>{ControlTableItem::MAX_POSITION_LIMIT}, m_ids, m_models,
                                                portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxPositions);
    writer.syncWrite();
}
*/

/**
 * @brief       Set the same maximum position to all motors
 * @param[in]   maxPosition Max. position for all motors [rad]
 */  
/* 
void MotorHandler::setMaxPosition(float maxPosition)
{
    vector<float> maxPositions(m_nbrMotors, maxPosition);
    setMaxPosition(maxPositions);
}
*/

/**
 * @brief       Set the maximum speed (absolute value) to all motors
 * @note        If all motors have the same max. speed, you can use the overload
 * @param[in]   maxSpeeds Max. absolute speeds for all motors [rad/s]
 */  
/*      
void MotorHandler::setMaxSpeed(std::vector<float> maxSpeeds)
{
    if (maxSpeeds.size() != m_nbrMotors) {
        cout << "Error! The max. speed values do not coincide with the number of motors" << endl;
        exit(1);
    }
    for (int i=0; i<m_nbrMotors; i++) {
        if (maxSpeeds[i] < 0) {
            cout << "Error! Max speed limit set as negative. Exiting" << endl;
            exit(1);
        }
    }

    Writer writer(vector<ControlTableItem>{ControlTableItem::VELOCITY_LIMIT}, m_ids, m_models,
                                            portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxSpeeds);
    writer.syncWrite();
}
*/

/**
 * @brief       Set the same maximum speed (absolute value) to all motors
 * @param[in]   maxSpeed Max. absolute speed for all motors [rad/s]
 */   
/*
void MotorHandler::setMaxSpeed(float maxSpeed)
{
    vector<float> maxSpeeds(m_nbrMotors, maxSpeed);
    setMaxSpeed(maxSpeeds);
}
*/

/**
 * @brief       Set the maximum current (absolute value) to all motors
 * @note        If all motors have the same max. current, you can use the overload
 * @param[in]   maxCurrents Max. absolute currents for all motors [A]
 */   
/*  
void MotorHandler::setMaxCurrent(std::vector<float> maxCurrents)
{
    if (maxCurrents.size() != m_nbrMotors) {
        cout << "Error! The max. current values do not coincide with the number of motors" << endl;
        exit(1);
    }
    for (int i=0; i<m_nbrMotors; i++) {
        if (maxCurrents[i] < 0) {
            cout << "Error! Max current limit set as negative. Exiting" << endl;
            exit(1);
        }
    }

    Writer writer(vector<ControlTableItem>{ControlTableItem::CURRENT_LIMIT}, m_ids, m_models,
                                            portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxCurrents);
    writer.syncWrite();    
}
*/

/**
 * @brief       Set the same maximum current (absolute value) to all motors
 * @param[in]   maxCurrent Max. absolute current for all motors [A]
 */   
/*
void MotorHandler::setMaxCurrent(float maxCurrent)
{
    vector<float> maxCurrents(m_nbrMotors, maxCurrent);
    setMaxCurrent(maxCurrents);
}
*/

/**
 * @brief       Set the maximum PWM (absolute value) to all motors
 * @note        If all motors have the same max. PWM, you can use the overload
 * @param[in]   maxPWMs Max. absolute PWMs for all motors [%]
 */   
/*
void MotorHandler::setMaxPWM(std::vector<float> maxPWMs)
{
    if (maxPWMs.size() != m_nbrMotors) {
        cout << "Error! The max. PWM values do not coincide with the number of motors" << endl;
        exit(1);
    }
    for (int i=0; i<m_nbrMotors; i++) {
        if (maxPWMs[i] < 0) {
            cout << "Error! Max PWM limit set as negative. Exiting" << endl;
            exit(1);
        }
    }

    Writer writer(vector<ControlTableItem>{ControlTableItem::PWM_LIMIT}, m_ids, m_models,
                                        portHandler_, packetHandler_, m_hal, 0);

    writer.addDataToWrite(maxPWMs);
    writer.syncWrite();   
}
*/

/**
 * @brief       Set the same maximum PWM (absolute value) to all motors
 * @param[in]   maxPWM Max. absolute PWM for all motors [%]
 */ 
/*  
void MotorHandler::setMaxPWM(float maxPWM)
{
    vector<float> maxPWMs(m_nbrMotors, maxPWM);
    setMaxPWM(maxPWMs);
}
*/

/******************************************************************************
/ *                           Control and feedback commands
/ ****************************************************************************/

/**
 * @brief       Set the positions of all motors
 * @param[in]   positions Goal positions of all motors [rad]
 */ 
void MotorHandler::setPositions(std::vector<float> positions)
{
    m_positionWriter->send(positions);
}

/**
 * @brief       Get the feedback positions of all motors
 * @param[out]  positions [Output] Vector to hold the feedback positions [rad]
 * @return      1 if reading was successful, 0 otherwise
 */
/*
bool MotorHandler::getPositions(std::vector<float>& positions)
{
    bool readSuccess = m_positionReader->syncRead();
    if (readSuccess) {
        positions = m_positionReader->getReadingResults();
        return true;
    }
    else
        return false;
}
*/

/**
 * @brief       Set the speeds of all motors
 * @param[in]   speeds Goal speeds of all motors [rad/s]
 */ 
/*
void MotorHandler::setSpeeds(std::vector<float> speeds)
{
    m_speedWriter->addDataToWrite(speeds);
    m_speedWriter->syncWrite();
}
*/

/**
 * @brief       Get the feedback speeds of all motors
 * @param[out]  speeds [Output] Vector to hold the feedback speeds [rad/s]
 * @return      1 if reading was successful, 0 otherwise
 */
/*
bool MotorHandler::getSpeeds(std::vector<float>& speeds)
{
    bool readSuccess = m_speedReader->syncRead();
    if (readSuccess) {
        speeds = m_speedReader->getReadingResults();
        return true;
    }
    else
        return false;
}
*/

/**
 * @brief       Set the currents of all motors
 * @param[in]   currents Goal currents of all motors [A]
 */ 
/*
void MotorHandler::setCurrents(std::vector<float> currents)
{
    m_currentWriter->addDataToWrite(currents);
    m_currentWriter->syncWrite();
}
*/

/**
 * @brief       Get the feedback currents of all motors
 * @param[out]  currents [Output] Vector to hold the feedback currents [A]
 * @return      1 if reading was successful, 0 otherwise
 */
/*
bool MotorHandler::getCurrents(std::vector<float>& currents)
{
    bool readSuccess = m_currentReader->syncRead();
    if (readSuccess) {
        currents = m_currentReader->getReadingResults();
        return true;
    }
    else 
        return false;
}
*/

/**
 * @brief       Set the PWMs of all motors
 * @param[in]   pwms Goal PWMs of all motors [%]
 */ 
/*
void MotorHandler::setPWMs(std::vector<float> pwms)
{
    m_PWMWriter->addDataToWrite(pwms);
    m_PWMWriter->syncWrite();
}
*/

/**
 * @brief       Get the feedback PWMs of all motors
 * @param[out]  pwms [Output] Vector to hold the feedback pwms [%]
 * @return      1 if reading was successful, 0 otherwise
 */
/*
bool MotorHandler::getPWMs(std::vector<float>& pwms)
{
    bool readSuccess = m_PWMReader->syncRead();
    if (readSuccess) {
        pwms = m_PWMReader->getReadingResults();
        return true;
    }
    else 
        return false;
}
*/

// Hybrid command mode, to be finished
/*
void MotorHandler::setHybrid(std::vector<float> positions, std::vector<float> currents)
{
    m_positionWriter->addDataToWrite(positions);
    m_currentWriter->addDataToWrite(currents);

    m_positionWriter->syncWrite();
    m_currentWriter->syncWrite();
}

bool MotorHandler::getHybrid(std::vector<float>& positions, std::vector<float>& currents)
{
    bool readPositionSuccess = m_positionReader->syncRead();
    bool readCurrentSuccess = m_currentReader->syncRead();

    if (readPositionSuccess && readCurrentSuccess) {
        positions = m_positionReader->getReadingResults();
        currents = m_currentReader->getReadingResults();
        return true;
    }
    else 
        return false;
}*/

/*
*****************************************************************************
*                             Multiturn mode
****************************************************************************/

/**
 * @brief       Reset multiturn motors flagged as needing a reset.
 * @note        Make sure the motors had enough time to execute the goal position command before 
 *              calling this function. Failure to do so results in undefined behavior.
 */
/*
void MotorHandler::resetMultiturnMotors()
{
    bool needSleep = 0;
    for(int i=0; i<m_nbrMotors; i++) {
        int id = m_ids[i];
        Motor motor = m_hal->getMotorFromID(id);

        if (motor.toReset) {
            needSleep = 1;
            reboot(id);
            m_hal->updateResetStatus(id, 0);
        }
    }

    if (needSleep) {
        usleep(100*1000);  // Wait for the reboot to finish
        enableMotors();
        usleep(5*1000); // Allow the enable
    }
}
*/

}