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
        m_motorEnableWriter = getNewWriter(ControlTableItem::TORQUE_ENABLE, m_ids);

        // Integrated base command handlers
        m_positionWriter = getNewWriter(ControlTableItem::GOAL_POSITION, m_ids);
        m_torqueWriter = getNewWriter(ControlTableItem::GOAL_TORQUE, m_ids);
        m_positionReader = getNewReader(ControlTableItem::PRESENT_POSITION, m_ids);
        m_speedReader = getNewReader(ControlTableItem::PRESENT_VELOCITY, m_ids);
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
    deleteWriter(m_torqueWriter);
    deleteReader(m_speedReader);
    deleteReader(m_positionReader);

    // Security against double freeing
    m_motorEnableWriter = nullptr;
    m_positionWriter = nullptr;
    m_torqueWriter = nullptr;
    m_positionReader = nullptr;
    m_speedReader = nullptr;

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
Reader* MotorHandler::getNewReader(ControlTableItem field, vector<int> ids)
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

    Reader* reader = new Reader(field, ids, models, portHandler_, packetHandler_, m_hal);
    return reader;
}

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
void MotorHandler::deleteReader(Reader* reader)
{
    delete reader;
    reader = nullptr; // Security in case of double freeing
}


/*
******************************************************************************
*                         Enable/disable motors
****************************************************************************/

/**
 * @brief   Enable all the motors
 */
bool MotorHandler::enableMotors()
{
    vector<int> vec(m_nbrMotors, ENABLE);
    return(m_motorEnableWriter->send(vec) );
}

bool MotorHandler::enableMotors(std::vector<int> ids)
{
    vector<int> vec(ids.size(), ENABLE);
    return(m_motorEnableWriter->send(ids, vec));
}


bool MotorHandler::enableMotor(int id)
{
    return(m_motorEnableWriter->send(id, ENABLE));
}


/**
 * @brief   disable all the motors
 */
bool MotorHandler::disableMotors()
{
    vector<int> vec(m_nbrMotors, DISABLE);
    return(m_motorEnableWriter->send(vec) );
}

bool MotorHandler::disableMotors(std::vector<int> ids)
{
    vector<int> vec(ids.size(), DISABLE);
    return(m_motorEnableWriter->send(ids, vec));
}

bool MotorHandler::disableMotor(int id)
{
    return(m_motorEnableWriter->send(id, DISABLE));
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

bool MotorHandler::setReturnDelayTime(float val)
{
    Writer writer(ControlTableItem::RETURN_DELAY, m_ids, m_models,
                    portHandler_, packetHandler_, m_hal);

    vector<float> vals(m_nbrMotors, val);
    return(writer.send(vals));
}


/**
 * @brief       Set the minimum voltages to all motors
 * @param[in]   minVoltages Min. allowed voltages in motors [V]
 */                                
bool MotorHandler::setMinVoltage(std::vector<float> minVoltages)
{
    Writer writer(ControlTableItem::MIN_VOLTAGE_LIMIT, m_ids, m_models,
                    portHandler_, packetHandler_, m_hal);

    return(writer.send(minVoltages));
}

/**
 * @brief       Set the same minimum voltage to all motors
 * @param[in]   minVoltage Min. allowed voltage in motors [V]
 */                             
bool MotorHandler::setMinVoltage(float minVoltage)
{
    vector<float> minVoltages(m_nbrMotors, minVoltage);
    return(setMinVoltage(minVoltages));
}

/**
 * @brief       Set the maximum voltage to all motors
 * @param[in]   maxVoltages Max. allowed voltages in motors [V]
 */                                 
bool MotorHandler::setMaxVoltage(std::vector<float> maxVoltages)
{
    Writer writer(ControlTableItem::MAX_VOLTAGE_LIMIT, m_ids, m_models,
                    portHandler_, packetHandler_, m_hal);

    return(writer.send(maxVoltages));
}

/**
 * @brief       Set the same maximum voltage to all motors
 * @param[in]   maxVoltage Max. allowed voltage in motors [V]
 */                              
bool MotorHandler::setMaxVoltage(float maxVoltage)
{
    vector<float> maxVoltages(m_nbrMotors, maxVoltage);
    return(setMaxVoltage(maxVoltages));
}


/****************************************************************************
*                  Setting limits in different operating modes
****************************************************************************/

/**
 * @brief       Set the minimum position to all motors
 * @note        If all motors have the same min. position, you can use the overload
 * @param[in]   minPositions Min. positions for motors [rad]
 */                                 
bool MotorHandler::setMinPosition(std::vector<float> minPositions)
{
    if (minPositions.size() != m_nbrMotors) {
        cout << "Error! The min. position values do not coincide with the number of motors" << endl;
        exit(1);
    }
    
    Writer writer(ControlTableItem::CW_ANGLE_LIMIT, m_ids, m_models,
                    portHandler_, packetHandler_, m_hal);

    return(writer.send(minPositions));
}

/**
 * @brief       Set the same minimum position to all motors
 * @param[in]   minPosition Min. position for all motors [rad]
 */   
bool MotorHandler::setMinPosition(float minPosition)
{
    vector<float> minPositions(m_nbrMotors, minPosition);
    return(setMinPosition(minPositions));
}

/**
 * @brief       Set the maximum position to all motors
 * @note        If all motors have the same max. position, you can use the overload
 * @param[in]   maxPositions Max. positions for motors [rad]
 */                                
bool MotorHandler::setMaxPosition(std::vector<float> maxPositions)
{
    if (maxPositions.size() != m_nbrMotors) {
        cout << "Error! The max. position values do not coincide with the number of motors" << endl;
        exit(1);
    }
    
    Writer writer(ControlTableItem::CCW_ANGLE_LIMIT, m_ids, m_models,
                    portHandler_, packetHandler_, m_hal);

    return(writer.send(maxPositions));
}

/**
 * @brief       Set the same minimum position to all motors
 * @param[in]   maxPosition Min. position for all motors [rad]
 */   
bool MotorHandler::setMaxPosition(float maxPosition)
{
    vector<float> maxPositions(m_nbrMotors, maxPosition);
    return(setMaxPosition(maxPositions));
}

/******************************************************************************
/ *                           Control commands
/ ****************************************************************************/

/**
 * @brief       Set the positions of all motors
 * @param[in]   positions Goal positions of all motors [rad]
 */ 
bool MotorHandler::setPositions(std::vector<float> positions)
{
    return(m_positionWriter->send(positions));
}

bool MotorHandler::setPositions(std::vector<int> ids, std::vector<float> positions)
{
    return(m_positionWriter->send(ids, positions));
}

bool MotorHandler::setPosition(int id, float position)
{
    return(m_positionWriter->send(id, position));
}

/**
 * @brief       Set the currents of all motors
 * @param[in]   currents Goal currents of all motors [A]
 */ 

bool MotorHandler::setTorques(std::vector<float> torques)
{
    return(m_torqueWriter->send(torques));
}

bool MotorHandler::setTorques(std::vector<int> ids, std::vector<float> torques)
{
    return(m_torqueWriter->send(ids, torques));
}

bool MotorHandler::setTorque(int id, float torque)
{
    return(m_torqueWriter->send(id, torque));
}


/******************************************************************************
/ *                           Feedback commands
/ ****************************************************************************/

/**
 * @brief       Get the feedback positions of all motors
 * @param[out]  positions [Output] Vector to hold the feedback positions [rad]
 * @return      1 if reading was successful, 0 otherwise
 */
bool MotorHandler::getPositions(std::vector<float>& positions)
{
    // debug
    timespec start = time_s();
    bool success = m_positionReader->read(positions);
    timespec end = time_s();
    double elapsed = get_delta_us(end, start);
    cout << "elapsed: " << elapsed << " us " << endl;
    return success;
}

bool MotorHandler::getPositions(std::vector<int> ids, std::vector<float>& positions)
{
    return (m_positionReader->read(ids, positions));
}

bool MotorHandler::getPosition(int id, float position)
{
    return(m_positionReader->read(id, position));
}

/**
 * @brief       Get the feedback speeds of all motors
 * @param[out]  speeds [Output] Vector to hold the feedback speeds [rad]
 * @return      1 if reading was successful, 0 otherwise
 */
bool MotorHandler::getSpeeds(std::vector<float>& speeds)
{
    return(m_speedReader->read(speeds));
}

bool MotorHandler::getSpeeds(std::vector<int> ids, std::vector<float>& speeds)
{
    return (m_speedReader->read(ids, speeds));
}

bool MotorHandler::getSpeed(int id, float speed)
{
    return(m_speedReader->read(id, speed));
}

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