/**
 *****************************************************************************
 * @file            writer.cpp
 * @brief           Methods of the Writer class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <algorithm>
#include <cstdint>
#include <cmath>

#include "writer.hpp"

using namespace std;

namespace KMR::dxlP1
{

/**
 * @brief       Constructor for a Writer handler
 * @param[in]   list_fields List of fields to be handled by the writer
 * @param[in]   ids Motors to be handled by the writer
 * @param[in]   models Models of the motors to be handled by the writer
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Hal object for interface with hardware
 * @param[in]   forceIndirect 1 to force the Handler as indirect when only 1 field
 */
Writer::Writer(ControlTableItem field, std::vector<int> ids, std::vector<int> models,
            dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
            Hal* hal)
: Handler(field, ids, models, packetHandler, portHandler, hal)
{
    m_groupSyncWriter = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_,
                                                    m_data_address, m_data_byte_size);

    // Create the table to save parametrized data (to be read or sent)
    m_dataParam = new uint8_t *[m_nbrMotors];
    for (int i=0; i<m_ids.size(); i++)
        m_dataParam[i] = new uint8_t[m_data_byte_size];
}

/**
 * @brief Destructor
 */
Writer::~Writer()
{
    delete m_groupSyncWriter;

    for (int i=0; i<m_ids.size(); i++)
        delete[] m_dataParam[i];
    delete[] m_dataParam;

    m_groupSyncWriter = nullptr;
    m_dataParam = nullptr;
}


/*
 *****************************************************************************
 *                             Data writing
 ****************************************************************************/

/**
 * @brief       Save a parametrized data into the general table. Used by addDataToWrite()
 * @param[in]   data Parametrized data to be sent to motor
 * @param[in]   motor_idx Index of the motor
 */
void Writer::populateDataParam(int32_t data, int motor_idx)
{
    if (m_data_byte_size == 4) {
        m_dataParam[motor_idx][0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][1] = DXL_HIBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][2] = DXL_LOBYTE(DXL_HIWORD(data));
        m_dataParam[motor_idx][3] = DXL_HIBYTE(DXL_HIWORD(data));
    }
    else if (m_data_byte_size == 2) {
        m_dataParam[motor_idx][0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][1] = DXL_HIBYTE(DXL_LOWORD(data));
    }
    else if (m_data_byte_size == 1) {
        m_dataParam[motor_idx][0] = DXL_LOBYTE(DXL_LOWORD(data));
    }
    else
        cout << "Wrong number of parameters to populate the parametrized matrix!" << endl;
}

/**
 * @brief       Send the previously prepared data with addDataToWrite to motors
 * @param[in]   ids List of motors who will receive data
 */
bool Writer::syncWrite(std::vector<int> ids)
{
    clearParam();

    for(int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        bool dxl_addparam_result = addParam((uint8_t) ids[i], m_dataParam[idx]);

        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << ids[i] << endl;
            return 0;
        }
    }

    // Send the packet
    int dxl_comm_result = m_groupSyncWriter->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
        return 0;
    }

    return 1;
}


/**
 * @brief   Clear the parameters list
 */
void Writer::clearParam()
{
    m_groupSyncWriter->clearParam();
}


/**
 * @brief       Add data to be written to a motor
 * @param[in]   id ID of the motor
 * @param[in]   data Parametrized data to be sent to the motor
 * @retval      bool: true if data-to-send added to the list successfully
 */
bool Writer::addParam(uint8_t id, uint8_t* data)
{
    bool dxl_addparam_result = m_groupSyncWriter->addParam(id, data);
    return dxl_addparam_result;
}

// !!! Do NOT use with multiturn, it does not check the flag condition
bool Writer::sendParameter(std::vector<int> ids, std::vector<int32_t> parameters)
{
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        populateDataParam(parameters[i], idx);
    }

    bool success = syncWrite(ids);
    return success;
}

bool Writer::sendParameter(std::vector<int32_t> parameters)
{
    return(sendParameter(m_ids, parameters));
}

bool Writer::sendParameter(int id, int32_t parameter)
{
    vector<int> ids = {id};
    vector<int32_t> parameters = {parameter};

    return(sendParameter(ids, parameters));
}

/*
 *****************************************************************************
 *                            Multiturn functions
 ****************************************************************************/

/** 
 * @brief       Set the internal reset request flag in multiturn mode if necessary
 * @param[in]   id ID of the motor
 * @param[in]   angle Current input angle sent to the motor [rad]
 */
void Writer::multiturnUpdate(int id, float angle)
{
    Motor motor = m_hal->getMotorFromID(id);

    if (motor.multiturn && multiturnOverLimit(angle)) {
        m_hal->updateResetStatus(id, 1);
    }
}

/**
 * @brief       Check if the goal position will place the motor over a full turn
 * @param[in]   angle Current goal position sent to a motor [rad]
 * @retval      1 if over a full turn, 0 otherwise
 */
bool Writer::multiturnOverLimit(float angle)
{
    if (angle > 2*M_PI || angle < -2*M_PI)
        return true;
    else
        return false;
}

}