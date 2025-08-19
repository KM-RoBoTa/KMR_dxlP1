/**
 *****************************************************************************
 * @file            reader.cpp
 * @brief           Methods of the Reader class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include <algorithm>
#include <cstdint>

#include "reader.hpp"
#include "utils.hpp"

using namespace std;

namespace KMR::dxlP1
{

const int BITS_PER_BYTE = 8;

/**
 * @brief       Constructor for a Reader handler
 * @param[in]   list_fields List of fields to be handled by the reader
 * @param[in]   ids Motors to be handled by the reader
 * @param[in]   models Models of the motors to be handled by the reader
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Hal object for interface with hardware
 * @param[in]   forceIndirect 1 to force the Handler as indirect when only 1 field
 */
Reader::Reader(ControlTableItem field, vector<int> ids, vector<int> models,
                dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                Hal* hal)
: Handler(field, ids, models, packetHandler, portHandler, hal)
{
    checkBulkAvailability();

    // Create the table to save read data
    m_fbckValues = vector<float>(m_nbrMotors, 0);
}


/**
 * @brief Destructor
 */
Reader::~Reader()
{
    delete m_groupBulkReader;
    m_groupBulkReader = nullptr;
}


void Reader::checkBulkAvailability()
{
    // If there is only 1 motor, bulk read cannot be used
    if(m_nbrMotors == 1)
        m_bulkAvailable = 0;

    // Split IDs between those who can, and those who cannot be read through bulk.
    // Note: a simplistic way would to use the default read as soon as 1 motor
    //      does not support bulk, but this might result in a much slower reading
    //      for no good reason. Bulk read is supported by MX and X series
    else {
        for (int i=0; i<m_nbrMotors; i++) {
            switch (m_models[i])
            {
            case MODEL_NBR_AX_12A:
                m_basicReadIds.push_back(m_ids[i]);
                break;
            
            case MODEL_NBR_MX_64:
                m_bulkReadIds.push_back(m_ids[i]);
                break;

            default:
                break;
            }
        }
    }

    // If there is only 1 motor supporting bulk, once again, bulk cannot be used
    if (m_bulkReadIds.size() == 1) {
        m_bulkAvailable = 0;
        m_bulkReadIds.clear();
        m_basicReadIds.clear();
    } 
    else
        m_groupBulkReader = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
}

bool Reader::read(std::vector<float>& fbckValues)
{
    bool success = 0;

    if (m_bulkAvailable == 0)
        success = basicRead(m_ids);
    else {
        bool successBulk = 1, successBasic = 1;

        successBulk = bulkRead(m_bulkReadIds);
        if (m_basicReadIds.size() > 0)
            successBasic = basicRead(m_basicReadIds);

        if (successBasic && successBulk)
            success = 1;
        else
            success = 0;
    }

    fbckValues = m_fbckValues;

    return success;
}

bool Reader::read(int id, float& fbckValue)
{
    vector<int> ids = {id};
    bool success = basicRead(ids);

    int idx = getIndex(m_ids, id);
    fbckValue = m_fbckValues[idx];

    return success;
}


bool Reader::read(std::vector<int> ids, std::vector<float>& fbckValues)
{
    // Check if the provided ids allow using bulk, basic, or both
    bool split = 0;
    vector<int> tmpBulkReadIds, tmpBasicReadIds;

    if (m_bulkAvailable == 0)
        split = 0;
    else {
        for (int i=0; i<ids.size(); i++) {
            int idx = getIndex(m_bulkReadIds, ids[i]);
            if (idx != -1)
                tmpBulkReadIds.push_back(ids[i]);
            else
                tmpBasicReadIds.push_back(ids[i]);
        }

        if (tmpBulkReadIds.size() < 2)
            split = 0;
        else
            split = 1;
    }

    // Read the values from the motors
    bool success = 1;
    if (!split)
        success = basicRead(ids);
    else {
        bool successBulk = bulkRead(tmpBulkReadIds);

        bool successBasic = 1;
        if (tmpBasicReadIds.size() > 0)
            successBasic = basicRead(tmpBasicReadIds);

        if (successBulk && successBasic)
            success = 1;
        else 
            success = 0;
    }

    // Fill the output vector
    vector<float> fbckVec(ids.size(), 0);
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        fbckVec[i] = m_fbckValues[idx];
    }

    fbckValues = fbckVec;

   return success;
}


bool Reader::basicRead(std::vector<int> ids)
{
    if (ids.size() == 0) {
        cout << "Error! Basic read got sent an empty id vector" << endl;
        return 0; 
    }

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint32_t paramOutput32;
    bool success = 1;

    for (int i=0; i<ids.size(); i++) {

        switch (m_data_byte_size)
        {
        case 1:
            {   // Reminder: cases are not naturally scoped....
            uint8_t outputParam = 0;
            dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, ids[i],
                                        m_data_address, &outputParam, &dxl_error);
            paramOutput32 = (uint32_t) outputParam;
            }
            break;

        case 2:
            {
            uint16_t outputParam = 0;
            dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, ids[i],
                                        m_data_address, &outputParam, &dxl_error);
            paramOutput32 = (uint32_t) outputParam;
            }
            break;

        case 4:
            {
            uint32_t outputParam = 0;
            dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, ids[i],
                                        m_data_address, &outputParam, &dxl_error);
            paramOutput32 = (uint32_t) outputParam;
            } 
            break;
        
        default:
            cout << "ERROR! Reading a byte size of " << m_data_byte_size << " is not supported" << endl;
            exit(1);
            break;
        }

        // Check the success of the read
        if (dxl_comm_result != COMM_SUCCESS) {
            cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
            success = 0;
        }
        else {
            // For some reason, the error "Input voltage error" is constantly reported, 
            // despite the readings being successful.
            // The next part is thus uncommented, hopefully only temporarily until 
            // we find where the issue lies

            //else if (dxl_error != 0) {
            //    cout << packetHandler_->getRxPacketError(dxl_error) << endl;
            //    return 0;
            //}

            // Transform the parametrized value into SI and save it to the matrix
            fillOutputMatrix(paramOutput32, ids[i]);
        }
    }

    return success;
}

bool Reader::bulkRead(std::vector<int> ids)
{
    if (ids.size() == 0) {
        cout << "Error! Bulk read got sent an empty id vector" << endl;
        return 0; 
    }

    bool success = 1;
    clearParam();

    // Add the input motors to the reading list
    for (int i=0; i<ids.size(); i++) {
        bool dxl_addparam_result = addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << ids[i] << endl;
            success = 0;
        }
    }

    // Read the motors' feedbacks
    int dxl_comm_result = m_groupBulkReader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
        success = 0;
    }

    // Transform those feedbacks into SI
    for (int i=0; i<ids.size(); i++) {
        if (m_groupBulkReader->isAvailable(ids[i], m_data_address, m_data_byte_size)) {
            uint32_t paramData = m_groupBulkReader->getData(ids[i], m_data_address, m_data_byte_size);
            fillOutputMatrix(paramData, ids[i]);
        }
        else {
            cout << "Group bulk read data is not available" << endl;
            success = 0;
        }
    }

    return success;
}


/*
 *****************************************************************************
 *                             Data reading
 ****************************************************************************/

/**
 * @brief   Clear the parameters list: no motors added
 */
void Reader::clearParam()
{
    m_groupBulkReader->clearParam();
}

/**
 * @brief       Add a motor to the list of motors who will read
 * @param[in]   id ID of the motor
 * @retval      true if motor added successfully
 */
bool Reader::addParam(uint8_t id)
{
    bool dxl_addparam_result = m_groupBulkReader->addParam(id, m_data_address, m_data_byte_size);
    return dxl_addparam_result;
}


void Reader::fillOutputMatrix(uint32_t paramData , int id)
{
    // Apply 2's complement if the received parameter represents a negative value
    int sign = 0;
    uint32_t absValue = 0;
    if (canBeNegative(m_field)) {

        int shift = m_data_byte_size * BITS_PER_BYTE - 1;

        switch (m_data_byte_size)
        {
        case 1:
        {
            uint8_t param8 = (uint8_t) paramData;
            if ( (param8>>shift) == 0) {
                sign = 1;
                absValue = paramData;
            }
            else {
                sign = -1;
                uint8_t absValue8 = ~(param8-1);
                absValue = (uint32_t) absValue8;
            }
            break;
        }
        case 2:
        {
            uint16_t param16 = (uint16_t) paramData;
            if ( (param16>>shift) == 0) {
                sign = 1;
                absValue = paramData;
            }
            else {
                sign = -1;
                uint16_t absValue16 =  ~(param16-1);
                absValue = (uint32_t) absValue16;
            }
            break;
        }
        case 4:
        {
            uint32_t param32 = (uint32_t) paramData;
            if ( (param32>>shift) == 0) {
                sign = 1;
                absValue = paramData;
            }
            else {
                sign = -1;
                uint32_t absValue32 = ~(param32-1);
                absValue = (uint32_t) absValue32;
            }
            break;    
        }
        default:
            cout << "Error! Field length not supported" << endl;
            exit(1);
            break;
        }
    }
    else {
        sign = 1;
        absValue = paramData;
    }

    // Get SI value
    int idx = getIndex(m_ids, id);
    float data =  (float)absValue * (float)sign * m_units[idx] - m_offsets[idx];

    // Save the converted value into the output matrix
    m_fbckValues[idx] = data;   
}


/** 
 * @brief   Check if the field can have negative feedback values
 * @param   field Query field
 * @return  1 if feedback values can be negative, 0 otherwise
 */
bool Reader::canBeNegative(ControlTableItem field)
{
    switch (field)
    {
    case ControlTableItem::MODEL_NBR:               return false;   break;
    case ControlTableItem::FIRMWARE:                return false;   break;
    case ControlTableItem::ID:                      return false;   break;
    case ControlTableItem::BAUDRATE:                return false;   break;
    case ControlTableItem::RETURN_DELAY:            return false;   break;
    case ControlTableItem::CW_ANGLE_LIMIT:          return true;    break;
    case ControlTableItem::CCW_ANGLE_LIMIT:         return true;    break;
    case ControlTableItem::TEMPERATURE_LIMIT:       return false;   break;
    case ControlTableItem::MIN_VOLTAGE_LIMIT:       return false;   break;
    case ControlTableItem::MAX_VOLTAGE_LIMIT:       return false;   break;
    case ControlTableItem::MAX_TORQUE:              return true;    break;
    case ControlTableItem::STATUS_RETURN:           return false;   break;
    case ControlTableItem::ALARM_LED:               return false;   break;   
    case ControlTableItem::SHUTDOWN:                return false;   break;
    case ControlTableItem::MULTITURN_OFFSET:        return true;    break;
    case ControlTableItem::RESOLUTION_DIVIDER:      return false;   break;

    case ControlTableItem::TORQUE_ENABLE:           return false;   break;
    case ControlTableItem::LED:                     return false;   break;
    case ControlTableItem::D_GAIN:                  return false;   break;
    case ControlTableItem::I_GAIN:                  return false;   break;
    case ControlTableItem::P_GAIN:                  return false;   break;
    case ControlTableItem::GOAL_POSITION:           return true;    break;
    case ControlTableItem::MOVING_SPEED:            return true;    break;
    case ControlTableItem::TORQUE_LIMIT:            return false;   break;
    case ControlTableItem::PRESENT_POSITION:        return true;    break;
    case ControlTableItem::PRESENT_VELOCITY:        return true;    break;
    case ControlTableItem::PRESENT_LOAD:            return false;   break;
    case ControlTableItem::PRESENT_VOLTAGE:         return false;   break;
    case ControlTableItem::PRESENT_TEMPERATURE:     return false;   break;
    case ControlTableItem::REGISTERED:              return false;   break;
    case ControlTableItem::MOVING:                  return false;   break;
    case ControlTableItem::LOCK:                    return false;   break;
    case ControlTableItem::PUNCH:                   return false;   break;
    case ControlTableItem::REALTIME_TICK:           return false;   break;
    case ControlTableItem::CONSUMED_CURRENT:        return false;   break;
    case ControlTableItem::ENABLE_TORQUE_MODE:      return false;   break;
    case ControlTableItem::GOAL_TORQUE:             return true;    break;
    case ControlTableItem::GOAL_ACCELERATION:       return true;    break;

    default:
        cout << "Error! Unknown field being tested for being able to have negative feedback" << endl;
        exit(1);
        break;
    }
}


}