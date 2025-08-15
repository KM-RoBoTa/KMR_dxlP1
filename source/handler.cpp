/**
 *****************************************************************************
 * @file            handler.cpp
 * @brief           Methods of the Handler class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include "handler.hpp"
#include <algorithm>
#include <cstdint>

using namespace std;

namespace KMR::dxlP1
{

/*
 *****************************************************************************
 *                                Initializations
 ****************************************************************************/

/** 
 * @brief       Constructor for Handler
 * @param[in]   list_fields Fields handled by this Handler
 * @param[in]   ids Motors handled by this Handler
 * @param[in]   models Models of the handled motors
 * @param[in]   packetHandler Handler of the communication packets
 * @param[in]   portHandler Handler of the serial port
 * @param[in]   hal Hal object for interface with hardware
 */
Handler::Handler(ControlTableItem field, vector<int> ids, vector<int> models,
                 dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler,
                 Hal* hal)
{
    m_ids = ids;
    m_models = models;
    m_nbrMotors = ids.size();
    packetHandler_ = packetHandler;
    portHandler_ = portHandler;
    m_hal = hal;
    m_field = field;   

    getDataByteSize();
    checkMotorCompatibility();

    getConversionVariables();
}

/**
 * @brief       Calculate and store the byte length of data read/written by the handler. \n 
 *              Also check if the motors are field-compatible (same data lengths for a given field)
 */
void Handler::getDataByteSize()
{
    uint8_t length = 0, length_prev = 0;
    
    for (int j=1; j<m_ids.size(); j++){
        length = m_hal->getControlFieldFromModel(m_models[j], m_field).length;
        length_prev = m_hal->getControlFieldFromModel(m_ids[j-1], m_field).length;       

        if(length != length_prev){
            cout << "Motors " << m_ids[j] << " and " << m_ids[j-1] << " have incompatible field lengths!" << endl;
            exit(1);
        }
    }

    if (m_ids.size() == 1)
        length = m_hal->getControlFieldFromModel(m_ids[0], m_field).length;

    m_data_byte_size = length;
}


/**
 * @brief       Check if the motors are compatible for a given field: same address for data storing
 */
void Handler::checkMotorCompatibility()
{
    uint8_t address = -1;
    uint8_t address_prev = -1;
    int id = -1, id_prev = -1;
  
    for(int i=1; i<m_ids.size(); i++){
        address = m_hal->getControlFieldFromModel(m_models[i], m_field).addr;
        address_prev = m_hal->getControlFieldFromModel(m_models[i-1], m_field).addr;

        if(address != address_prev){
            cout << "Motors " << m_ids[i] << " and " << m_ids[i-1] << " have incompatible addresses!" << endl;
            exit(1);
        }
    }                                                                                                                                                                                                                                                                                                                        

    if (m_ids.size() == 1)
        address = m_hal->getControlFieldFromModel(m_models[0], m_field).addr;

    m_data_address = address;
}


/**
 * @brief   Get the conversion variables (units and offsets) between the SI units and parameters
 */
void Handler::getConversionVariables()
{
    m_units = vector<float>(m_nbrMotors);
    m_offsets = vector<float>(m_nbrMotors);

    for (int i=0; i<m_nbrMotors; i++) {
        float unit = m_hal->getControlFieldFromModel(m_models[i], m_field).unit;
        float offset = 0;

        if (m_field == ControlTableItem::GOAL_POSITION      ||
            m_field == ControlTableItem::PRESENT_POSITION   ||
            m_field == ControlTableItem::CW_ANGLE_LIMIT     || 
            m_field == ControlTableItem::CCW_ANGLE_LIMIT
            )
            offset = m_hal->getPositionOffset(m_models[i]);

        m_units[i] = unit;
        m_offsets[i] = offset;
    }
} 

}