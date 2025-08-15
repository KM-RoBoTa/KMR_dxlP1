/**
 *****************************************************************************
 * @file            writer.hpp
 * @brief           Define the Writer class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include <cstdint>
#include "handler.hpp"

namespace KMR::dxlP1
{

/**
 * @brief   Custom Writer class that handles any writing (sending) to motors
 * @details This custom Writer class simplifies greatly the creation of dynamixel writing handlers. \n 
 *          Each Writer object contains a dynamixel::GroupSyncWrite object that enables synchronized
 *          writing to all motors.
 */         
class Writer : public Handler
{
public:
    Writer(ControlTableItem field, std::vector<int> ids, std::vector<int> models,
            dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
            Hal* hal);
    ~Writer();

    template <typename T> void addDataToWrite(std::vector<T> data);
    void syncWrite();

private:
    dynamixel::GroupSyncWrite *m_groupSyncWriter = nullptr;
    uint8_t **m_dataParam = nullptr; // Table containing all parametrized data to be sent

    void populateDataParam(int32_t data, int motor_idx);
    void clearParam();
    bool addParam(uint8_t id, uint8_t *data);

    void multiturnUpdate(int id, float angle);
    bool multiturnOverLimit(float angle);
};


// Templates need to be defined in hpp

/**
 * @brief       Add data to the list to be sent later with syncWrite()
 * @note        If the Writer object handles only one field, you can use the overload
 *              function that does not need the field argument
 * @param[in]   data Data to be sent to all motors handled by this object (eg, new goal positions),
 *              in SI units. NB: If vector of size 1, its value will be sent to all motors
 * @param[in]   field Control field of the data (eg goal position)
 */
template <typename T>
void Writer::addDataToWrite(std::vector<T> data)
{
    for (int i=0; i<m_nbrMotors; i++)
    {
        int id = m_ids[i];
        T current_data = data[i];

        // Transform data into its parametrized form and write it into the parametrized data matrix
        T data = current_data + m_offsets[i];  // Go to the same reference as Dynamixel's SDK

        int32_t parameter = 0;
        int32_t absParam = (int32_t) abs((float)data/m_units[i]);

        if (data >= 0)
            parameter = absParam;
        else
            parameter = (~absParam) + 1;  // 2's complement for negative values

        populateDataParam(parameter, i); 

        if (m_field == ControlTableItem::GOAL_POSITION)
            multiturnUpdate(id, (float)current_data);
    }
}

}
