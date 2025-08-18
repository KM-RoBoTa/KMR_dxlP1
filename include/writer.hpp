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

    template <typename T> bool send(std::vector<T> data, std::vector<int> ids);
    template <typename T> bool send(std::vector<T> data);
    template <typename T> bool send(T data, int id);

private:
    dynamixel::GroupSyncWrite *m_groupSyncWriter = nullptr;
    uint8_t **m_dataParam = nullptr; // Table containing all parametrized data to be sent

    template <typename T> bool calculateParametrizedVals(std::vector<T> data, std::vector<int> ids);
    bool syncWrite(std::vector<int> ids);

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
bool Writer::calculateParametrizedVals(std::vector<T> data, std::vector<int> ids)
{
    for (int i=0; i<ids.size(); i++)
    {
        int idx = getIndex(m_ids, ids[i]);

        if (idx == -1) {
            std::cout << "[KMR_dxlP1] Error! The motor " << ids[i] << " is not handled by this writer" << std::endl;
            return 0;
        }

        T current_data = data[i];

        // Transform data into its parametrized form and write it into the parametrized data matrix
        T data = current_data + m_offsets[idx];  // Go to the same reference as Dynamixel's SDK

        int32_t parameter = 0;
        int32_t absParam = (int32_t) abs((float)data/m_units[idx]);

        if (data >= 0)
            parameter = absParam;
        else
            parameter = (~absParam) + 1;  // 2's complement for negative values

        populateDataParam(parameter, idx); 

        if (m_field == ControlTableItem::GOAL_POSITION)
            multiturnUpdate(ids[i], (float)current_data);
    }

    return 1;
}

template <typename T>
bool Writer::send(std::vector<T> data, std::vector<int> ids)
{
    if (calculateParametrizedVals(data, ids)) {
        bool success = syncWrite(ids);
        return success;
    }
    else
        return 0;
}

template <typename T>
bool Writer::send(std::vector<T> data)
{
    if (calculateParametrizedVals(data, m_ids)) {
        bool success = syncWrite(m_ids);
        return success;
    }
    else
        return 0;
}

template <typename T>
bool Writer::send(T data, int id)
{
    std::vector<T> datas = {data};
    std::vector<int> ids = {id};

    return(send(datas, ids));
}


}
