/**
 *****************************************************************************
 * @file            reader.hpp
 * @brief           Define the Reader class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include "handler.hpp"

namespace KMR::dxlP1
{

/**
 * @brief   Custom Reader class that handles any reading (receiving feedback) from motors
 * @details This custom Reader class simplifies greatly the creation of dynamixel reading handlers. \n 
 *          Each Reader object contains a dynamixel::GroupSyncRead object that enables synchronized
 *          reading from all motors when applicable, otherwise uses the default read function
 */      
class Reader : public Handler
{
public:
	Reader(ControlTableItem field, std::vector<int> ids, std::vector<int> models,
                dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                Hal* hal);
	~Reader();
    
    bool read(std::vector<float>& fbckValues);
    bool read(std::vector<int> ids, std::vector<float>& fbckValues);
    bool read(int id, float& fbckValue);

private:
	dynamixel::GroupBulkRead *m_groupBulkReader = nullptr;
    std::vector<int> m_bulkReadIds;
    std::vector<int> m_basicReadIds;
    bool m_bulkAvailable = 0;

	std::vector<float> m_fbckValues;

    void checkBulkAvailability();
    bool basicRead(std::vector<int> ids);
    bool bulkRead(std::vector<int> ids);
    void fillOutputMatrix(uint32_t paramData , int id);

	void clearParam();
	bool addParam(uint8_t id);
	bool canBeNegative(ControlTableItem field);
};

} 
