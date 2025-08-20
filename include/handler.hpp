/**
 *****************************************************************************
 * @file            handler.hpp
 * @brief           Define the Handler class
 *****************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#pragma once

#include <cstdint>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hal.hpp"
#include "utils.hpp"


namespace KMR::dxlP1
{

/**
 * @brief       Abstract parent class, to be specialized as a Reader or Writer
 * @details		This class is not usable by itself, it is a non-specialized skeleton inherited
 * 				by the child classes Reader and Writer. \n
 * 				It contains functionalities to check the viability of sync/bulk readers/writers 
 * 				that will be defined in child classes (motor compatibility). \n 
 */
class Handler
{
protected:
	int m_nbrMotors;							// Number of handled motors
	std::vector<int> m_ids;						// IDs of motors handled by this Handler
	std::vector<int> m_models;					// Models of the handled motors

	ControlTableItem m_field;		            // Field handled by this specific handler

	dynamixel::PacketHandler *packetHandler_;	// Handler for communication packets
	dynamixel::PortHandler *portHandler_;		// Handler for the serial port
	Hal* m_hal;									// Hal object for interface with hardware

	uint8_t m_data_address = -1;				// Address where the data is written/read
	uint8_t m_data_byte_size = 0;				// Total data byte size handled by the handler	

	// SI-to-parameters conversion variables
	std::vector<float> m_units;	    // Units to convert from SI to parameter
	std::vector<float> m_offsets;	// SI offsets for custom references
	
	Handler(ControlTableItem field, std::vector<int> ids, std::vector<int> models,
			dynamixel::PacketHandler* packetHandler, dynamixel::PortHandler* portHandler,
			Hal* hal);
	virtual ~Handler() = default;				// Dstr needs to be virtual to avoid undef. behavior

	// Methods that need to be implemented in child classes
	virtual void clearParam() = 0; // Pure virtual function


private:	
	// Initialization functions on constructor call
	
	void getDataByteSize();
	void checkMotorCompatibility();
	void getConversionVariables();
};

} 
