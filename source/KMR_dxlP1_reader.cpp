/**
 ******************************************************************************
 * @file            KMR_dxlP1_reader.cpp
 * @brief           Defines the Reader class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT 
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxlP1_reader.hpp"
#include <algorithm>
#include <cstdint>

#define MAX_POS         28672
#define UINT_OVERFLOW   65535

using std::cout;
using std::endl;
using std::vector;


namespace KMR::dxlP1
{

/**
 * @brief       Constructor for a Reader handler
 * @param[in]   list_fields List of fields to be handled by the reader
 * @param[in]   ids Motors to be handled by the reader
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Previouly initialized Hal object
 */
Reader::Reader(Fields field, vector<int> ids, dynamixel::PortHandler *portHandler,
                            dynamixel::PacketHandler *packetHandler, Hal* hal)
{
    portHandler_ = portHandler;
    packetHandler_ = packetHandler;
    m_hal = hal;
    m_ids = ids;

    m_field = field;

    getDataByteSize();
    checkMotorCompatibility(field);

    m_groupBulkReader = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);

    // Create the table to save read data
    m_dataFromMotor = new float [m_ids.size()];                     
}


/**
 * @brief Destructor
 */
Reader::~Reader()
{
    // Free the dynamically allocated memory to heap
    delete m_groupBulkReader;
    delete[] m_dataFromMotor;
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
 * @retval      bool: true if motor added successfully
 */
bool Reader::addParam(uint8_t id)
{
    bool dxl_addparam_result = m_groupBulkReader->addParam(id, m_data_address, m_data_byte_size);
    return dxl_addparam_result;
}

/**
 * @brief       Read the handled fields of input motors
 * @param[in]   ids List of motors whose fields will be read 
 */
void Reader::syncRead(vector<int> ids)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = 0;
    uint8_t id;

    clearParam();    

    // Add the input motors to the reading list
    for (int i=0; i<ids.size(); i++){
        id = (uint8_t)ids[i];
        dxl_addparam_result = addParam(id);
        if (dxl_addparam_result != true) {
            cout << "[KMR::dxlP1::Reader] Adding parameters failed for ID = " << ids[i] << endl;
            exit(1);
        }
    }

    // Read the motors' sensors
    dxl_comm_result = m_groupBulkReader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
        //exit(1);
    }

    checkReadSuccessful(ids);
    populateOutputMatrix(ids);
}

/**
 * @brief       Read the handled fields of input motors with the slow, basic read function
 * @note        Should only be used to read positions for AX_12A motors, who cannot use syncRead
 * @param[in]   ids List of motors whose fields will be read 
 * @param[out]  output Vector that will handle the output readings 
 * @retval      1 if read successful, 0 otherwise
 */
int Reader::read(vector<int> ids, vector<float>& output)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    float units, data;
    Fields field = m_field;
    uint32_t paramOutput32;

    for (int i=0; i<ids.size(); i++) {

        if(m_data_byte_size == 1) {
            uint8_t outputParam = 0;
            dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, ids[i],
                                        m_data_address, &outputParam, &dxl_error);
            paramOutput32 = (uint32_t) outputParam;
        }
        else if(m_data_byte_size == 2) {
            uint16_t outputParam = 0;
            dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, ids[i],
                                        m_data_address, &outputParam, &dxl_error);
            paramOutput32 = (uint32_t) outputParam;
        }
        else if(m_data_byte_size == 4) {
            uint32_t outputParam = 0;
            dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, ids[i],
                                        m_data_address, &outputParam, &dxl_error);
            paramOutput32 = (uint32_t) outputParam;
        }
        else {
            cout << "ERROR! Unknown byte size to read" << endl;
            exit(1);
        }

        // Check the success of the read
        if (dxl_comm_result != COMM_SUCCESS) {
            cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
            return 0;
        }

        // For some reason, the error "Input voltage error" is constantly reported, 
        // despite the readings being successful.
        // The next part is thus uncommented, hopefully only temporarily until 
        // we find where the issue lies

        //else if (dxl_error != 0) {
        //    cout << packetHandler_->getRxPacketError(dxl_error) << endl;
        //    return 0;
        //}

        // Transform the parametrized value into SI
        units = m_hal->getControlParametersFromID(ids[i], field).unit;

        if (field != GOAL_POS && field != PRESENT_POS &&
            field != CW_ANGLE_LIMIT && field != CCW_ANGLE_LIMIT) {
            data = paramOutput32 * units;        
        }
        else {
            // In multiturn mode, paramData overflows when position parameter < 0
            if (paramOutput32 > MAX_POS)
                paramOutput32 = paramOutput32 - UINT_OVERFLOW; 
            data = position2Angle(paramOutput32, ids[i], units);
        }
            
        // Save the converted value into the output vector
        output[i] = data;
    }

    return 1;
}


/**
 * @brief       Check if read data from motors is available
 * @param[in]   ids List of motors whose fields have just been read
 */
void Reader::checkReadSuccessful(vector<int> ids)
{
    // Check if groupsyncread data of Dyanamixel is available
    bool dxl_getdata_result = false;
    Fields field = m_field;

    for (int i=0; i<ids.size(); i++) {
        dxl_getdata_result = m_groupBulkReader->isAvailable(ids[i], m_data_address, m_data_byte_size);

        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed \n", ids[i]);
            //exit(1);
        }
    }
}


/**
 * @brief       The reading being successful, save the read data into the output matrix
 * @param[in]   ids List of motors whose fields have been successfully read
 */
void Reader::populateOutputMatrix(vector<int> ids)
{
    Fields field = m_field;
    uint32_t paramData;
    float units, data;
    int id = 0, idx = 0;

    for (int i=0; i<ids.size(); i++) {
        id = ids[i];
        units = m_hal->getControlParametersFromID(id, field).unit;

        paramData = m_groupBulkReader->getData(id, m_data_address, m_data_byte_size);

        // Transform data from parametrized value to SI units
        if (field != GOAL_POS && field != PRESENT_POS &&
            field != CW_ANGLE_LIMIT && field != CCW_ANGLE_LIMIT) {
            data = paramData * units;        
        }
        else {
            // In multiturn mode, paramData overflows when position parameter < 0
            if (paramData > MAX_POS)
                paramData = paramData - UINT_OVERFLOW; 
            data = position2Angle(paramData, id, units);
        }
            
        // Save the converted value into the output matrix
        idx = getMotorIndexFromID(id);
        m_dataFromMotor[idx] = data;
    }
}


/**
 * @brief       Convert position into angle based on motor model 
 * @param[in]   position Position to be converted
 * @param[in]   id ID of the motor
 * @param[in]   units Conversion units between the position and the angle
 * @return      Angle position [rad] of the query motor
 */
float Reader::position2Angle(int32_t position, int id, float units)
{
    float angle;

    int motor_idx = m_hal->getMotorsListIndexFromID(id);
    int model = m_hal->m_motors_list[motor_idx].scanned_model;
    int model_max_position;

    if (model == 1030 || model == 1000 || model == 310){
    	model_max_position = 4095;
        
        angle = ((float) position - model_max_position/2) * units;
    }
    else if (model == 12) {  // AX_12A
        model_max_position = 1023;
        angle = ((float) position - model_max_position/2) * units;
    }
    else {
        cout << "This model is unknown, cannot calculate angle from position!" << endl;
        return (1);
    }

    return angle;
}

}