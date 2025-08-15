/**
 ******************************************************************************
 * @file            hal.cpp
 * @brief           Methods of the Hal class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 *****************************************************************************
 */

#include "hal.hpp"
#include "utils.hpp"


using namespace std;

namespace KMR::dxlP1

#define POS_OFFSET_DEFAULT 3.14159265358979323846264338327950288 // M_PI
{

/**
 * @brief   Constructor for Hal
 */
Hal::Hal()
{
    AX_12A = new AX_12A_P1();
    MX_64 = new MX_64_P1();
}

/**
 * @brief   Destructor for Hal
 */
Hal::~Hal()
{
    delete AX_12A;
    delete MX_64;  

    // Protection against accidental double freeing
    AX_12A = nullptr;
    MX_64 = nullptr;
}

/**
 * @brief   Initialize the Hal - called by MotorHandler in its cstr
 * @param   ids List of IDs of all the motors in the robot
 * @param   nbrMotors Number of motors in the robot
 * @param   models List of model numbers of each motor, gotten during the motors ping
 */
void Hal::init(vector<int> ids, int nbrMotors, vector<int> models)
{
    m_nbrMotors = nbrMotors;
    m_ids = ids;
    m_models =  models;

    for (int i=0; i<m_nbrMotors; i++) {
        Motor motor(m_ids[i], m_models[i]);
        m_motorsList.push_back(motor);
    }
}


/*****************************************************************************
 *                     Query functions from outside
 ****************************************************************************/

/**
 * @brief       Get a motor's info structure from motor ID
 * @param[in]   id ID of the query motor
 * @retval      The Motor structure of the query motor
 */
Motor Hal::getMotorFromID(int id)
{
    int motor_idx = getIndex(m_ids, id);
    Motor motor = m_motorsList[motor_idx];

    return motor;
}

/**
 * @brief       Get our custom position offset, so that the 0 angle is in the center
 * @param[in]   modelNumber Query motor model number
 * @return      Position offset [rad]
 */
float Hal::getPositionOffset(int modelNumber)
{
    float offset = 0;

    // Insert any special case here (eg AX-12A in protocol 1)
    offset = POS_OFFSET_DEFAULT;
            
    // TODO
    return offset;
}

/**
 * @brief       Get a specific control field corresponding to the input motor model number
 * @param[in]   modelNumber Query motor model number
 * @param[in]   item Query field in the control table (ex: GOAL_POSITION)
 * @return      Control field corresponding to the query
 */
Field Hal::getControlFieldFromModel(int modelNumber, ControlTableItem item)
{
    ControlTable motor = getControlTable(modelNumber);
    Field field = getControlField(motor, item);

    return field;
}


/*****************************************************************************
 *                     Get hardware info, in private scope
 ****************************************************************************/

/**
 * @brief       Get the control table corresponding to the input motor model number
 * @param[in]   modelNumber Query motor model number
 * @return      Control table corresponding to the query motor
 */
ControlTable Hal::getControlTable(int modelNumber)
{
    ControlTable motor;

    switch (modelNumber) {

    case MODEL_NBR_AX_12A: 
        motor = *AX_12A; break;
    case MODEL_NBR_MX_64:
        motor = *MX_64; break;

    default:
        cout << "Error: this model is unknown! Exiting" << endl;
        exit(1);
    }

    return motor;
}


/**
 * @brief       Extract a specific control field from the input control table
 * @param[in]   motor Control table of the query motor, previously gotten with getControlTable()
 * @param[in]   item Query field in the control table (ex: GOAL_POSITION)
 * @return      Control field corresponding to the query
 */
Field Hal::getControlField(ControlTable motor, ControlTableItem item)
{
    Field field;

    switch (item) {
    case ControlTableItem::MODEL_NBR:               field = motor.modelNumber;              break;
    case ControlTableItem::FIRMWARE:                field = motor.firmware;                 break;
    case ControlTableItem::ID:                      field = motor.id;                       break;
    case ControlTableItem::BAUDRATE:                field = motor.baudrate;                 break;
    case ControlTableItem::RETURN_DELAY:            field = motor.returnDelayTime;          break;
    case ControlTableItem::CW_ANGLE_LIMIT:          field = motor.CW_angleLimit;            break;
    case ControlTableItem::CCW_ANGLE_LIMIT:         field = motor.CCW_angleLimit;           break;
    case ControlTableItem::TEMPERATURE_LIMIT:       field = motor.temperatureLimit;         break;
    case ControlTableItem::MIN_VOLTAGE_LIMIT:       field = motor.minVoltageLimit;          break;
    case ControlTableItem::MAX_VOLTAGE_LIMIT:       field = motor.maxVoltageLimit;          break;
    case ControlTableItem::MAX_TORQUE:              field = motor.maxTorque;                break;
    case ControlTableItem::STATUS_RETURN:           field = motor.statusReturn;             break;
    case ControlTableItem::ALARM_LED:               field = motor.alarmLed;                 break;
    case ControlTableItem::SHUTDOWN:                field = motor.shutdown;                 break;
    case ControlTableItem::MULTITURN_OFFSET:        field = motor.multiturnOffset;          break;
    case ControlTableItem::RESOLUTION_DIVIDER:      field = motor.resolutionDivider;        break;


    case ControlTableItem::TORQUE_ENABLE:           field = motor.torqueEnable;             break; 
    case ControlTableItem::LED:                     field = motor.LED;                      break;
    case ControlTableItem::D_GAIN:                  field = motor.D_gain;                   break;
    case ControlTableItem::I_GAIN:                  field = motor.I_gain;                   break;
    case ControlTableItem::P_GAIN:                  field = motor.P_gain;                   break;
    case ControlTableItem::GOAL_POSITION:           field = motor.goalPosition;             break;
    case ControlTableItem::MOVING_SPEED:            field = motor.movingSpeed;              break;
    case ControlTableItem::TORQUE_LIMIT:            field = motor.torqueLimit;              break;
    case ControlTableItem::PRESENT_POSITION:        field = motor.presentPosition;          break;
    case ControlTableItem::PRESENT_VELOCITY:        field = motor.presentVelocity;          break;
    case ControlTableItem::PRESENT_LOAD:            field = motor.presentLoad;              break;
    case ControlTableItem::PRESENT_VOLTAGE:         field = motor.presentVoltage;           break;
    case ControlTableItem::PRESENT_TEMPERATURE:     field = motor.presentTemperature;       break;
    case ControlTableItem::REGISTERED:              field = motor.registered;               break;
    case ControlTableItem::MOVING:                  field = motor.moving;                   break;
    case ControlTableItem::LOCK:                    field = motor.lock;                     break;
    case ControlTableItem::PUNCH:                   field = motor.punch;                    break;
    case ControlTableItem::REALTIME_TICK:           field = motor.realtimeTick;             break;
    case ControlTableItem::CONSUMED_CURRENT:        field = motor.consumedCurrent;          break;
    case ControlTableItem::ENABLE_TORQUE_MODE:      field = motor.enableTorqueMode;         break;
    case ControlTableItem::GOAL_TORQUE:             field = motor.goalTorque;               break;
    case ControlTableItem::GOAL_ACCELERATION:       field = motor.goalAcceleration;         break;

    default:
        cout << "Error: this field is unknown! Exiting" << endl;
        exit(1);
    }    

    if (field.length == UNDEF) {
        cout << "Error: this field does not exist for this motor or protocol!" << endl;
        exit(1);
    }

    return field;
}


/*****************************************************************************
 *                           Multiturn functions
 ****************************************************************************/

/**
 * @brief       Save the input motor as being in multiturn mode
 * @param[in]   id ID of the multiturn motor
 */
void Hal::setMultiturnMode(int id)
{
    int idx = getIndex(m_ids, id);
    if (idx < 0) {
        cout << "Error! Unknown ID set as a multiturn motor" << endl;
        exit(1);
    }
    m_motorsList[idx].multiturn = true;    
} 

/**
 * @brief       Update a motor's "to reset" status in multiturn mode
 * @param[in]   id ID of the query motor
 * @param[in]   status Boolean: 1 if need to reset, 0 if not
 */
void Hal::updateResetStatus(int id, int status)
{
    int idx = getIndex(m_ids, id);
    m_motorsList[idx].toReset = status;
}

}