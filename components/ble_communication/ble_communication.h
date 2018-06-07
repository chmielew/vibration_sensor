/*
 * ble_communication.h
 *
 *  Created on: 18 mar 2018
 *      Author: chmielew
 */

#ifndef COMPONENTS_BLE_COMMUNICATION_BLE_COMMUNICATION_H_
#define COMPONENTS_BLE_COMMUNICATION_BLE_COMMUNICATION_H_

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "stdint.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:

******************************************************************************************
Parameters:

******************************************************************************************
Abstract:

\****************************************************************************************/
void ble_communication_init(void);

void ble_communication_threshold_exceeded_notification_send(uint16_t exceeded_value);
//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

#endif /* COMPONENTS_BLE_COMMUNICATION_BLE_COMMUNICATION_H_ */
