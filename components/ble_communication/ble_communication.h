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
#include "stdbool.h"
//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////
typedef union _calculated_val_rsp{
	uint8_t int_type[sizeof(float)];
	float float_type;
}calculated_val_rsp;

typedef enum _calculated_value{
	RMS_VALUE = 0,
	AVERAGE_VALUE = 1,
	MAX_VALUE = 2,
	MIN_VALUE = 3,
	AMPLITUDE_VALUE = 4,
	CREST_FACTOR_VALUE = 5,
	MAX_CALCULATED_VALUES = 6
}calculated_value;

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

void ble_communication_update_calculated_value(calculated_value type, float val);

bool ble_communication_is_measurement_requested(void);

float ble_communication_get_requested_measurement_duration(void);
uint16_t ble_communication_get_requested_measurement_frequency(void);
void ble_communication_measurement_request_handled(void);

void ble_communication_update_time_measured_data(uint16_t *data, uint16_t size);
void ble_communication_update_fft_data(uint8_t *data, uint16_t size);
void ble_communication_calculation_completed_notification_send(void);
//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

#endif /* COMPONENTS_BLE_COMMUNICATION_BLE_COMMUNICATION_H_ */
