/** ble_communication.h **/

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

/**union for conversion of the int/float vals into float so the gui will interpret all
 * the results as floats **/
typedef union _calculated_val_rsp{
	uint8_t int_type[sizeof(float)];
	float float_type;
} calculated_val_rsp;

/** enum determining values which available for the user through ble interface **/
typedef enum _calculated_value{
	RMS_VALUE = 0,
	AVERAGE_VALUE = 1,
	MAX_VALUE = 2,
	MIN_VALUE = 3,
	AMPLITUDE_VALUE = 4,
	CREST_FACTOR_VALUE = 5,
	MAX_CALCULATED_VALUES = 6
} calculated_value;

//////////////////////////////////////////////////////////////////////////////////////////
//Global variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
ble_communication_init
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function initializes the ble communication module.
\****************************************************************************************/
void ble_communication_init(void);

/****************************************************************************************\
Function:
ble_communication_threshold_exceeded_notification_send
******************************************************************************************
Parameters:
uint16_t exceed_value - value which will be send with the notification
******************************************************************************************
Abstract:
This function sends notification to the sensor with the threshold_monitoring 
characteristic as a source of notification.
\****************************************************************************************/
void ble_communication_threshold_exceeded_notification_send(uint16_t exceeded_value);

/****************************************************************************************\
Function:
ble_communication_update_calculated_value
******************************************************************************************
Parameters:
calculated_value type - desired value to update
float val - the value itself
******************************************************************************************
Abstract:
This function updateds internal ble module variables which store the values of the 
calculated indicators. The stored values are accessible by the ble interface.
\****************************************************************************************/
void ble_communication_update_calculated_value(calculated_value type, float val);

/****************************************************************************************\
Function:
ble_communication_is_measurement_requested
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function is a quick check for other modules if the new measurement is requested. It
returns true if yes or false if not.
\****************************************************************************************/
bool ble_communication_is_measurement_requested(void);

/****************************************************************************************\
Function:
ble_communication_get_requested_measurement_duration
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function returns the requested measurement duration value.
\****************************************************************************************/
float ble_communication_get_requested_measurement_duration(void);

/****************************************************************************************\
Function:
ble_communication_get_requested_measurement_frequency
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function returns the requested measurement frequency value.
\****************************************************************************************/
uint16_t ble_communication_get_requested_measurement_frequency(void);

/****************************************************************************************\
Function:
ble_communication_measurement_request_handled
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function indicates the ble module that the measurement request was handled.
\****************************************************************************************/
void ble_communication_measurement_request_handled(void);

/****************************************************************************************\
Function:
ble_communication_update_time_measured_data
******************************************************************************************
Parameters:
uint16_t *data - pointer to the data which should be accessible with the ble interface
uint16_t size - size of the data
******************************************************************************************
Abstract:
This function updates the time measured data inside the ble module.
\****************************************************************************************/
void ble_communication_update_time_measured_data(uint16_t *data, uint16_t size);

/****************************************************************************************\
Function:
ble_communication_update_fft_data
******************************************************************************************
Parameters:
uint16_t *data - pointer to the data which should be accessible with the ble interface
uint16_t size - size of the data
******************************************************************************************
Abstract:
This function updates the fft data inside the ble module.
\****************************************************************************************/
void ble_communication_update_fft_data(uint8_t *data, uint16_t size);

/****************************************************************************************\
Function:
ble_communication_calculation_completed_notification_send
******************************************************************************************
Parameters:
uint16_t zero_val_offset - value of the 0g acceleration
******************************************************************************************
Abstract:
This function sends notification to the user about data accessibility in the sensor.
\****************************************************************************************/
void ble_communication_calculation_completed_notification_send(uint16_t zero_val_offset);

/****************************************************************************************\
Function:
ble_communication_is_threshold_exceed_monitoring_requested
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function is a simple check if the threshold exceeding monitoring is requested. It
returns true if yes and false if not.
\****************************************************************************************/
bool ble_communication_is_threshold_exceed_monitoring_requested(void);

/****************************************************************************************\
Function:
ble_communication_threshold_exceeded_monitoring_handled
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function indicates the ble module that the threshold monitoring was handled.
\****************************************************************************************/
void ble_communication_threshold_exceeded_monitoring_handled(void);

/****************************************************************************************\
Function:
ble_communication_get_threshold_exceed_monitoring_val
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function returns the threshold for monitoring value sent by the user to the ble
interface.
\****************************************************************************************/
uint16_t ble_communication_get_threshold_exceed_monitoring_val(void);

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

#endif /* COMPONENTS_BLE_COMMUNICATION_BLE_COMMUNICATION_H_ */
