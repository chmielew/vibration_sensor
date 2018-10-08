/** threshold_exceeded_notification.h **/

#ifndef COMPONENTS_THRESHOLD_EXCEEDED_NOTIFICATION_H_
#define COMPONENTS_THRESHOLD_EXCEEDED_NOTIFICATION_H_

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
threshold_exceeded_init
******************************************************************************************
Parameters:
uint16_t zero_val - zero acceleration value
******************************************************************************************
Abstract:
This function intializes the threshold exceeded module. The zero_valu is used to set the
reference.
\****************************************************************************************/
void threshold_exceeded_init(uint16_t zero_val);

/****************************************************************************************\
Function:
threshold_exceeded_task
******************************************************************************************
Parameters:
void *pvParameter - standard task parameter 
******************************************************************************************
Abstract:
This is the main task for the threshold monitoring. If the task is active and the 
threshold is non-zero valu the task checks the adc conversion result every 1 ms and 
evaluates if the threshold was exceeded. If the threshold is a zero value, the task 
suspends itself. It also stores tha max value during monitoring inside the module,
and shares it with the get function.
\****************************************************************************************/
void threshold_exceeded_task(void *pvParameter);

/****************************************************************************************\
Function:
threshold_exceeded_reset
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function resets the threshold and the max stored value inside the module. 
\****************************************************************************************/
void threshold_exceeded_reset(void);

/****************************************************************************************\
Function:
threshold_exceeded_set_threshold
******************************************************************************************
Parameters:
uint16_t threshold - desired threshold for monitoring
******************************************************************************************
Abstract:
This function sets the desired threshold for monitoring.
\****************************************************************************************/
void threshold_exceeded_set_threshold(uint16_t threshold);

/****************************************************************************************\
Function:
threshold_exceeded_get_max_val_raw
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function is a getter for the maximal stored value inside the module.
\****************************************************************************************/
uint16_t threshold_exceeded_get_max_val_raw(void);

/****************************************************************************************\
Function:
threshold_exceeded_get_max_val_voltage
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function is a getter for the max stored val converted to voltage.
\****************************************************************************************/
float threshold_exceeded_get_max_val_voltage(void);

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

#endif /* COMPONENTS_THRESHOLD_EXCEEDED_NOTIFICATION_H_ */
