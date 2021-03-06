/** heartbeat.h **/

#ifndef COMPONENTS_HEARTBEAT_HEARTBEAT_H_
#define COMPONENTS_HEARTBEAT_HEARTBEAT_H_

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////

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
heartbeat_init
******************************************************************************************
Parameters:
None
******************************************************************************************
Abstract:
This function initializes the chosen BLINK_GPIO as output.
\****************************************************************************************/
void heartbeat_init(void);

/****************************************************************************************\
Function:
heartbeat_task
******************************************************************************************
Parameters:
void *pvParameter - standard parameter needed for task creation
******************************************************************************************
Abstract:
Simple heartbeat task, which toggles output on chosen pin to blink the LED.
\****************************************************************************************/
void heartbeat_task(void *pvParameter);

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////
#endif /* COMPONENTS_HEARTBEAT_HEARTBEAT_H_ */
