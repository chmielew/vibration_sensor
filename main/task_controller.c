/** task_creator.c **/

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "task_controller.h"

#include <stddef.h>

#include "../components/heartbeat/heartbeat.h"
#include "../components/threshold_exceeded_notification/threshold_exceeded_notification.h"
#include "../components/calculation/calculation.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Local typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Static variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

/** array conatining the handles to each created task **/
static TaskHandle_t task_handle_array[TASK_HANDLE_SIZE] = { NULL };

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

void entry_task_creator(void)
{
	xTaskCreate(&heartbeat_task, "heartbeat_task", configMINIMAL_STACK_SIZE, NULL,
				   	5, &(task_handle_array[HEARTBEAT_TASK_HANDLE]));
	xTaskCreate(&threshold_exceeded_task, "threshold_exceeded_task", 2048, NULL,
				   	5, &(task_handle_array[THRESHOLD_EXCEEDED_TASK_HANDLE]));
	xTaskCreate(&calculation_rms, "calculation_rms_task", 2048, NULL, 5,
				   	&(task_handle_array[CALCULATION_RMS_TASK_HANDLE]));
	xTaskCreate(&calculation_average, "calculation_average_task", 2048, NULL,
				   	5, &(task_handle_array[CALCULATION_AVERAGE_TASK_HANDLE]));
	xTaskCreate(&calculation_range, "calculation_range_task", 2048, NULL,
				   	5, &(task_handle_array[CALCULATION_RANGE_TASK_HANDLE]));
	xTaskCreate(&calculation_amplitude, "calculation_amplitude_task", 2048, NULL,
				   	5, &(task_handle_array[CALCULATION_AMPLITUDE_TASK_HANDLE]));
	xTaskCreate(&calculation_crest_factor, "calculation_crest_factor_task", 2048, NULL,
				   	5, &(task_handle_array[CALCULATION_CREST_FACTOR_TASK_HANDLE]));
}
/****************************************************************************************/

TaskHandle_t get_task_handle(task_handle task_hnd)
{
	return task_handle_array[task_hnd];
}

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////
