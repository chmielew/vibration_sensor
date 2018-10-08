//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "calculation.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/task.h"
#include "../../main/task_controller.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
#define CALC_FINISH_RMS_FLAG		((uint8_t)0x1<<0)
#define CALC_FINISH_AVERAGE_FLAG	((uint8_t)0x1<<1)
#define CALC_FINISH_RANGE_FLAG		((uint8_t)0x1<<2)
#define CALC_FINISH_AMPLITUDE_FLAG	((uint8_t)0x1<<3)
#define CALC_FINISH_CREST_FLAG		((uint8_t)0x1<<4)
#define is_calc_finished(x)			((0x1f) == x ? CALCULATION_FINISHED : CALCULATION_IN_PROGRESS)

//////////////////////////////////////////////////////////////////////////////////////////
//Local typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Static variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

/** obj structure implementation hidden under handle **/
struct Calculation_obj {
	uint16_t size;
	float rms;
	float average;
	uint16_t max_val;
	uint16_t min_val;
	uint16_t amplitude;
	float crest_factor;
	calculation_state state;
	uint8_t finish_flags;
	uint16_t * data;
};

/** queues handles for calculation tasks **/
static QueueHandle_t xQueue_rms, xQueue_average, xQueue_range, xQueue_amplitude, xQueue_crest_factor;

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
sum_array
******************************************************************************************
Parameters:
uint16_t data[] - pointer to data array which will be summed
uint16_t size - size of data
******************************************************************************************
Abstract:
This function sums all the elements of the data array.
\****************************************************************************************/
static uint32_t sum_array(uint16_t data[], uint16_t size);

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

Calculation_obj_handle calculation_new_obj(uint16_t data[], uint16_t size)
{
	struct Calculation_obj * instance = malloc(sizeof(struct Calculation_obj));
	instance->data = data;
	instance->size = size;
	instance->state = CALCULATION_INITIALIZED;
	instance->finish_flags = 0;
	xQueue_rms = xQueueCreate(1, sizeof(Calculation_obj_handle));
	xQueue_average = xQueueCreate(1, sizeof(Calculation_obj_handle));
	xQueue_range = xQueueCreate(1, sizeof(Calculation_obj_handle));
	xQueue_amplitude = xQueueCreate(1, sizeof(Calculation_obj_handle));
	xQueue_crest_factor = xQueueCreate(1, sizeof(Calculation_obj_handle));
	return instance;
}
/****************************************************************************************/

uint16_t calculation_get_size(Calculation_obj_handle obj)
{
	return obj->size;
}
/****************************************************************************************/

uint16_t * calculation_get_data_ptr(Calculation_obj_handle obj)
{
	return obj->data;
}
/****************************************************************************************/

void calculation_calculate_factors(Calculation_obj_handle obj)
{
	xQueueSend(xQueue_rms, (void *)&obj, 0);
	xQueueSend(xQueue_average, (void *)&obj, 0);
	xQueueSend(xQueue_range, (void *)&obj, 0);
	xQueueSend(xQueue_amplitude, (void *)&obj, 0);
	xQueueSend(xQueue_crest_factor, (void *)&obj, 0);

	while(xQueue_rms == 0 ||
			xQueue_average == 0 ||
			xQueue_range == 0 ||
			xQueue_amplitude == 0 ||
			xQueue_crest_factor == 0){

	}

	vTaskResume(get_task_handle(CALCULATION_RMS_TASK_HANDLE));
	vTaskResume(get_task_handle(CALCULATION_AVERAGE_TASK_HANDLE));
	vTaskResume(get_task_handle(CALCULATION_RANGE_TASK_HANDLE));
	vTaskResume(get_task_handle(CALCULATION_AMPLITUDE_TASK_HANDLE));
	vTaskResume(get_task_handle(CALCULATION_CREST_FACTOR_TASK_HANDLE));

}
/****************************************************************************************/

calculation_factor_type calculation_get_factor(Calculation_obj_handle obj, calculation_factors factor)
{
	switch(factor){
	case CALCULATION_RMS:
		return (calculation_factor_type)obj->rms;
		break;
	case CALCULATION_AVERAGE:
		return (calculation_factor_type)obj->average;
		break;
	case CALCULATION_MAXVAL:
		return (calculation_factor_type)obj->max_val;
		break;
	case CALCULATION_MINVAL:
		return (calculation_factor_type)obj->min_val;
		break;
	case CALCULATION_AMPLITUDE:
		return (calculation_factor_type)obj->amplitude;
		break;
	case CALCULATION_CREST_FACTOR:
		return (calculation_factor_type)obj->crest_factor;
		break;
	default:
		return (calculation_factor_type)(uint16_t)0;
		break;
	}
}
/****************************************************************************************/

calculation_state calculation_get_state(Calculation_obj_handle obj)
{
	return obj == NULL ? CALCULATION_NOT_INITIALIZED : obj->state;
}
/****************************************************************************************/

void calculation_delete_obj(Calculation_obj_handle * obj)
{
	vQueueDelete(xQueue_rms);
	vQueueDelete(xQueue_average);
	vQueueDelete(xQueue_range);
	vQueueDelete(xQueue_amplitude);
	vQueueDelete(xQueue_crest_factor);
	free(*obj);
	*obj = NULL;
}
/****************************************************************************************/

void calculation_rms(void *pvParameter)
{
	while(true){
		Calculation_obj_handle obj;
			if(xQueue_rms != 0 && xQueueReceive(xQueue_rms, &obj, 0)){
			float sum = 0;
			for (int16_t iterator = (obj->size - 1); iterator >= 0; --iterator) {
				sum += pow((obj->data[iterator]), 2.0);
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}
			obj->rms = sqrt(sum/obj->size);
			obj->finish_flags |= CALC_FINISH_RMS_FLAG;
			obj->state = is_calc_finished(obj->finish_flags);
		}
		vTaskSuspend(NULL);
	}
}
/****************************************************************************************/

void calculation_average(void *pvParameter)
{
	while(true){
		Calculation_obj_handle obj;
		if(xQueue_average != 0 && xQueueReceive(xQueue_average, &obj, 0)){
			uint32_t sum = sum_array(obj->data, obj->size);
			obj->average = sum/obj->size;
			obj->finish_flags |= CALC_FINISH_AVERAGE_FLAG;
			obj->state = is_calc_finished(obj->finish_flags);
		}
		vTaskSuspend(NULL);
	}
}
/****************************************************************************************/

void calculation_range(void *pvParameter)
{
	while(true){
		Calculation_obj_handle obj;
		if(xQueue_range != 0 && xQueueReceive(xQueue_range, &obj, 0)){
		obj->max_val = obj->data[0];
		obj->min_val = obj->data[0];
		for (int16_t iterator = (obj->size - 1); iterator >= 0; --iterator){
			if(obj->data[iterator] > obj->max_val){
				obj->max_val = obj->data[iterator];
			}else if(obj->data[iterator] < obj->min_val){
				obj->min_val = obj->data[iterator];
			}
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		obj->finish_flags |= CALC_FINISH_RANGE_FLAG;
		obj->state = is_calc_finished(obj->finish_flags);
		}
		vTaskSuspend(NULL);
	}
}
/****************************************************************************************/

void calculation_amplitude(void *pvParameter)
{
	while(true){
		Calculation_obj_handle obj;
		if(xQueue_amplitude != 0 && xQueueReceive(xQueue_amplitude, &obj, 0)){
			while (!(obj->finish_flags & (CALC_FINISH_AVERAGE_FLAG | CALC_FINISH_RANGE_FLAG))) {
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}

			uint16_t amplitude = (abs(obj->max_val - obj->average) >= abs(obj->min_val - obj->average) ?
					abs(obj->max_val - obj->average) : abs(obj->min_val - obj->average));
			obj->amplitude = amplitude;
			obj->finish_flags |= CALC_FINISH_AMPLITUDE_FLAG;
			obj->state = is_calc_finished(obj->finish_flags);
		}
		vTaskSuspend(NULL);
	}
}
/****************************************************************************************/

void calculation_crest_factor(void *pvParameter)
{
	while(true){
		Calculation_obj_handle obj;
		if(xQueue_crest_factor != 0 && xQueueReceive(xQueue_crest_factor, &obj, 0)){
			while(!(obj->finish_flags & (CALC_FINISH_RANGE_FLAG | CALC_FINISH_RMS_FLAG))){
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
			obj->crest_factor = obj->max_val/obj->rms;
			obj->finish_flags |= CALC_FINISH_CREST_FLAG;
			obj->state = is_calc_finished(obj->finish_flags);
		}
		vTaskSuspend(NULL);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

static uint32_t sum_array(uint16_t data[], uint16_t size)
{
	int32_t sum = 0;
	for (int16_t iterator = (size - 1); iterator >= 0; --iterator) {
		sum += data[iterator];
	}
	return sum;
}

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////
