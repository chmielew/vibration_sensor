//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "calculation.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "fft.h"

#include "esp_log.h"
//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Local typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Static variables																		//
//////////////////////////////////////////////////////////////////////////////////////////
struct Calculation_obj {
	uint16_t size;
	float rms;
	float average;
	uint16_t max_val;
	uint16_t min_val;
	uint16_t amplitude;
	float crest_factor;
	uint16_t data[];
};

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////
/****************************************************************************************\
Function:

******************************************************************************************
Parameters:

******************************************************************************************
Abstract:

\****************************************************************************************/
static void calculation_rms(Calculation_obj_handle obj);

static void calculation_average(Calculation_obj_handle obj);

static void calculation_range(Calculation_obj_handle obj);

static void calculation_amplitude(Calculation_obj_handle obj);

static void calculation_crest_factor(Calculation_obj_handle obj);

static uint32_t sum_array(uint16_t data[], uint16_t size);

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

Calculation_obj_handle calculation_NewObj(uint16_t data[], uint16_t size)
{
	struct Calculation_obj* instance = malloc(sizeof(struct Calculation_obj)+size*sizeof(uint16_t));
	memcpy(instance->data, data, sizeof(uint16_t)*size);
	instance->size = size;
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
	calculation_rms(obj);
	calculation_average(obj);
	calculation_range(obj);
	calculation_amplitude(obj);
	calculation_crest_factor(obj);
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

uint8_t * calculation_calculate_fft(Calculation_obj_handle obj)
{
	complex * input = (complex*) malloc(sizeof(struct complex_t)*obj->size);
	for(uint8_t i = 0; i<obj->size ; ++i){
		input[i].re = (double) obj->data[i];
		input[i].im = (double) 0.0;
	}
	complex * result;
	result = DFT_naive(input, obj->size);
	free(input);
	double divider = 0.0;
	uint16_t fft_size = (obj->size/2)+1;
	for(uint16_t i = 0; i < fft_size; ++i){
		if(divider<fabs(result[i].re)){
			divider = fabs(result[i].re);
		}
	}
	uint8_t * fft_data = (uint8_t *) malloc(sizeof(uint8_t)*fft_size);
	for (uint16_t i = 0; i < fft_size; ++i) {
			fft_data[i] = (uint8_t)floor(200*(fabs(result[i].re)/divider));
			ESP_LOGI("DATA","%d",fft_data[i]);
	}
	free(result);
	return fft_data;
}

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////
static void calculation_rms(Calculation_obj_handle obj)
{
	float sum = 0;
	for (int16_t iterator = (obj->size - 1); iterator >= 0; --iterator) {
		sum += pow((obj->data[iterator]),2.0);
	}
	obj->rms = sqrt(sum/obj->size);
}

/****************************************************************************************/
static void calculation_average(Calculation_obj_handle obj)
{
	uint32_t sum = sum_array(obj->data, obj->size);
	obj->average = sum/obj->size;
}

/****************************************************************************************/
static void calculation_range(Calculation_obj_handle obj)
{
	obj->max_val = obj->data[0];
	obj->min_val = obj->data[0];
	for (int16_t iterator = (obj->size - 1); iterator >= 0; --iterator){
		if(obj->data[iterator] > obj->max_val){
			obj->max_val = obj->data[iterator];
		}else if(obj->data[iterator] < obj->min_val){
			obj->min_val = obj->data[iterator];
		}
	}
}

/****************************************************************************************/
static void calculation_amplitude(Calculation_obj_handle obj)
{
	uint16_t amplitude = 0;
	for (int16_t iterator = (obj->size - 1); iterator >= 0; --iterator){
		if(abs(obj->data[iterator]) > amplitude){
			amplitude = abs(obj->data[iterator]);
		}
	}
	obj->amplitude = amplitude;
}

/****************************************************************************************/
static void calculation_crest_factor(Calculation_obj_handle obj)
{
	obj->crest_factor = obj->amplitude/obj->rms;
}

/****************************************************************************************/
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
