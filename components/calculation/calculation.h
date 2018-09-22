#ifndef COMPONENTS_CALCULATION_CALCULATION_H_
#define COMPONENTS_CALCULATION_CALCULATION_H_
//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////
typedef struct Calculation_obj *Calculation_obj_handle;

typedef enum {
	CALCULATION_RMS = 0,
	CALCULATION_AVERAGE,
	CALCULATION_MAXVAL,
	CALCULATION_MINVAL,
	CALCULATION_AMPLITUDE,
	CALCULATION_CREST_FACTOR
} calculation_factors;

typedef union _calculation_factor_type {
	uint16_t integer_type;
	float float_type;
} calculation_factor_type;

//////////////////////////////////////////////////////////////////////////////////////////
//Global variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
measurement_Init
******************************************************************************************
Parameters:
adc_channel_t channel - chosen adc channel to initialize
adc_atten_t attenuation - chosen attenuation defining the range of adc measurements
adc_bits_width_t width - the resolution of adc measurements
******************************************************************************************
Abstract:
This function initializes chosen adc channel to enable taking measurements with chosen
parameters.
\****************************************************************************************/

Calculation_obj_handle calculation_NewObj(uint16_t data[], uint16_t size);

uint16_t calculation_get_size(Calculation_obj_handle obj);

uint16_t * calculation_get_data_ptr(Calculation_obj_handle obj);

void calculation_calculate_factors(Calculation_obj_handle obj);

calculation_factor_type calculation_get_factor(Calculation_obj_handle obj, calculation_factors factor);

uint8_t * calculation_calculate_fft(Calculation_obj_handle obj);
//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

#endif /* COMPONENTS_CALCULATION_CALCULATION_H_ */

