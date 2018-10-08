/**  measurement.h **/

#ifndef COMPONENTS_MEASUREMENT_MEASUREMENT_H_
#define COMPONENTS_MEASUREMENT_MEASUREMENT_H_

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "stdint.h"
#include "driver/adc.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////
typedef enum{
	MEASUREMENT_NOT_INITIALIZED = 0,
	MEASUREMENT_INITIALIZED,
	MEASUREMENT_ACTIVE,
	MEASUREMENT_FINISHED
} measurement_status;

//////////////////////////////////////////////////////////////////////////////////////////
//Global variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
measurement_init
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
void measurement_init(adc1_channel_t channel, adc_atten_t attenuation, adc_bits_width_t width);

/****************************************************************************************\
Function:
measurement_read
******************************************************************************************
Parameters:
adc_channel_t channel - chosen channel to get measurement
******************************************************************************************
Abstract:
This function returns the value of the measured adc channel.
\****************************************************************************************/
uint16_t measurement_read(adc1_channel_t channel);

/****************************************************************************************\
Function:
measurement_trigger
******************************************************************************************
Parameters:
uint16_t frequency - desired frequency
float duration - desired duration
******************************************************************************************
Abstract:
This function triggers measurement with the desired parameters. It configures the timer
to be used for collecting adc conversion results into the buffer. It returns pointer to
the stored measured values.
\****************************************************************************************/
uint16_t * measurement_trigger(uint16_t frequency, float duration);

/****************************************************************************************\
Function:
measurement_get_status
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function returns the status of the active measurement.
\****************************************************************************************/
measurement_status measurement_get_status(void);

/****************************************************************************************\
Function:
measurement_get_pointer
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This funtion returns a pointer for the current measurement data.
\****************************************************************************************/
uint16_t * measurement_get_pointer(void);

/****************************************************************************************\
Function:
measurement_get_size
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function returns size of current measurement data.
\****************************************************************************************/
uint32_t measurement_get_size(void);

/****************************************************************************************\
Function:
measurement_get_zero_val
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function returns zero value of the sensor from init phase.
\****************************************************************************************/
uint16_t measurement_get_zero_val(void);

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////
#endif /* COMPONENTS_MEASUREMENT_MEASUREMENT_H_ */
