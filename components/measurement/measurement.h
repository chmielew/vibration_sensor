/*
 * measurement.h
 *
 *  Created on: 7 mar 2018
 *      Author: chmielew
 *
 *  Component used for handling the analog accelerometer sensor.
 */

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
void measurement_Init(adc1_channel_t channel, adc_atten_t attenuation, adc_bits_width_t width);

/****************************************************************************************\
Function:
measurement_Read
******************************************************************************************
Parameters:
adc_channel_t channel - chosen channel to get measurement
******************************************************************************************
Abstract:
This function returns the value of the measured adc channel.
\****************************************************************************************/
uint16_t measurement_Read(adc1_channel_t channel);

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

#endif /* COMPONENTS_MEASUREMENT_MEASUREMENT_H_ */
