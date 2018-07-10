/*
 * threshold_exceeded_notification.c
 *
 *  Created on: May 15, 2018
 *      Author: chmielew
 */

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "threshold_exceeded_notification.h"
#include "freertos/FreeRTOS.h"
#include "esp_event_loop.h"

#include "../measurement/measurement.h"
#include "../ble_communication/ble_communication.h"

/** debug includes */
//#include "esp_log.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
#define ATTEN_11_DB_MULTIPLIER ((float)3.6)
#define CONVERT_RAW_TO_VOLTAGE(x) ((float)((x/4095)*ATTEN_11_DB_MULTIPLIER))
#define ACCELEROMETER_ADC_CHANNEL (ADC1_CHANNEL_7)

//////////////////////////////////////////////////////////////////////////////////////////
//Local typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Static variables																		//
//////////////////////////////////////////////////////////////////////////////////////////
/** threshold set on the init phase defining when the notification should be sent */
static uint16_t threshold_exceeded_threshold = 0;

/** maximal value of the adc read from the last reset */
static uint16_t threshold_exceeded_max_val_from_reset = 0;

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
static void reset_max_val(void);

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////
void threshold_exceeded_init(uint16_t threshold)
{
	threshold_exceeded_threshold = threshold;
	reset_max_val();
}

/****************************************************************************************/
void threshold_exceeded_task(void *pvParameter)
{
	while(1){
		uint16_t result = measurement_Read(ACCELEROMETER_ADC_CHANNEL);
		if(result > threshold_exceeded_threshold){
			threshold_exceeded_max_val_from_reset = result;
			ble_communication_threshold_exceeded_notification_send(result);
		} else if(result > threshold_exceeded_max_val_from_reset){
			threshold_exceeded_max_val_from_reset = result;
		}
		vTaskDelay(1/portTICK_PERIOD_MS);
	}
}

/****************************************************************************************/
void threshold_exceeded_reset(void)
{
	reset_max_val();
}

/****************************************************************************************/
void threshold_exceeded_set_threshold(uint16_t threshold)
{
	threshold_exceeded_threshold = threshold;
}

/****************************************************************************************/
uint16_t threshold_exceeded_get_max_val_raw(void)
{
	return threshold_exceeded_max_val_from_reset;
}

/****************************************************************************************/
float threshold_exceeded_get_max_val_voltage(void)
{
	return CONVERT_RAW_TO_VOLTAGE(threshold_exceeded_max_val_from_reset);
}

/****************************************************************************************/
uint16_t * threshold_exceeded_get_max_val_pointer(void)
{
	return &threshold_exceeded_max_val_from_reset;
}

/****************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////
static void reset_max_val(void)
{
	threshold_exceeded_max_val_from_reset = 0;
}

/****************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

