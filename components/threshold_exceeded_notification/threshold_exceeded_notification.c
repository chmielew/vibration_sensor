/** threshold_exceeded_notification.c **/

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "threshold_exceeded_notification.h"
#include "freertos/FreeRTOS.h"
#include "esp_event_loop.h"

#include "../measurement/measurement.h"
#include "../ble_communication/ble_communication.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
#define ATTEN_11_DB_MULTIPLIER 			((float)3.6)
#define CONVERT_RAW_TO_VOLTAGE(x) 		((float)((x/4095)*ATTEN_11_DB_MULTIPLIER))
#define ACCELEROMETER_ADC_CHANNEL 		(ADC1_CHANNEL_7)

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

/** the calibration val for the threshold evaluation. **/
static uint16_t threshold_exceed_zero_val = 0;

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
reset_max_val
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function resets maximal stored value during monitoring to 0.
\****************************************************************************************/
static void reset_max_val(void);

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////
void threshold_exceeded_init(uint16_t zero_val)
{
	threshold_exceeded_threshold = 0;
	reset_max_val();
	threshold_exceed_zero_val = zero_val;
}

/****************************************************************************************/
void threshold_exceeded_task(void *pvParameter)
{
	while (true) {
		if (0 != threshold_exceeded_threshold) {
			int16_t result = measurement_read(ACCELEROMETER_ADC_CHANNEL);
			if (abs(result-threshold_exceed_zero_val) > threshold_exceeded_threshold) {
				threshold_exceeded_max_val_from_reset = result;
				ble_communication_threshold_exceeded_notification_send(result);
				threshold_exceeded_set_threshold(0);
				vTaskSuspend(NULL);
			} else if (result > threshold_exceeded_max_val_from_reset) {
				threshold_exceeded_max_val_from_reset = result;
			}
			vTaskDelay(1 / portTICK_PERIOD_MS);
		} else {
			vTaskSuspend(NULL);
		}
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
	reset_max_val();
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

