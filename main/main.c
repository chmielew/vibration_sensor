/** main.c **/

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////

/** basic includes */
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

/** application includes */
#include "../components/heartbeat/heartbeat.h"
#include "../components/ble_communication/ble_communication.h"
#include "../components/threshold_exceeded_notification/threshold_exceeded_notification.h"
#include "../components/measurement/measurement.h"
#include "../components/calculation/calculation.h"
#include "task_controller.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
#define ACCELEROMETER_ADC_CHANNEL	(ADC1_CHANNEL_7)

//////////////////////////////////////////////////////////////////////////////////////////
//Local typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
entry_initialization
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function calls the init functions.
\****************************************************************************************/
void entry_initialization(void);

//////////////////////////////////////////////////////////////////////////////////////////
//Static variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

void app_main(){
	/** initialization **/
	entry_initialization();
	entry_task_creator();

	/** local task variables **/
	uint16_t * measurement_ptr = NULL;
	uint16_t no_of_samples = 0;
	Calculation_obj_handle obj = NULL;

	/** main loop **/
	while (true) {
		if (ble_communication_is_threshold_exceed_monitoring_requested()) {
			/** threshold exceeded monitoring control **/
			vTaskSuspend(get_task_handle(THRESHOLD_EXCEEDED_TASK_HANDLE));
			threshold_exceeded_set_threshold(
					ble_communication_get_threshold_exceed_monitoring_val());
			vTaskResume(get_task_handle(THRESHOLD_EXCEEDED_TASK_HANDLE));
			ble_communication_threshold_exceeded_monitoring_handled();
		}

		if (ble_communication_is_measurement_requested()) {
			/** prepare local variables **/
			free(measurement_ptr);
			measurement_ptr = NULL;
			no_of_samples = 0;

			/** measurement trigger **/
			measurement_ptr = measurement_trigger(
					ble_communication_get_requested_measurement_frequency(),
					ble_communication_get_requested_measurement_duration());
			no_of_samples = measurement_get_size();
			obj = calculation_new_obj(measurement_ptr, no_of_samples);
			if (NULL != measurement_ptr && NULL != obj) {
				ble_communication_measurement_request_handled();
			}
		} else if ((MEASUREMENT_FINISHED == measurement_get_status())
				&& (CALCULATION_INITIALIZED == calculation_get_state(obj))) {
			/** calculation trigger **/
			calculation_calculate_factors(obj);
		} else if ((MEASUREMENT_FINISHED == measurement_get_status())
				&& (CALCULATION_FINISHED == calculation_get_state(obj))) {
			/** results update **/
			ble_communication_update_time_measured_data(measurement_ptr,
					no_of_samples);
			ble_communication_update_calculated_value(RMS_VALUE,
					(calculation_get_factor(obj, CALCULATION_RMS).float_type));
			ble_communication_update_calculated_value(AVERAGE_VALUE,
					calculation_get_factor(obj, CALCULATION_AVERAGE).float_type);
			ble_communication_update_calculated_value(MIN_VALUE,
					calculation_get_factor(obj, CALCULATION_MINVAL).float_type);
			ble_communication_update_calculated_value(MAX_VALUE,
					calculation_get_factor(obj, CALCULATION_MAXVAL).float_type);
			ble_communication_update_calculated_value(CREST_FACTOR_VALUE,
					calculation_get_factor(obj, CALCULATION_CREST_FACTOR).float_type);
			ble_communication_update_calculated_value(AMPLITUDE_VALUE,
					calculation_get_factor(obj, CALCULATION_AMPLITUDE).float_type);

			calculation_delete_obj(&obj);
			ble_communication_calculation_completed_notification_send(measurement_get_zero_val());
		}

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
//Static functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

void entry_initialization(void)
{
	nvs_flash_init();
	heartbeat_init();
	ble_communication_init();
	measurement_init(ACCELEROMETER_ADC_CHANNEL, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12);
	threshold_exceeded_init(measurement_get_zero_val());
}

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////
