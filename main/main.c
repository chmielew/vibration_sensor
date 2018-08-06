/** main.c */
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
#include "task_creator.h"
#include "../components/heartbeat/heartbeat.h"
#include "../components/ble_communication/ble_communication.h"
#include "../components/threshold_exceeded_notification/threshold_exceeded_notification.h"
#include "../components/measurement/measurement.h"
#include "../components/calculation/calculation.h"

/** debug includes*/
#include "esp_log.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
#define ACCELEROMETER_ADC_CHANNEL (ADC1_CHANNEL_7)
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

void app_main(void)
{
	entry_initialization();
	entry_task_creator();
	bool flag = 0;
	uint16_t * measurement_ptr = NULL;
    while (1) {
    	if(true == ble_communication_is_measurement_requested()){
    		measurement_ptr = measurement_trigger(11, 2);
    		flag = 1;
    	}

    	if(1 == flag && (MEASUREMENT_FINISHED == measurement_GetStatus())){
    		Calculation_obj_handle obj = calculation_NewObj(measurement_ptr, 20);
    		if(NULL != measurement_ptr){
    		free(measurement_ptr);
    		measurement_ptr = NULL;
    		}
    		calculation_calculate_factors(obj);

    		ESP_LOGI("RMS:","%f", (calculation_get_factor(obj, CALCULATION_RMS).float_type));
    		ESP_LOGI("AVERAGE:","%f", calculation_get_factor(obj, CALCULATION_AVERAGE).float_type);
    		ESP_LOGI("MAXVAL:","%d",calculation_get_factor(obj, CALCULATION_MAXVAL).integer_type);
    		ESP_LOGI("MINVAL:","%d",calculation_get_factor(obj, CALCULATION_MINVAL).integer_type);
    		ESP_LOGI("AMPLITUDE:","%d",calculation_get_factor(obj, CALCULATION_AMPLITUDE).integer_type);
    		ESP_LOGI("CREST FACTOR:","%f",calculation_get_factor(obj, CALCULATION_CREST_FACTOR).float_type);

    		ble_communication_update_calculated_value(RMS_VALUE, (calculation_get_factor(obj, CALCULATION_RMS).float_type));
			ble_communication_update_calculated_value(AVERAGE_VALUE, calculation_get_factor(obj, CALCULATION_AVERAGE).float_type);
			ble_communication_update_calculated_value(MIN_VALUE, calculation_get_factor(obj, CALCULATION_MINVAL).float_type);
			ble_communication_update_calculated_value(MAX_VALUE, calculation_get_factor(obj, CALCULATION_MAXVAL).float_type);
			ble_communication_update_calculated_value(CREST_FACTOR_VALUE, calculation_get_factor(obj, CALCULATION_CREST_FACTOR).float_type);
			ble_communication_update_calculated_value(AMPLITUDE_VALUE, calculation_get_factor(obj, CALCULATION_AMPLITUDE).float_type);
			if(NULL != obj){
			free(obj);
			}
			flag = 0;
    	}
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
	measurement_Init(ACCELEROMETER_ADC_CHANNEL, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12);
//	threshold_exceeded_init(4000);
}

//////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////
