/** ble_communication.c **/

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "ble_communication.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <string.h>
#include "../threshold_exceeded_notification/threshold_exceeded_notification.h"

/** bluetooth specific includes */
#include "bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
#define DEVICE_NAME		"ESP_VIBRATION_SENSOR"

/** (GAP) advertising configuration macros */
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

/** (GATTS) macros */
#define INITIALIZE_UUID_TABLE(uuid)	{0x00, 0x00, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
	   	0x00, 0x10, 0x00, 0x00, (uint8_t)(uuid>>0), (uint8_t)(uuid>>8), 0x00, 0x00}

#define PROFILE_NUM 5

/** profile_threshold_exceeded_notification parameters */
#define PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION 0
#define GATTS_SERVICE_UUID_THRESHOLD_EXCEEDED_NOTIFICATION	((uint16_t)0x0100)
#define GATTS_CHAR_UUID_THRESHOLD_EXCEEDED_NOTIFICATION		((uint16_t) \
				(GATTS_SERVICE_UUID_THRESHOLD_EXCEEDED_NOTIFICATION + 0x0001))
#define THRESHOLD_EXCEEDED_WRITE_VAL						(0x01)

/** profile_get_calculated_values parameters */
#define PROFILE_GET_CALCULATED_VALUES 1
#define GATTS_SERVICE_UUID_GET_CALCULATED_VALUES	((uint16_t)0x0200)
#define GATTS_CHAR_UUID_GET_RMS_VALUE				((uint16_t) \
				(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0001))
#define GATTS_CHAR_UUID_GET_AVERAGE_VALUE			((uint16_t) \
				(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0002))
#define GATTS_CHAR_UUID_GET_MAX_VALUE				((uint16_t) \
				(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0003))
#define GATTS_CHAR_UUID_GET_MIN_VALUE				((uint16_t) \
				(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0004))
#define GATTS_CHAR_UUID_GET_AMPLITUDE_VALUE			((uint16_t) \
				(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0005))
#define GATTS_CHAR_UUID_GET_CREST_FACTOR_VALUE		((uint16_t) \
				(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0006))

/** profile_trigger_measurement */
#define PROFILE_TRIGGER_MEASUREMENT 2
#define GATTS_SERVICE_UUID_TRIGGER_MEASUREMENT 	((uint16_t)0x0300)
#define GATTS_CHAR_UUID_TRIGGER_MEASUREMENT		((uint16_t) \
				(GATTS_SERVICE_UUID_TRIGGER_MEASUREMENT+0x0001))
#define MEASUREMENT_TRIGGER_WRITE_VAL			(0x01)

/** profile_get_time_results */
#define PROFILE_GET_TIME_RESULTS 3
#define GATTS_SERVICE_UUID_GET_TIME_RESULTS ((uint16_t)0x0400)
#define GATTS_CHAR_UUID_GET_TIME_RESULTS	((uint16_t) \
				(GATTS_SERVICE_UUID_GET_TIME_RESULTS+0x0001))

/** profile_get_fft_results */
#define PROFILE_GET_FFT_RESULTS 4
#define GATTS_SERVICE_UUID_GET_FFT_RESULTS	 	((uint16_t)0x0500)
#define GATTS_CHAR_UUID_GET_FFT_RESULTS			((uint16_t) \
				(GATTS_SERVICE_UUID_GET_FFT_RESULTS+0x0001))

#define GATTS_CHAR_VAL_LEN_MAX 0x40

/** device BLE TAG */
#define GATTS_TAG "VIBRATION SENSOR"

//////////////////////////////////////////////////////////////////////////////////////////
//Local typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
gap_event_handler
******************************************************************************************
Parameters:
esp_gap_ble_cb_event_t event - gap callback event type
esp_ble_gap_cb_param_t *param - gap callback parameters union
******************************************************************************************
Abstract:
This is a callback handler for gap event.
\****************************************************************************************/
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/****************************************************************************************\
Function:
gatts_event_handler
******************************************************************************************
Parameters:
esp_gatts_cb_event_t event - gatt callback event type
esp_gatt_if_t gatts_if - gatt interface type
esp_ble_gatts_cb_param_t *param - gatt server callback parameters union
******************************************************************************************
Abstract:
This is a callback handler for gatts event.
\****************************************************************************************/
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
			   	esp_ble_gatts_cb_param_t *param);

/****************************************************************************************\
Function:
gatts_profile_threshold_exceeded_notification
******************************************************************************************
Parameters:
esp_gatts_cb_event_t event - gatt callback event type
esp_gatt_if_t gatts_if - gatt interface type
esp_ble_gatts_cb_param_t *param - gatt server callback parameters union
******************************************************************************************
Abstract:
Implementation of threshold exceeded profile.
\****************************************************************************************/
static void gatts_profile_threshold_exceeded_notification(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/****************************************************************************************\
Function:
gatts_profile_get_calculated_values
******************************************************************************************
Parameters:
esp_gatts_cb_event_t event - gatt callback event type
esp_gatt_if_t gatts_if - gatt interface type
esp_ble_gatts_cb_param_t *param - gatt server callback parameters union
******************************************************************************************
Abstract:
Implementation of calculated values profile.
\****************************************************************************************/
static void gatts_profile_get_calculated_values(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/****************************************************************************************\
Function:
gatts_profile_trigger_measurement
******************************************************************************************
Parameters:
esp_gatts_cb_event_t event - gatt callback event type
esp_gatt_if_t gatts_if - gatt interface type
esp_ble_gatts_cb_param_t *param - gatt server callback parameters union
******************************************************************************************
Abstract:
Implementation of measurement trigger profile.
\****************************************************************************************/
static void gatts_profile_trigger_measurement(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/****************************************************************************************\
Function:
gatts_profile_get_time_results
******************************************************************************************
Parameters:
esp_gatts_cb_event_t event - gatt callback event type
esp_gatt_if_t gatts_if - gatt interface type
esp_ble_gatts_cb_param_t *param - gatt server callback parameters union
******************************************************************************************
Abstract:
Implementation of time results profile.
\****************************************************************************************/
static void gatts_profile_get_time_results(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/****************************************************************************************\
Function:
gatts_profile_get_fft_results
******************************************************************************************
Parameters:
esp_gatts_cb_event_t event - gatt callback event type
esp_gatt_if_t gatts_if - gatt interface type
esp_ble_gatts_cb_param_t *param - gatt server callback parameters union
******************************************************************************************
Abstract:
Implementation of fft restults profile.
\****************************************************************************************/
static void gatts_profile_get_fft_results(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/****************************************************************************************\
Function:
set_uuid
******************************************************************************************
Parameters:
uint16_t new_uuid_16bit - uuid to be set value
uint8_t uuid_tab[]- tab where the uuid is to be set
******************************************************************************************
Abstract:
This function sets uuid for a profile.
\****************************************************************************************/
static void set_uuid(uint16_t new_uuid_16bit, uint8_t uuid_tab[]);

/****************************************************************************************\
Function:
initialize_calculated_values_attr_vals_array
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
Funtion used for initialization for multiple attribute values array.
\****************************************************************************************/
static void initialize_calculated_values_attr_vals_array(void);

/****************************************************************************************\
Function:
reset_measurement_request_struct
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function resets measurement request.
\****************************************************************************************/
static void reset_measurement_request_struct(void);

/****************************************************************************************\
Function:
reset_time_measured_struct
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function resets time_measured_struct data.
\****************************************************************************************/
static void reset_time_measured_struct(void);

/****************************************************************************************\
Function:
reset_fft_data_struct
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function resets fft data struct.
\****************************************************************************************/
static void reset_fft_data_struct(void);

/****************************************************************************************\
Function:
reset_threshold_exceed_monitoring_val
******************************************************************************************
Parameters:
None.
******************************************************************************************
Abstract:
This function resets static variable containing threshold exceed montoring val.
\****************************************************************************************/
static void reset_threshold_exceed_monitoring_val(void);

/****************************************************************************************\
Function:
set_threshold_exceed_monitoring_val
******************************************************************************************
Parameters:
uint16_t threshold
******************************************************************************************
Abstract:
This function sets static variable containing threshold exceed montoring val.
\****************************************************************************************/
static void set_threshold_exceed_monitoring_val(uint16_t threshold);

//////////////////////////////////////////////////////////////////////////////////////////
//Static variables																		//
//////////////////////////////////////////////////////////////////////////////////////////

/** ADVERTISING LOCAL VARIABLES */

/** flag defining the state of the advertising configuration */
static uint8_t adv_config_done = 0;

/** advertising parameters structure */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min		= 0x20,
    .adv_int_max		= 0x40,
    .adv_type			= ADV_TYPE_IND,
    .own_addr_type		= BLE_ADDR_TYPE_PUBLIC,
    .peer_addr			= {0},
    .peer_addr_type		= 0,
    .channel_map		= ADV_CHNL_ALL,
    .adv_filter_policy 	= ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/** advertising service uuids **/
static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 
	0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 
	0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/** advertising data */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/** scam response data */
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/** GATTS LOCAL VARIABLES */

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, 
 * this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
	[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION] = {
			.gatts_cb = gatts_profile_threshold_exceeded_notification,
			.gatts_if = ESP_GATT_IF_NONE,
	},
	[PROFILE_GET_CALCULATED_VALUES] = {
			.gatts_cb = gatts_profile_get_calculated_values,
			.gatts_if = ESP_GATT_IF_NONE,
	},
	[PROFILE_TRIGGER_MEASUREMENT] = {
			.gatts_cb = gatts_profile_trigger_measurement,
			.gatts_if = ESP_GATT_IF_NONE,
	},
	[PROFILE_GET_TIME_RESULTS] = {
				.gatts_cb = gatts_profile_get_time_results,
				.gatts_if = ESP_GATT_IF_NONE,
	},
	[PROFILE_GET_FFT_RESULTS] = {
					.gatts_cb = gatts_profile_get_fft_results,
					.gatts_if = ESP_GATT_IF_NONE,
		}
};

/** static array containing characteristics uuids for calculated values profile **/
static const uint16_t calculated_values_char_uuid[MAX_CALCULATED_VALUES] =
{
	GATTS_CHAR_UUID_GET_RMS_VALUE,
	GATTS_CHAR_UUID_GET_AVERAGE_VALUE,
	GATTS_CHAR_UUID_GET_MAX_VALUE,
	GATTS_CHAR_UUID_GET_MIN_VALUE,
	GATTS_CHAR_UUID_GET_AMPLITUDE_VALUE,
	GATTS_CHAR_UUID_GET_CREST_FACTOR_VALUE
};

/** array conatining calc values attributes vals **/
static esp_attr_value_t calculated_values_attr_val_tab[MAX_CALCULATED_VALUES];

/** array containing current values of calculated indicators **/
static calculated_val_rsp calculated_vals_response_tab[MAX_CALCULATED_VALUES];

/** structure containing data about measurement trigger request **/
static struct _measurement_trigger_request{
	uint16_t frequency;
	float duration;
	bool is_requested;
} measurement_trigger_request;

/** structure contating time measured data **/
static struct _time_measured_data{
	uint16_t size;
	uint16_t *data;
	uint16_t current_pos;
} time_measured_data;

/** structure contating fft data **/
static struct _fft_data{
	uint16_t size;
	uint8_t *data;
	uint16_t current_pos;
} fft_data;

/** static var containing threshold monitoring exceed val **/
static uint16_t threshold_exceed_monitoring_val = 0;

/** configuration struct needed to create characteristics **/
static uint8_t char_str[] = {0x11};

esp_attr_value_t gatts_char_val =
{
    .attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char_str),
    .attr_value   = char_str,
};

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////
void ble_communication_init()
{
	reset_measurement_request_struct();
	reset_time_measured_struct();
	reset_fft_data_struct();
	reset_threshold_exceed_monitoring_val();
	/** BT controller initialization */
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);

	esp_bt_controller_enable(ESP_BT_MODE_BLE);

	/** BT stack initialization */
	esp_bluedroid_init();
	esp_bluedroid_enable();

	/** register GAP and GATTS callbacks */
	esp_ble_gap_register_callback(gap_event_handler);
	esp_ble_gatts_register_callback(gatts_event_handler);
	/** set device name */
	esp_ble_gap_set_device_name(DEVICE_NAME);
	/** configure advertising */
    //config adv data
    esp_ble_gap_config_adv_data(&adv_data);
    adv_config_done |= ADV_CONFIG_FLAG;
    //config scan response data
    esp_ble_gap_config_adv_data(&scan_rsp_data);
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;

    /** register app profiles */
	esp_ble_gatts_app_register(PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION);
	esp_ble_gatts_app_register(PROFILE_GET_CALCULATED_VALUES);
	esp_ble_gatts_app_register(PROFILE_TRIGGER_MEASUREMENT);
	esp_ble_gatts_app_register(PROFILE_GET_TIME_RESULTS);
	esp_ble_gatts_app_register(PROFILE_GET_FFT_RESULTS);

	/** set mtu */
	esp_ble_gatt_set_local_mtu(500);
}
/****************************************************************************************/

void ble_communication_threshold_exceeded_notification_send(uint16_t exceeded_value)
{
	uint8_t val[2] = {(exceeded_value>>8)&0xff, (exceeded_value)&0xff};
	esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].gatts_if,
				   	gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].conn_id, 
					gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_handle, 
					sizeof(val), val, false);
}
/****************************************************************************************/

void ble_communication_update_calculated_value(calculated_value type, float val)
{
	calculated_vals_response_tab[type].float_type = val;
}
/****************************************************************************************/

bool ble_communication_is_measurement_requested(void)
{
	return measurement_trigger_request.is_requested;
}
/****************************************************************************************/

float ble_communication_get_requested_measurement_duration(void)
{
	return measurement_trigger_request.duration;
}
/****************************************************************************************/

uint16_t ble_communication_get_requested_measurement_frequency(void)
{
	return measurement_trigger_request.frequency;
}
/****************************************************************************************/

void ble_communication_measurement_request_handled(void)
{
	reset_measurement_request_struct();
}
/****************************************************************************************/

void ble_communication_update_time_measured_data(uint16_t *data, uint16_t size)
{
	time_measured_data.current_pos = 0;
	time_measured_data.data = data;
	time_measured_data.size = size;
}
/****************************************************************************************/

void ble_communication_update_fft_data(uint8_t *data, uint16_t size)
{
	fft_data.current_pos = 0;
	fft_data.data = data;
	fft_data.size = size;
}
/****************************************************************************************/

void ble_communication_calculation_completed_notification_send(uint16_t zero_val_offset)
{
	uint8_t val[2] = {(zero_val_offset>>8)&0xff, (zero_val_offset)&0xff};
	esp_ble_gatts_send_indicate(
			gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].gatts_if,
			gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].conn_id,
			gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].char_handle,
			sizeof(val), val, false);
}
/****************************************************************************************/

bool ble_communication_is_threshold_exceed_monitoring_requested()
{
	return (threshold_exceed_monitoring_val ? true : false);
}
/****************************************************************************************/

void ble_communication_threshold_exceeded_monitoring_handled()
{
	reset_threshold_exceed_monitoring_val();
}
/****************************************************************************************/

uint16_t ble_communication_get_threshold_exceed_monitoring_val(void)
{
	return threshold_exceed_monitoring_val;
}

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        }
        else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        break;
    default:
        break;
    }
}
/****************************************************************************************/

static void gatts_event_handler(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
     /* here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || 
							/* ESP_GATT_IF_NONE,  not specify a certain gatt_if, 
							 * need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}
/****************************************************************************************/

static void gatts_profile_threshold_exceeded_notification(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.is_primary = true;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.id.uuid.len = ESP_UUID_LEN_128;
		set_uuid(GATTS_SERVICE_UUID_THRESHOLD_EXCEEDED_NOTIFICATION, 
				gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.id.uuid.uuid.uuid128);
		esp_ble_gap_set_device_name(DEVICE_NAME);
		esp_ble_gap_config_adv_data(&adv_data);
		esp_ble_gap_config_adv_data(&scan_rsp_data);
		esp_ble_gatts_create_service(gatts_if,
					   	&gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id, 0x2e);
		break;
	case ESP_GATTS_READ_EVT:
		break;
	case ESP_GATTS_WRITE_EVT:
		if (param->write.value[1] == THRESHOLD_EXCEEDED_WRITE_VAL) {
			uint16_t threshold = param->write.value[2] << 8
					| param->write.value[3];
			set_threshold_exceed_monitoring_val(threshold);
		}
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		break;
	case ESP_GATTS_MTU_EVT:
		break;
	case ESP_GATTS_CONF_EVT:
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle =
			   	param->create.service_handle;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_uuid.len =
			   	ESP_UUID_LEN_128;
		set_uuid(GATTS_CHAR_UUID_THRESHOLD_EXCEEDED_NOTIFICATION, 
						gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_uuid.uuid.uuid128);

		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle);
		esp_ble_gatts_add_char(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle, 
						&gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_uuid,
		ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE | 
		ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &gatts_char_val, NULL);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:;
		uint16_t length = 0;
		const uint8_t *prf_char;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].descr_uuid.uuid.uuid16 = 
				ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
		esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle, 
						&gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].descr_uuid, 
						ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT:
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		break;
	case ESP_GATTS_OPEN_EVT:
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		break;
	case ESP_GATTS_CLOSE_EVT:
		break;
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		break;
	case ESP_GATTS_RESPONSE_EVT:
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		break;
	case ESP_GATTS_SET_ATTR_VAL_EVT:
		break;
	}
}
/****************************************************************************************/

static void gatts_profile_get_calculated_values(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_id.is_primary = true;
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_id.id.uuid.len = ESP_UUID_LEN_128;
		set_uuid(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES,
				gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_id.id.uuid.uuid.uuid128);
		esp_ble_gap_set_device_name(DEVICE_NAME);
		esp_ble_gap_config_adv_data(&adv_data);
		esp_ble_gap_config_adv_data(&scan_rsp_data);
		esp_ble_gatts_create_service(gatts_if,
				&gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_id, 0x2e);

		break;
	case ESP_GATTS_READ_EVT:{
#define RMS_VALUE_HANDLE			0x58
#define AVERAGE_VALUE_HANDLE		0x5a
#define MAX_VALUE_HANDLE			0x5c
#define MIN_VALUE_HANDLE			0x5e
#define AMPLITUDE_VALUE_HANDLE		0x60
#define CREST_FACTOR_VALUE_HANDLE	0x62
		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = sizeof(float);
		switch (param->read.handle) {
		case RMS_VALUE_HANDLE:
			memcpy(rsp.attr_value.value, calculated_vals_response_tab[RMS_VALUE].int_type,
					rsp.attr_value.len);
			break;
		case AVERAGE_VALUE_HANDLE:
			memcpy(rsp.attr_value.value, calculated_vals_response_tab[AVERAGE_VALUE].int_type,
								rsp.attr_value.len);
			break;
		case MAX_VALUE_HANDLE:
			memcpy(rsp.attr_value.value, calculated_vals_response_tab[MAX_VALUE].int_type,
								rsp.attr_value.len);
			break;
		case MIN_VALUE_HANDLE:
			memcpy(rsp.attr_value.value, calculated_vals_response_tab[MIN_VALUE].int_type,
								rsp.attr_value.len);
			break;
		case AMPLITUDE_VALUE_HANDLE:
			memcpy(rsp.attr_value.value, calculated_vals_response_tab[AMPLITUDE_VALUE].int_type,
								rsp.attr_value.len);
			break;
		case CREST_FACTOR_VALUE_HANDLE:
			memcpy(rsp.attr_value.value, calculated_vals_response_tab[CREST_FACTOR_VALUE].int_type,
								rsp.attr_value.len);
			break;
		}
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
				ESP_GATT_OK, &rsp);
		break;
	}
	case ESP_GATTS_WRITE_EVT:
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		break;
	case ESP_GATTS_MTU_EVT:
		break;
	case ESP_GATTS_CONF_EVT:
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		initialize_calculated_values_attr_vals_array();

		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_handle = param->create.service_handle;
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].char_uuid.len = ESP_UUID_LEN_128;

		/* Add all the characteristics */
		esp_attr_control_t response_config = {.auto_rsp = ESP_GATT_RSP_BY_APP};
		for(uint8_t i = 0; i<MAX_CALCULATED_VALUES; ++i){
			set_uuid(calculated_values_char_uuid[i],
					gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].char_uuid.uuid.uuid128);
			esp_ble_gatts_add_char(
					gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_handle,
					&gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].char_uuid,
					ESP_GATT_PERM_READ,
					ESP_GATT_CHAR_PROP_BIT_READ, &(calculated_values_attr_val_tab[i]),
					&response_config);
		}
		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_handle);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:;
		uint16_t length = 0;
		const uint8_t *prf_char;
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].descr_uuid.uuid.uuid16 =
				ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
		esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_handle,
				&gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].descr_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT:
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		break;
	case ESP_GATTS_OPEN_EVT:
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		break;
	case ESP_GATTS_CLOSE_EVT:
		break;
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		break;
	case ESP_GATTS_RESPONSE_EVT:
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		break;
	case ESP_GATTS_SET_ATTR_VAL_EVT:
		break;
	}
}
/****************************************************************************************/

static void gatts_profile_trigger_measurement(esp_gatts_cb_event_t event, 
				esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_id.is_primary = true;
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_id.id.uuid.len = ESP_UUID_LEN_128;
		set_uuid(GATTS_SERVICE_UUID_TRIGGER_MEASUREMENT,
				gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_id.id.uuid.uuid.uuid128);
		esp_ble_gap_set_device_name(DEVICE_NAME);
		esp_ble_gap_config_adv_data(&adv_data);
		esp_ble_gap_config_adv_data(&scan_rsp_data);
		esp_ble_gatts_create_service(gatts_if,
				&gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_id, 0x2e);

		break;
	case ESP_GATTS_READ_EVT:
		break;
	case ESP_GATTS_WRITE_EVT:{
		/* for some reason 0 element is always 0 no matter what is written,
		 * also when char-write-cmd in gatttool the value has to be 0x01 not 0x1
		 * */
		if(param->write.value[1] == MEASUREMENT_TRIGGER_WRITE_VAL){
			/*trigger measurement */
			measurement_trigger_request.is_requested = true;
			measurement_trigger_request.frequency = param->write.value[2]<<8 | param->write.value[3];
			uint32_t *ptr = (uint32_t*)&(measurement_trigger_request.duration);
			*ptr = (param->write.value[4]<<24 | param->write.value[5]<<16 |
						   	param->write.value[6]<<8 | param->write.value[7]);
		}
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		break;
	case ESP_GATTS_MTU_EVT:
		break;
	case ESP_GATTS_CONF_EVT:
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		initialize_calculated_values_attr_vals_array();

		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_handle = param->create.service_handle;
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].char_uuid.len = ESP_UUID_LEN_128;

		/* Add all the characteristics */
		esp_attr_control_t response_config = {.auto_rsp = ESP_GATT_RSP_BY_APP};
			set_uuid(GATTS_CHAR_UUID_TRIGGER_MEASUREMENT,
					gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].char_uuid.uuid.uuid128);
			esp_ble_gatts_add_char(
					gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_handle,
					&gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].char_uuid,
					ESP_GATT_PERM_WRITE,
					ESP_GATT_CHAR_PROP_BIT_WRITE  | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &(gatts_char_val),
					&response_config);
		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_handle);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:;
		uint16_t length = 0;
		const uint8_t *prf_char;
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].descr_uuid.uuid.uuid16 =
				ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
		esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_handle,
				&gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].descr_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT:
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		break;
	case ESP_GATTS_OPEN_EVT:
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		break;
	case ESP_GATTS_CLOSE_EVT:
		break;
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		break;
	case ESP_GATTS_RESPONSE_EVT:
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		break;
	case ESP_GATTS_SET_ATTR_VAL_EVT:
		break;
	}
}
/****************************************************************************************/

static void gatts_profile_get_time_results(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
			   	esp_ble_gatts_cb_param_t *param)
{
	switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_id.is_primary = true;
		gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_id.id.uuid.len = ESP_UUID_LEN_128;
		set_uuid(GATTS_SERVICE_UUID_GET_TIME_RESULTS,
				gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_id.id.uuid.uuid.uuid128);
		esp_ble_gap_set_device_name(DEVICE_NAME);
		esp_ble_gap_config_adv_data(&adv_data);
		esp_ble_gap_config_adv_data(&scan_rsp_data);
		esp_ble_gatts_create_service(gatts_if,
				&gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_id, 0x2e);

		break;
	case ESP_GATTS_READ_EVT:{
#define FIRST_FRAME_IDN		0x1
#define MORE_DATA_IDN 		0x2
#define NO_MORE_DATA_IDN	0x3
#define FRAME_SIZE	21
		esp_gatt_rsp_t rsp;
		if(NULL != time_measured_data.data){
			if(time_measured_data.current_pos >= (time_measured_data.size-((time_measured_data.size)%10)-1)){
				memset(rsp.attr_value.value, 0xff, FRAME_SIZE);
				rsp.attr_value.value[0] = NO_MORE_DATA_IDN;
				memcpy(rsp.attr_value.value+1, time_measured_data.data+(time_measured_data.current_pos),
							   	2*(time_measured_data.size - time_measured_data.current_pos));
				time_measured_data.current_pos = 0;
			} else if (0 == time_measured_data.current_pos) {
				rsp.attr_value.value[0] = FIRST_FRAME_IDN;
				memcpy(rsp.attr_value.value+1, time_measured_data.data, 20);
				time_measured_data.current_pos += 10;
			} else {
				rsp.attr_value.value[0] = MORE_DATA_IDN;
				memcpy(rsp.attr_value.value+1, time_measured_data.data+(time_measured_data.current_pos), 20);
				time_measured_data.current_pos += 10;
			}
		} else {
			memset(rsp.attr_value.value, 0xff, FRAME_SIZE);
			rsp.attr_value.value[0] = NO_MORE_DATA_IDN;
		}
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = FRAME_SIZE;

		esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
				param->read.trans_id, ESP_GATT_OK, &rsp);
		break;
	}
	case ESP_GATTS_WRITE_EVT:
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		break;
	case ESP_GATTS_MTU_EVT:
		break;
	case ESP_GATTS_CONF_EVT:
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:

		gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_handle = param->create.service_handle;
		gl_profile_tab[PROFILE_GET_TIME_RESULTS].char_uuid.len = ESP_UUID_LEN_128;

		/* Add all the characteristics */
		esp_attr_control_t response_config = {.auto_rsp = ESP_GATT_RSP_BY_APP};
			set_uuid(GATTS_CHAR_UUID_GET_TIME_RESULTS,
					gl_profile_tab[PROFILE_GET_TIME_RESULTS].char_uuid.uuid.uuid128);
			esp_ble_gatts_add_char(
					gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_handle,
					&gl_profile_tab[PROFILE_GET_TIME_RESULTS].char_uuid,
					ESP_GATT_PERM_READ,
					ESP_GATT_CHAR_PROP_BIT_READ, &gatts_char_val,
					&response_config);
		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_handle);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:;
		uint16_t length = 0;
		const uint8_t *prf_char;
		gl_profile_tab[PROFILE_GET_TIME_RESULTS].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_GET_TIME_RESULTS].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_GET_TIME_RESULTS].descr_uuid.uuid.uuid16 =
				ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
		esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_GET_TIME_RESULTS].service_handle,
				&gl_profile_tab[PROFILE_GET_TIME_RESULTS].descr_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT:
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		break;
	case ESP_GATTS_OPEN_EVT:
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		break;
	case ESP_GATTS_CLOSE_EVT:
		break;
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		break;
	case ESP_GATTS_RESPONSE_EVT:
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		break;
	case ESP_GATTS_SET_ATTR_VAL_EVT:
		break;
	}
}
/****************************************************************************************/

static void gatts_profile_get_fft_results(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
			   	esp_ble_gatts_cb_param_t *param)
{
	switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_id.is_primary = true;
		gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_id.id.uuid.len = ESP_UUID_LEN_128;
		set_uuid(GATTS_SERVICE_UUID_GET_FFT_RESULTS,
				gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_id.id.uuid.uuid.uuid128);
		esp_ble_gap_set_device_name(DEVICE_NAME);
		esp_ble_gap_config_adv_data(&adv_data);
		esp_ble_gap_config_adv_data(&scan_rsp_data);
		esp_ble_gatts_create_service(gatts_if,
				&gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_id, 0x2e);
		break;
	case ESP_GATTS_READ_EVT:{
#define FIRST_FRAME_IDN		0x1
#define MORE_DATA_IDN 		0x2
#define NO_MORE_DATA_IDN	0x3
#define FRAME_SIZE	21
		esp_gatt_rsp_t rsp;
		if(NULL != fft_data.data){
			if(fft_data.current_pos >= (fft_data.size-((fft_data.size)%20)-1)){
				memset(rsp.attr_value.value, 0xff, FRAME_SIZE);
				rsp.attr_value.value[0] = NO_MORE_DATA_IDN;
				memcpy(rsp.attr_value.value+1, fft_data.data+(fft_data.current_pos), 
								(fft_data.size - fft_data.current_pos));
				fft_data.current_pos = 0;
			} else if (0 == fft_data.current_pos) {
				rsp.attr_value.value[0] = FIRST_FRAME_IDN;
				memcpy(rsp.attr_value.value+1, fft_data.data, 20);
				fft_data.current_pos += 20;
			} else {
				rsp.attr_value.value[0] = MORE_DATA_IDN;
				memcpy(rsp.attr_value.value+1, fft_data.data+(fft_data.current_pos), 20);
				fft_data.current_pos += 20;
			}
		} else {
			memset(rsp.attr_value.value, 0xff, FRAME_SIZE);
			rsp.attr_value.value[0] = NO_MORE_DATA_IDN;
		}
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = FRAME_SIZE;
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
				param->read.trans_id, ESP_GATT_OK, &rsp);
		break;
	}
	case ESP_GATTS_WRITE_EVT:
		break;
	case ESP_GATTS_EXEC_WRITE_EVT:
		break;
	case ESP_GATTS_MTU_EVT:
		break;
	case ESP_GATTS_CONF_EVT:
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:

		gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_handle = param->create.service_handle;
		gl_profile_tab[PROFILE_GET_FFT_RESULTS].char_uuid.len = ESP_UUID_LEN_128;

		/* Add all the characteristics */
		esp_attr_control_t response_config = {.auto_rsp = ESP_GATT_RSP_BY_APP};
			set_uuid(GATTS_CHAR_UUID_GET_FFT_RESULTS,
					gl_profile_tab[PROFILE_GET_FFT_RESULTS].char_uuid.uuid.uuid128);
			esp_ble_gatts_add_char(
					gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_handle,
					&gl_profile_tab[PROFILE_GET_FFT_RESULTS].char_uuid,
					ESP_GATT_PERM_READ,
					ESP_GATT_CHAR_PROP_BIT_READ, &gatts_char_val,
					&response_config);
		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_handle);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:;
		uint16_t length = 0;
		const uint8_t *prf_char;
		gl_profile_tab[PROFILE_GET_FFT_RESULTS].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_GET_FFT_RESULTS].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_GET_FFT_RESULTS].descr_uuid.uuid.uuid16 =
				ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
		esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_GET_FFT_RESULTS].service_handle,
				&gl_profile_tab[PROFILE_GET_FFT_RESULTS].descr_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT:
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		break;
	case ESP_GATTS_OPEN_EVT:
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		break;
	case ESP_GATTS_CLOSE_EVT:
		break;
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		break;
	case ESP_GATTS_RESPONSE_EVT:
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		break;
	case ESP_GATTS_SET_ATTR_VAL_EVT:
		break;
	}
}
/****************************************************************************************/

static void set_uuid(uint16_t new_uuid_16bit, uint8_t uuid_tab[])
{
	uint8_t new_uuid_tab[16] = INITIALIZE_UUID_TABLE(new_uuid_16bit);
	memcpy(uuid_tab, new_uuid_tab, ESP_UUID_LEN_128);
}
/****************************************************************************************/

static void initialize_calculated_values_attr_vals_array(void)
{
	for(uint8_t i = 0; i<MAX_CALCULATED_VALUES; ++i){
		calculated_values_attr_val_tab[i].attr_len = sizeof(float);
		calculated_values_attr_val_tab[i].attr_max_len = GATTS_CHAR_VAL_LEN_MAX;
		calculated_values_attr_val_tab[i].attr_value = 
				calculated_vals_response_tab[i].int_type;
	}
}
/****************************************************************************************/

static void reset_measurement_request_struct(void)
{
	measurement_trigger_request.duration = 0;
	measurement_trigger_request.frequency = 0;
	measurement_trigger_request.is_requested = false;
}
/****************************************************************************************/

static void reset_time_measured_struct(void)
{
	time_measured_data.current_pos = 0;
	time_measured_data.data = NULL;
	time_measured_data.size = 0;
}
/****************************************************************************************/

static void reset_fft_data_struct(void)
{
	fft_data.current_pos = 0;
	fft_data.data = NULL;
	fft_data.size = 0;
}
/****************************************************************************************/

static void reset_threshold_exceed_monitoring_val(void)
{
	threshold_exceed_monitoring_val = 0;
}
/****************************************************************************************/

static void set_threshold_exceed_monitoring_val(uint16_t threshold)
{
	threshold_exceed_monitoring_val = threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

