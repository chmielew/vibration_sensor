/*
 * ble_communication.c
 *
 *  Created on: 18 mar 2018
 *      Author: chmielew
 */


//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "ble_communication.h"
#include "freertos/FreeRTOS.h"

/** bluetooth specific includes */
#include "bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

/** debug includes */
#include "esp_log.h"
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
#define DEVICE_NAME		"ESP_VIBRATION_SENSOR"

/** (GAP) advertising configuration macros */
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

/** (GATTS) macros */
#define INITIALIZE_UUID_TABLE(uuid)	{0x00, 0x00, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, (uint8_t)(uuid>>0), (uint8_t)(uuid>>8), 0x00, 0x00}

#define PROFILE_NUM 3

/** profile_threshold_exceeded_notification parameters */
#define PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION 0
#define GATTS_SERVICE_UUID_THRESHOLD_EXCEEDED_NOTIFICATION	((uint16_t)0x0100)
#define GATTS_CHAR_UUID_THRESHOLD_EXCEEDED_NOTIFICATION		((uint16_t)(GATTS_SERVICE_UUID_THRESHOLD_EXCEEDED_NOTIFICATION + 0x0001))

/** profile_get_calculated_values parameters */
#define PROFILE_GET_CALCULATED_VALUES 1
#define GATTS_SERVICE_UUID_GET_CALCULATED_VALUES	((uint16_t)0x0200)
#define GATTS_CHAR_UUID_GET_RMS_VALUE				((uint16_t)(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0001))
#define GATTS_CHAR_UUID_GET_AVERAGE_VALUE			((uint16_t)(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0002))
#define GATTS_CHAR_UUID_GET_MAX_VALUE				((uint16_t)(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0003))
#define GATTS_CHAR_UUID_GET_MIN_VALUE				((uint16_t)(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0004))
#define GATTS_CHAR_UUID_GET_AMPLITUDE_VALUE			((uint16_t)(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0005))
#define GATTS_CHAR_UUID_GET_CREST_FACTOR_VALUE		((uint16_t)(GATTS_SERVICE_UUID_GET_CALCULATED_VALUES + 0x0006))

/** profile_trigger_measurement */
#define PROFILE_TRIGGER_MEASUREMENT 2
#define GATTS_SERVICE_UUID_TRIGGER_MEASUREMENT 	((uint16_t)0x0300)
#define GATTS_CHAR_UUID_TRIGGER_MEASUREMENT		((uint16_t)(GATTS_SERVICE_UUID_TRIGGER_MEASUREMENT+0x0001))
#define MEASUREMENT_TRIGGER_WRITE_VAL			(0x01)

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
//TODO:update headers
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
gap_event_handler
******************************************************************************************
Parameters:
esp_gap_ble_cb_event_t event - gap callback event type
esp_ble_gap_cb_param_t *param - gap callback parameters union
******************************************************************************************
Abstract:
This is a callback handler for gap event.
\****************************************************************************************/
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/****************************************************************************************\
Function:
gap_event_handler
******************************************************************************************
Parameters:
esp_gatts_cb_event_t event -
esp_gatt_if_t gatts_if -
esp_ble_gatts_cb_param_t *param
******************************************************************************************
Abstract:
This is a callback handler for gatts event.
\****************************************************************************************/
static void gatts_profile_threshold_exceeded_notification(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_get_calculated_values(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_trigger_measurement(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void set_uuid(uint16_t new_uuid_16bit, uint8_t uuid_tab[]);
static void initialize_calculated_values_attr_vals_array(void);

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

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
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

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
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
	}
};

static const uint16_t calculated_values_char_uuid[MAX_CALCULATED_VALUES] =
{
	GATTS_CHAR_UUID_GET_RMS_VALUE,
	GATTS_CHAR_UUID_GET_AVERAGE_VALUE,
	GATTS_CHAR_UUID_GET_MAX_VALUE,
	GATTS_CHAR_UUID_GET_MIN_VALUE,
	GATTS_CHAR_UUID_GET_AMPLITUDE_VALUE,
	GATTS_CHAR_UUID_GET_CREST_FACTOR_VALUE
};

static esp_attr_value_t calculated_values_attr_val_tab[MAX_CALCULATED_VALUES];

static calculated_val_rsp calculated_vals_response_tab[MAX_CALCULATED_VALUES];

static bool measurement_trigger_request = false;

uint8_t char1_str[] = {0x11,0x22,0x33};

esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////
void ble_communication_init()
{
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
	/** set mtu */
	esp_ble_gatt_set_local_mtu(500);
}

/****************************************************************************************/
void ble_communication_threshold_exceeded_notification_send(uint16_t exceeded_value)
{
	uint8_t val[2] = {(exceeded_value>>8)&0xff, (exceeded_value)&0xff};
	esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].gatts_if, gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].conn_id, gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_handle, sizeof(val), val, false);
}

/****************************************************************************************/
void ble_communication_update_calculated_value(calculated_value type, float val)
{
	calculated_vals_response_tab[type].float_type = val;
}

/****************************************************************************************/
bool ble_communication_is_measurement_requested(void)
{
	if(true == measurement_trigger_request){
		measurement_trigger_request = false;
		return true;
	}
	return false;
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
         ESP_LOGI(GATTS_TAG, "update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
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
    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/****************************************************************************************/
static void gatts_profile_threshold_exceeded_notification(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.is_primary = true;
//		TODO: what is the instance id?
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.id.uuid.len = ESP_UUID_LEN_128;
		set_uuid(GATTS_SERVICE_UUID_THRESHOLD_EXCEEDED_NOTIFICATION, gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id.id.uuid.uuid.uuid128);
		esp_ble_gap_set_device_name(DEVICE_NAME);
		esp_ble_gap_config_adv_data(&adv_data);
		esp_ble_gap_config_adv_data(&scan_rsp_data);
		esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_id, 0x2e);

		break;
	case ESP_GATTS_READ_EVT:{
		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = 2;
		rsp.attr_value.value[0] = 0xde;
		rsp.attr_value.value[1] = 0xad;
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
	}
		break;
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
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle = param->create.service_handle;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_uuid.len = ESP_UUID_LEN_128;
		set_uuid(GATTS_CHAR_UUID_THRESHOLD_EXCEEDED_NOTIFICATION, gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_uuid.uuid.uuid128);

		esp_ble_gatts_start_service(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle);
		esp_ble_gatts_add_char(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle, &gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_uuid,
		ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, &gatts_demo_char1_val, NULL);
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:;
		uint16_t length = 0;
		const uint8_t *prf_char;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].char_handle = param->add_char.attr_handle;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
		esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].service_handle, &gl_profile_tab[PROFILE_THRESHOLD_EXCEEDED_NOTIFICATION].descr_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
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

static void gatts_profile_get_calculated_values(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_GET_CALCULATED_VALUES].service_id.is_primary = true;
//		TODO: what is the instance id?
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
		//TODO: get to know how to corelate characteristics with their handles and correct this
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
static void gatts_profile_trigger_measurement(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
switch(event){
	case ESP_GATTS_REG_EVT:
		gl_profile_tab[PROFILE_TRIGGER_MEASUREMENT].service_id.is_primary = true;
//		TODO: what is the instance id?
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
			measurement_trigger_request = true;
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
					ESP_GATT_CHAR_PROP_BIT_WRITE, &(gatts_demo_char1_val),
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
static void set_uuid(uint16_t new_uuid_16bit, uint8_t uuid_tab[])
{
	uint8_t new_uuid_tab[16] = INITIALIZE_UUID_TABLE(new_uuid_16bit);
	memcpy(uuid_tab, new_uuid_tab, ESP_UUID_LEN_128);
}

/****************************************************************************************/
static void initialize_calculated_values_attr_vals_array(void)
{
	//TODO: get to know how to not register it
	for(uint8_t i = 0; i<MAX_CALCULATED_VALUES; ++i){
		calculated_values_attr_val_tab[i].attr_len = sizeof(float);
		calculated_values_attr_val_tab[i].attr_max_len = GATTS_CHAR_VAL_LEN_MAX;
		calculated_values_attr_val_tab[i].attr_value = calculated_vals_response_tab[i].int_type;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

