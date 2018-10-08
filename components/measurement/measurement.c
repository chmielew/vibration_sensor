/** measurement.c **/

//////////////////////////////////////////////////////////////////////////////////////////
//Includes																				//
//////////////////////////////////////////////////////////////////////////////////////////
#include "measurement.h"
#include "driver/timer.h"
#include "esp_intr_alloc.h"

//////////////////////////////////////////////////////////////////////////////////////////
//Macros																				//
//////////////////////////////////////////////////////////////////////////////////////////
/** Timer divider value for the timer operation
 *  it has minimal value due to size of timer counter (64b) 
 **/
#define TIMER_DIVIDER_VALUE			((uint16_t)2) 
#define ZERO_VAL_AVERAGING_SAMPLES_NO 	((uint8_t)10)
#define ACCELEROMETER_ADC_CHANNEL 	(ADC1_CHANNEL_7)

//////////////////////////////////////////////////////////////////////////////////////////
//Local typedefs																		//
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//Static variables																		//
//////////////////////////////////////////////////////////////////////////////////////////
static uint16_t max_measurement_number;
static uint16_t * measurement_ptr;
static uint16_t current_measurement;
static measurement_status current_status = MEASUREMENT_NOT_INITIALIZED;
static uint16_t zero_val = 0;

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions prototypes															//
//////////////////////////////////////////////////////////////////////////////////////////

/****************************************************************************************\
Function:
measurement_timer_interrupt_function
******************************************************************************************
Parameters:
void * param - standard parameter for interrupt
******************************************************************************************
Abstract:
This is an interrupt function for a timer to read the adc conversion on timer interrupt
into the buffer.
\****************************************************************************************/
//TODO: convert to static??
static void IRAM_ATTR measurement_timer_interrupt_function(void *param);

//////////////////////////////////////////////////////////////////////////////////////////
//Global functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

void measurement_init(adc1_channel_t channel, adc_atten_t attenuation, adc_bits_width_t width)
{
	/* Initialize ADC */
	adc1_config_width(width);
	adc1_config_channel_atten(channel, attenuation);

	/* Initialize timer */
	timer_config_t config;
	config.alarm_en = TIMER_ALARM_EN;
	config.auto_reload = true;
	config.divider = 16;
	config.counter_dir = TIMER_COUNT_UP;
	config.counter_en = TIMER_PAUSE;
	config.intr_type = TIMER_INTR_LEVEL;
	timer_isr_register(TIMER_GROUP_0, TIMER_0, measurement_timer_interrupt_function,
				   	(void*)NULL, ESP_INTR_FLAG_IRAM, NULL);
	timer_init(TIMER_GROUP_0, TIMER_0, &config);
	current_status = MEASUREMENT_INITIALIZED;

	zero_val = 0;
	for (uint8_t i = 0; i < ZERO_VAL_AVERAGING_SAMPLES_NO; ++i) {
		zero_val += measurement_read(ACCELEROMETER_ADC_CHANNEL);
	}
	zero_val /= ZERO_VAL_AVERAGING_SAMPLES_NO;

}
/****************************************************************************************/

uint16_t measurement_read(adc1_channel_t channel)
{
	return adc1_get_raw(channel);
}
/****************************************************************************************/

uint16_t * measurement_trigger(uint16_t frequency, float duration)
{
	if(duration > 0 && frequency > 0){
	uint32_t counter = frequency*duration;
	measurement_ptr = malloc(counter*sizeof(uint16_t));
	max_measurement_number = counter;

	uint64_t alarm_value = APB_CLK_FREQ/(TIMER_DIVIDER_VALUE*frequency);
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, alarm_value);
	timer_set_divider(TIMER_GROUP_0, TIMER_0, TIMER_DIVIDER_VALUE);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);
	TIMERG0.hw_timer[0].config.alarm_en = TIMER_ALARM_EN;
	timer_start(TIMER_GROUP_0, TIMER_0);
	current_measurement = 0;
	current_status = MEASUREMENT_ACTIVE;
	return measurement_ptr;
	} else{
		return NULL;
	}
}
/****************************************************************************************/

measurement_status measurement_get_status(void)
{
	return current_status;
}
/****************************************************************************************/

uint16_t * measurement_get_pointer(void)
{
	return measurement_ptr;
}
/****************************************************************************************/

uint32_t measurement_get_size(void)
{
	return max_measurement_number;
}
/****************************************************************************************/

uint16_t measurement_get_zero_val(void)
{
	return zero_val;
}

//////////////////////////////////////////////////////////////////////////////////////////
//Static functions definitions															//
//////////////////////////////////////////////////////////////////////////////////////////

static void IRAM_ATTR measurement_timer_interrupt_function(void *param)
{
	if(current_measurement < max_measurement_number){
		TIMERG0.int_clr_timers.t0 = 1;
		TIMERG0.hw_timer[0].config.alarm_en = TIMER_ALARM_EN;
		*(measurement_ptr+current_measurement) = adc1_get_raw(ACCELEROMETER_ADC_CHANNEL);
		++current_measurement;
	} else{
		TIMERG0.int_clr_timers.t0 = 1;
		timer_pause(TIMER_GROUP_0, TIMER_0);
		timer_disable_intr(TIMER_GROUP_0, TIMER_0);
		current_status = MEASUREMENT_FINISHED;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////
