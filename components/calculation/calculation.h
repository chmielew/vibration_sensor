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
/** typedef of module object definition **/
typedef struct Calculation_obj *Calculation_obj_handle;

/** enum determining available factors which are calculated by the module **/
typedef enum {
	CALCULATION_RMS = 0,
	CALCULATION_AVERAGE,
	CALCULATION_MAXVAL,
	CALCULATION_MINVAL,
	CALCULATION_AMPLITUDE,
	CALCULATION_CREST_FACTOR
} calculation_factors;

/** enum determining the state of the object **/
typedef enum {
	CALCULATION_NOT_INITIALIZED = 0,
	CALCULATION_INITIALIZED,
	CALCULATION_IN_PROGRESS,
	CALCULATION_FINISHED
} calculation_state;

/** union for access of the values in different types **/
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
calculation_NewObj
******************************************************************************************
Parameters:
uint16_t data[] - pointer to the data array
uint16_t size - size of the data
******************************************************************************************
Abstract:
This function creates an object of calculation and returns a pointer to that object.
\****************************************************************************************/
Calculation_obj_handle calculation_new_obj(uint16_t data[], uint16_t size);

/****************************************************************************************\
Function:
calculation_get_size
******************************************************************************************
Parameters:
Calculation_obj_handle obj - handle to object on which the function should operate
******************************************************************************************
Abstract:
Thus function returns the size of data stored inside the obj.
\****************************************************************************************/
uint16_t calculation_get_size(Calculation_obj_handle obj);

/****************************************************************************************\
Function:
calculation_get_data_ptr
******************************************************************************************
Parameters:
Calculation_obj_handle obj - handle to object on which the function should operate
******************************************************************************************
Abstract:
This function returns pointer to the obj data.
\****************************************************************************************/
uint16_t * calculation_get_data_ptr(Calculation_obj_handle obj);

/****************************************************************************************\
Function:
calculation_calculate_factors
******************************************************************************************
Parameters:
Calculation_obj_handle obj - handle to object on which the function should operate
******************************************************************************************
Abstract:
This funtion triggers calculation of the factors.
\****************************************************************************************/
void calculation_calculate_factors(Calculation_obj_handle obj);

/****************************************************************************************\
Function:
calculation_get_factor
******************************************************************************************
Parameters:
Calculation_obj_handle obj - handle to object on which the function should operate
calculation_factors factor - desired factor to be returned
******************************************************************************************
Abstract:
This function returns the desired factor from the obj. It should be called when the 
calculation is finished
\****************************************************************************************/
calculation_factor_type calculation_get_factor(Calculation_obj_handle obj, calculation_factors factor);

/****************************************************************************************\
Function:
calculation_get_state
******************************************************************************************
Parameters:
Calculation_obj_handle obj - handle to object on which the function should operate
******************************************************************************************
Abstract:
This function returns the state of obj.
\****************************************************************************************/
calculation_state calculation_get_state(Calculation_obj_handle obj);

/****************************************************************************************\
Function:
calculation_delete_obj
******************************************************************************************
Parameters:
Calculation_obj_handle obj - handle to object on which the function should operate
******************************************************************************************
Abstract:
This function deletes the obj.
\****************************************************************************************/
void calculation_delete_obj(Calculation_obj_handle * obj);

/****************************************************************************************\
Function:
calculation_rms
******************************************************************************************
Parameters:
void *pvParameter - standard parameter for freertos task
******************************************************************************************
Abstract:
rms calculation task function
\****************************************************************************************/
void calculation_rms(void *pvParameter);

/****************************************************************************************\
Function:
calculation_average
******************************************************************************************
Parameters:
void *pvParameter - standard parameter for freertos task
******************************************************************************************
Abstract:
average calculation task function
\****************************************************************************************/
void calculation_average(void *pvParameter);

/****************************************************************************************\
Function:
calculation_range
******************************************************************************************
Parameters:
void *pvParameter - standard parameter for freertos task
******************************************************************************************
Abstract:
range calculation task function
\****************************************************************************************/
void calculation_range(void *pvParameter);

/****************************************************************************************\
Function:
calculation_amplitude
******************************************************************************************
Parameters:
void *pvParameter - standard parameter for freertos task
******************************************************************************************
Abstract:
amplitude calculation task function
\****************************************************************************************/
void calculation_amplitude(void *pvParameter);

/****************************************************************************************\
Function:
calculation_crest_factor
******************************************************************************************
Parameters:
void *pvParameter - standard parameter for freertos task
******************************************************************************************
Abstract:
crest factor calculation task function
\****************************************************************************************/
void calculation_crest_factor(void *pvParameter);

//////////////////////////////////////////////////////////////////////////////////////////
//End of file																			//
//////////////////////////////////////////////////////////////////////////////////////////

#endif /* COMPONENTS_CALCULATION_CALCULATION_H_ */

