/**
 * @file
 * @brief Header file for declarations.
 */

#ifndef TORQUE_MODULE_H_
#define TORQUE_MODULE_H_

#include <stdbool.h>
#include "drivers/adc_driver/adc_driver.h"

/************************************************
 *  Macro definitions used in calculating
 *  different quantities.
 ***********************************************/
#define MAX_THROTTLE_POSSIBLE		100 	// With respect to 100% (30 degrees) pedal angle
#define MAX_THROTTLE_DATA_COUNT		101 	//

#define ROTATING_OBJECT_RADIUS	0.5	// Meters
#define ROTATING_OBJECT_CIRCUM	2*3.14*ROTATING_OBJECT_RADIUS // 2*pi*r (r being the radius)

#define SECONDS_IN_HOUR		3600
#define METERS_IN_KM			1000
#define MILLISECONDS_IN_SECOND	1000
#define SECONDS_IN_A_MINUTE		60
#define MINUTES_IN_A_HOUR		60

#define MAX_TIMER_COUNT		1000
#define MIN_TIMER_COUNT		220

#define ADC1_MIN_VOLT			0.5
#define ADC2_MIN_VOLT			1.0

#define SIMULATION_SPEED_LEVELS	2	//	0 and 50 km/h

#define SPEED_AT_REST			0	// Km/h
#define SPEED_AT_MOVE			50	// Km/h
#define TWO_SPEED_DUMMY_THRESHOLD	25 	// Km/h just for the demo

#define MIN_ANGLE			0	// degrees
#define MAX_ANGLE			30	// degrees

#define TORQUE_AT_REST_0_DEG		0 	// Newton Meter
#define TORQUE_AT_50KM_0_DEG		-30 	// Newton Meter

#define TORQUE_AT_MAX_ANGLE		120 	// Newton Meter

#define MAX_POSSIBLE_SPEED		50	//	km/h for demo only

#define VAR_SPEED_TORQUE_DIFF_AT_0_THROTTLE	(0-(-30))/MAX_POSSIBLE_SPEED // Newton Meter

#define TORQUE_ERROR_VALUE		-50  	// Newton Meter
#define ADC_ERROR_VALUE		0 	//
#define THROTTLE_ERR_THRESHOLD	5 	// just for the demo
#define SPEED_ERR_THRESHOLD		MAX_POSSIBLE_SPEED+1 // Km/h just for the demo
#define ANGLE_ERR_VALUE		-30

#define MAX_ADC_VOLTAGE		5000 	// milli-volts
#define ADC_RESOLUTION			65536 	// 16 bit unsigned value
#define ADC_MULTIPLIER			(ADC_RESOLUTION/MAX_ADC_VOLTAGE)*1000
#define ADC_LPF_NR_OF_SAMPLES	25

#define OK				0
#define NOK				-1

#define DEBUG				0 	// 1 - Enables debug messages
#define CALC_SPEED_FROM_RPM		1 	// This is just to enable indirect way of calculating speed

/************************************************
 *  Enumeration definitions
 ***********************************************/
typedef enum {
	Resting,
	Moving,
	_SpeedLevels
}SpeedLevels;

/************************************************
 *  Structure definitions
 ***********************************************/
typedef struct {
	signed char pvRestingTorqueFiller[MAX_THROTTLE_DATA_COUNT];
	signed char pvMovingTorqueFiller[MAX_THROTTLE_DATA_COUNT];
}TorqueFiller_t;

/************************************************
 *  Global variable declarations
 ***********************************************/

/************************************************
 *  Function definitions
 ***********************************************/

/** @brief The function returns a random value in between
 * 	    MIN_TIMER_COUNT and MAX_TIMER_COUNT millisecond..
 *  @param[in]
 *  @param[ret] milliseconds elapsed between hypothetical consecutive rotations.
 *  @note
 */
unsigned int get_rotation_timer_count(void);

/** @brief This function returns speed of the vehicle
 *  @param[in]  Timer_counts is the time in millisecond elapsed between two consecutive rotation interrupts,
 * 				or signals captured via timer.
 *  @param[ret] Speed of the vehicle.
 *  @note
 */
unsigned int get_rpm_based_speed(unsigned int timer_counts);

/** @brief This function returns speed of the vehicle between two fixed values [SPEED_AT_REST and SPEED_AT_MOVE]
 *  @param[in]
 *  @param[ret] Speed of the vehicle.
 *  @note
 */
unsigned int get_fixed_speed(void);

/** @brief The function returns a random value in between 0 and 100.
 * 	   The applied throttle is between 0 and 100% of the total mechanical
 * 	   capacity of the throttle pedal.
 *  @param[in]
 *  @param[ret] Applied Throttle.
 *  @note
 */
int get_user_throttle_input(void);

/** @brief The function returns angle between 0 and 30 degrees based
 *	    on applied throttle from 0% to 100% of its mechanical range.
 *  @param[in]  Applied Throttle.
 *  @param[ret] Pedal Angle.
 *  @note
 */
float get_pedal_angle(unsigned int throttle_applied);

/** @brief This function returns ADC value from specific channel.
 *  @param[in]  adc_channel_id_t.
 *  @param[in]  angle w.r.t applied throttle.
 *  @param[ret] adc_value_t
 *  @note
 */
adc_value_t calc_adc_value(adc_channel_id_t inID, float angle);

/** @brief This function returns torque with respect to two speed levels and exerted angle
 *  @param[in]  angle.
 *  @param[in]  _SpeedLevel (Resting/Moving).
 *  @param[ret] torque
 *  @note
 */
signed char get_torque_two_speed(float angle, SpeedLevels _SpeedLevel);

/** @brief This function returns torque with respect to speed and exerted angle
 * 				This is calculated with hypothetically calculated difference of torque values
 * 				at 0% throttle between SPEED_AT_REST and SPEED_AT_MOVE assuming SPEED_AT_MOVE will be highest speed.
 *  @param[in]  angle.
 *  @param[in]  speed.
 *  @param[ret] torque
 *  @note
 */
signed char get_torque_rpm_based_speed(float angle, unsigned int speed);

/** @brief This function simply extrapolates the data of the provided graph,
 * 	   and fills up a hypothetical torque value array based on the two
 * 	   speed levels (SPEED_AT_REST/SPEED_AT_MOVE KPH) and Percentage of throttle angle capacity.
 * 	   This initialization is performed to skip non-trivial calculation at
 * 	   the run-time so as to increase performance even further.
 * 	   This is just one of the ways to do this kind of task and is here only
 * 	   for Demo purposes.
 *  @param[in]
 *  @param[ret]
 *  @note
 */
void init_two_speed_torque_data(void);

#endif /* TORQUE_MODULE_H_ */
