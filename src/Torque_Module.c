/** @file
 *  @brief torque_calculator file.
 *  @description This module calculates toque based on user input as well as current speed
 */

#include "Torque_Module.h"

#include "drivers/adc_driver/adc_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static TorqueFiller_t s_torque_filler				=	{0};
static float s_var_speed_torque_0_deg[MAX_POSSIBLE_SPEED]	=	{0.0};
static adc_value_t s_adc_samples[ADC_NUM_CHANNELS][ADC_LPF_NR_OF_SAMPLES]	=	{{0.0}};

int get_user_throttle_input(void)
/**
 * Description: The function returns a random value in between 0 and 100.
 * 		The applied throttle is between 0 and 100% of the total mechanical
 * 		capacity of the throttle pedal.
 * Inputs:
 * Output:
 * Return: 	applied_throttle
 */
{
	srand(time(0));
	unsigned int r_throttle = ((rand() % (MAX_THROTTLE_POSSIBLE - 0 + 1)) + 0);
	/** Below 10% of the max possible throttle (100%) 0 is returned
	 *  to indicate error just for demo.
	 */
	return r_throttle > THROTTLE_ERR_THRESHOLD ? r_throttle : NOK;
}

float get_pedal_angle(unsigned int throttle_applied)
/**
 * Description: The function returns angle between 0 and 30 degrees based
 * 		on applied throttle from 0% to 100% of its mechanical range.
 * Inputs: 	applied_throttle
 * Output:
 * Return: 	pedal_angle
 */
{
	return (float)((float)throttle_applied/MAX_THROTTLE_POSSIBLE)*MAX_ANGLE;
}

static adc_value_t get_movingAvg(adc_channel_id_t inID, adc_value_t nextSample)
/**
 * Description: This function returns ADC moving average value
 * Inputs:      adc_channel_id_t (inID)
 * 	     :	    angle w.r.t applied throttle
 * output:
 * return:      adc_value_t
 */
{
	static uint32_t lv_Sum = 0;
	static uint16_t adc_lpf_pos[ADC_NUM_CHANNELS] = {0};

	//Subtract the oldest number from the prev sum, add the new number
	lv_Sum = lv_Sum - s_adc_samples[inID][adc_lpf_pos[inID]] + nextSample;

	//Assign the nextNum to the position in the array
	s_adc_samples[inID][adc_lpf_pos[inID]] = nextSample;

	adc_lpf_pos[inID]++;
	if(adc_lpf_pos[inID] == ADC_LPF_NR_OF_SAMPLES) {
			adc_lpf_pos[inID] = 0;
	}
  /** In a normal situation we would not need following ternary statement
   *  as we would first take X number of samples, and then will apply
   *  the  moving average with each new sample; hence dividing by X.
   */
	adc_value_t lv_Mov_Avg = (adc_value_t)(s_adc_samples[inID][adc_lpf_pos[inID]] != 0 ? \
							 (lv_Sum / ADC_LPF_NR_OF_SAMPLES) : (lv_Sum/(adc_lpf_pos[inID])));

	#if DEBUG
		printf("%s | ADC_CHANNEL:%u SamplePosition:%u Removed_Sample:%u, NewSample:%u Sum:%u MovAvg:%u\n",
			   __func__, inID, adc_lpf_pos[inID], s_adc_samples[inID][adc_lpf_pos[inID]], nextSample, lv_Sum, lv_Mov_Avg);
	#endif

	return  lv_Mov_Avg;
}

adc_value_t calc_adc_value(adc_channel_id_t inID, float angle)
/**
 * Description: This function returns ADC value from specific channel
 * Inputs:      adc_channel_id_t (inID)
 * 	     :	    angle w.r.t applied throttle
 * output:
 * return:      adc_value_t
 */
{
	float lv_ADC	=	inID == ADC_CHANNEL0 ? (float)(0.5 + 0.1 * angle) : (float)(1.0 + 0.08 * angle);

	#if DEBUG
		printf("%s | ADC_CHANNEL:%d = %f => %u\n", __func__, inID, lv_ADC, (adc_value_t)(lv_ADC*ADC_MULTIPLIER));
	#endif

	return get_movingAvg(inID, (adc_value_t)(lv_ADC*ADC_MULTIPLIER));
}

unsigned int get_rotation_timer_count(void)
/**
 * Description: The function returns a random value in between
 * 		MIN_TIMER_COUNT and MAX_TIMER_COUNT millisecond.
 * Inputs:
 * output: 	milliseconds elapsed between consecutive rotations
 */
{
	srand(time(0));
	return ((rand() % (MAX_TIMER_COUNT - MIN_TIMER_COUNT + 1)) + MIN_TIMER_COUNT);
}

static unsigned int get_rpm(unsigned int timer_counts)
/**
 * Description: This function returns rotations per minute of the engine.
 * Inputs:	timer_counts is the time in millisecond elapsed between
 * 		two consecutive rotation interrupts or signals captured via free running timer.
 * output:
 * return: 	Speed of the vehicle
 */
{
	return ((float)1/((float)timer_counts/MILLISECONDS_IN_SECOND))*SECONDS_IN_A_MINUTE;
}

unsigned int get_rpm_based_speed(unsigned int timer_counts)
/**
 * Description: This function returns speed of the vehicle
 * Inputs:	timer_counts is the time in millisecond elapsed between two consecutive rotation interrupts,
 * 		or signals captured via timer.
 * output:
 * return: 	Speed of the vehicle
 */
{
	#if !CALC_SPEED_FROM_RPM
		return (unsigned int)(((float)ROTATING_OBJECT_CIRCUM*SECONDS_IN_HOUR)/\
				((float)MILLISECONDS_IN_SECOND*((float)timer_counts/METERS_IN_KM)));
	#else
		return (ROTATING_OBJECT_CIRCUM*get_rpm(timer_counts)*MINUTES_IN_A_HOUR)/METERS_IN_KM;
	#endif
}

unsigned int get_fixed_speed(void)
/**
 * Description: This function returns speed of the vehicle between two fixed values [SPEED_AT_REST and SPEED_AT_MOVE]
 * Inputs:
 * output:
 * return: 	Speed of the vehicle
 */
{
	srand(time(0));
	return ((rand() % (SPEED_AT_MOVE - SPEED_AT_REST + 1)) + SPEED_AT_REST) < TWO_SPEED_DUMMY_THRESHOLD ? SPEED_AT_REST : SPEED_AT_MOVE;
}

signed char get_torque_two_speed(float angle, SpeedLevels _SpeedLevel)
/**
 * Description: This function returns torque with respect to two speed levels and exerted angle
 * Inputs: 	angle
 * 	: 	speed
 * Output:
 * return: 	torque
 */
{
	unsigned int lv_Throttle = (unsigned int)(((float)angle/MAX_ANGLE)*MAX_THROTTLE_POSSIBLE);
	switch(_SpeedLevel) {
		case(Resting): {
			#if DEBUG
				printf("%s | Resting angle:%f lv_throttle:%u torque:%d\n", __func__, angle, lv_Throttle, s_torque_filler.pvRestingTorqueFiller[lv_Throttle]);
			#endif
			return s_torque_filler.pvRestingTorqueFiller[lv_Throttle];
		} break;
		case(Moving): {
			#if DEBUG
				printf("%s | Moving angle:%f lv_throttle:%u torque:%d\n", __func__, angle, lv_Throttle, s_torque_filler.pvMovingTorqueFiller[lv_Throttle]);
			#endif
			return s_torque_filler.pvMovingTorqueFiller[lv_Throttle];
		} break;
		default: break;
	}
	return TORQUE_ERROR_VALUE;
}

signed char get_torque_rpm_based_speed(float angle, unsigned int speed)
/**
 * Description: This function returns torque with respect to speed and exerted angle
 * 		This is calculated with hypothetically calculated difference of torque values
 * 		at 0% throttle between SPEED_AT_REST and SPEED_AT_MOVE assuming SPEED_AT_MOVE will be highest speed.
 * Inputs: 	angle
 * 	: 	speed
 * Output:
 * return: 	torque
 */
{
	unsigned int lv_throttle_applied 	=	(((float)angle/MAX_ANGLE)*MAX_THROTTLE_POSSIBLE);
	float lv_torque_step			=	(TORQUE_AT_MAX_ANGLE - s_var_speed_torque_0_deg[speed])/MAX_THROTTLE_POSSIBLE;
	float lv_torque			=	s_var_speed_torque_0_deg[speed];

	for(int throttle_applied = 1; throttle_applied <= lv_throttle_applied; throttle_applied++) {
		lv_torque	+=	lv_torque_step;
	}

	#if DEBUG
		printf("angle:%f Speed:%d torque_step:%f torque:%d\n", angle, speed, lv_torque_step, (signed char)lv_torque);
	#endif

	return (signed char)lv_torque;
}

void init_two_speed_torque_data(void)
/**
 * Description: This function simply extrapolates the data of the provided graph,
 * 		and fills up a hypothetical torque value array based on the two
 * 		speed levels (0/50 KPH) and Percentage of throttle angle capacity.
 * 		This initialization is performed to skip non-trivial calculation at
 * 		the run-time so as to increase performance even further.
 * 		This is just one of the ways to do this kind of task and is here only
 * 		for Demo purposes.
 * Inputs:
 * Output:
 * return:
 */
{
	float Torque_Step_Per_Angle[_SpeedLevels] = {0};

	printf("Filling torque structure for distinct two speed system\n");

	Torque_Step_Per_Angle[Resting]	=	(float)(TORQUE_AT_MAX_ANGLE-TORQUE_AT_REST_0_DEG)/MAX_THROTTLE_POSSIBLE;
	Torque_Step_Per_Angle[Moving] 	= 	(float)(TORQUE_AT_MAX_ANGLE-TORQUE_AT_50KM_0_DEG)/MAX_THROTTLE_POSSIBLE;

	s_torque_filler.pvRestingTorqueFiller[0]	=	TORQUE_AT_REST_0_DEG;
	s_torque_filler.pvMovingTorqueFiller[0]	=	TORQUE_AT_50KM_0_DEG;

	#if DEBUG
		printf("ThrottlePercent:%d RestingTorque:%d MovingTorque:%d\n", 0, \
				s_torque_filler.pvRestingTorqueFiller[0], \
				s_torque_filler.pvMovingTorqueFiller[0]);
	#endif

	float lv_Resting_Torque = TORQUE_AT_REST_0_DEG, lv_Moving_Torque = TORQUE_AT_50KM_0_DEG;

	for(unsigned int throttle_applied = 1; throttle_applied < MAX_THROTTLE_DATA_COUNT; throttle_applied++) {
		lv_Resting_Torque += Torque_Step_Per_Angle[Resting];
		lv_Moving_Torque += Torque_Step_Per_Angle[Moving];

		s_torque_filler.pvRestingTorqueFiller[throttle_applied]	=	(signed char)lv_Resting_Torque;
		s_torque_filler.pvMovingTorqueFiller[throttle_applied]	=	(signed char)lv_Moving_Torque;

		#if DEBUG
			printf("ThrottlePercent:%d RestingTorque:%d MovingTorque:%d\n", throttle_applied, \
					s_torque_filler.pvRestingTorqueFiller[throttle_applied], \
					s_torque_filler.pvMovingTorqueFiller[throttle_applied]);
		#endif
	}

	for(int i = 0; i <= MAX_POSSIBLE_SPEED; i++) {
		s_var_speed_torque_0_deg[i]	=	(float)-1*i*VAR_SPEED_TORQUE_DIFF_AT_0_THROTTLE;
		#if DEBUG
			printf("Speed:%dkm 0 throttle torque diff:%f\n", i, s_var_speed_torque_0_deg[i]);
		#endif
	}
}
