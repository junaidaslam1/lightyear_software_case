/** @file
 *  @brief Main file.
 *  @description This file contains entry to the software case from Lightyear
 */

#include <stdio.h>
#include <stdint.h>

#include "drivers/error_led/error_led.h"
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "Torque_Module.h"

/************************************************
 * 	Module definitions
 ***********************************************/
bool 	g_TwoSpeed	=	true, \
	g_ThreadedImplementation = false;

static float 	s_Angle = 0.0, \
		s_Torque = 0.0;

static unsigned int s_Speed = 0;

static volatile bool 	s_AngleReleaseTorqueThread = false,  \
			s_SpeedReleaseTorqueThread = false;

static pthread_mutex_t s_SharedMutex = PTHREAD_MUTEX_INITIALIZER;

static int Calculate_Angle(void)
/**
 * Description: The function is created to elude code duplication
 * 				for calculating Angle of throttle pedal
 * Inputs:
 * Output:
 * Return:	returns OK / NOK
 */
{
	int lvThrottleInput = get_user_throttle_input();
	#if DEBUG
		printf("throttle:%d %s\n", lvThrottleInput, lvThrottleInput==NOK?"should throw error":"OK");
	#endif
	if(lvThrottleInput == NOK) {
		error_led_set(true);
		adc_read_set_output(ADC_CHANNEL0, ADC_ERROR_VALUE, ADC_RET_NOK);
		adc_read_set_output(ADC_CHANNEL1, ADC_ERROR_VALUE, ADC_RET_NOK);
		s_Angle = ANGLE_ERR_VALUE;
		printf("[Error Angle Calc] Throttle_Percent < %d ; Assigned_Dummy_Angle:%.2fDeg\n", THROTTLE_ERR_THRESHOLD, s_Angle);
		return NOK;
	} else {
		s_Angle = get_pedal_angle(lvThrottleInput);
		adc_read_set_output(ADC_CHANNEL0, calc_adc_value(ADC_CHANNEL0, s_Angle)*ADC_MULTIPLIER, ADC_RET_OK);
		adc_read_set_output(ADC_CHANNEL1, calc_adc_value(ADC_CHANNEL1, s_Angle)*ADC_MULTIPLIER, ADC_RET_OK);
		#if DEBUG
			printf("s_Angle:%f\n", s_Angle);
		#endif
	}
	return OK;
}

void* AngleCalc_Thread(void *args)
/**
 * Description: The function calculates angle of the applied throttle pedal of the vehicle
 * Inputs:
 * Output:
 * Return:	returns OK / NOK based on the result and context of execution
 */
{
	static int lvResult = OK;
    pthread_t tid;
    tid = pthread_self();

	if(!g_ThreadedImplementation) {
		lvResult	=	Calculate_Angle();
	} else {
		printf("Entering thread:%s ID:%lu\n", __func__, tid);
		while(1) {
			if(!s_AngleReleaseTorqueThread) {
				lvResult = Calculate_Angle();
				if(lvResult == OK) {
					pthread_mutex_lock(&s_SharedMutex);
					s_AngleReleaseTorqueThread	=	true;
					pthread_mutex_unlock(&s_SharedMutex);
				} else {
					sleep(1);
				}
			}
			usleep(1000);
		}
	}

	return (void*)&lvResult;
}

static int Calculate_Speed(void)
/**
 * Description: The function is to avoid code duplication for calculating speed
 * Inputs:
 * Output:
 * Return: returns OK / NOK
 */
{
	if(g_TwoSpeed) {
		s_Speed = get_fixed_speed();
	} else {
		unsigned int lvRotationCounter = get_rotation_timer_count();
		s_Speed = get_rpm_based_speed(lvRotationCounter);
	}
	#if DEBUG
		printf("s_Speed:%u %s\n", s_Speed, s_Speed==SPEED_ERR_THRESHOLD ?"should throw error":"OK");
	#endif
	if(s_Speed > MAX_POSSIBLE_SPEED) {
		error_led_set(true);
		printf("[Error Speed Calc] Speed:%u\n", s_Speed);
		return NOK;
	}
	return OK;
}

void* SpeedCalc_Thread(void *args)
/**
 * Description: The function calculates speed of the vehicle
 * Inputs:
 * Output:
 * Return: returns OK / NOK based on the result and context of execution
 */
{
	static int lvResult = OK;
    pthread_t tid;
    tid = pthread_self();

	if(!g_ThreadedImplementation) {
		lvResult = Calculate_Speed();
	} else {
		printf("Entering thread:%s ID:%lu\n", __func__, tid);
		while(1) {
			if(!s_SpeedReleaseTorqueThread) {
				lvResult = Calculate_Speed();
				if(lvResult == OK) {
					pthread_mutex_lock(&s_SharedMutex);
					s_SpeedReleaseTorqueThread	=	true;
					pthread_mutex_unlock(&s_SharedMutex);
				} else {
					sleep(1);
				}
			}
			usleep(1000);
		}
	}

	lvResult = OK;
	return (void*)&lvResult;
}

static void Calculate_Torque(void)
/**
 * Description: The function is to avoid code duplication for calculating torque
 * Inputs:
 * Output:
 * Return:
 */
{
	if(g_TwoSpeed) {
		s_Torque = get_torque_two_speed(s_Angle, s_Speed==0?Resting:Moving);
		#if DEBUG
			printf("TwoSpeed Torque:%f %s\n", s_Torque, s_Torque==(-50)?"should throw error":"OK");
		#endif
	} else {
		s_Torque = get_torque_rpm_based_speed(s_Angle, s_Speed);
		#if DEBUG
			printf("Random Torque:%f %s\n", s_Torque, s_Torque==(-50)?"should throw error":"OK");
		#endif
	}
}

void* TorqueCalc_Thread(void *args)
/**
 * Description: The function calculates torque w.r.t previously calculated angle and speed
 * Inputs:
 * Output:
 * Return:
 */
{
    pthread_t tid;
    tid = pthread_self();

	if(!g_ThreadedImplementation) {
		Calculate_Torque();
	} else {
		printf("Entering thread:%s ID:%lu\n", __func__, tid);
		while(1)
		{
			if(s_AngleReleaseTorqueThread && s_SpeedReleaseTorqueThread) {
				Calculate_Torque();

				adc_value_t lvADC1 = 0.0,  lvADC2 = 0.0;
	    		(void)adc_read(ADC_CHANNEL0, &lvADC1);
	    		(void)adc_read(ADC_CHANNEL1, &lvADC2);

	    		printf("Speed:%uKm/h Throttle Angle:%.2fDeg Torque:%.2fNm ADC1:%u ADC2:%u\n",
	    				s_Speed, s_Angle, s_Torque, lvADC1, lvADC2);

				pthread_mutex_lock(&s_SharedMutex);
				s_AngleReleaseTorqueThread	=	false;
				s_SpeedReleaseTorqueThread	=	false;
				pthread_mutex_unlock(&s_SharedMutex);
			}
			usleep(500000);
		}
	}
	return NULL;
}

void Thread_creator(void)
/**
 * Description: The function creates Angle, Speed and Torque calculator threads.
 * 				these threads are just for DEMO, and is one of the ways of doing things.
 * 				A simpler implementation can simply call these three functions in sequence.
 * Inputs:
 * Output:
 * Return:
 */
{
	pthread_t lv_angle_thread, lv_speed_thread, lv_torque_thread;
	pthread_attr_t lv_thread_attr1, lv_thread_attr2, lv_thread_attr3;

	(void)pthread_attr_init(&lv_thread_attr1);
		(void)pthread_attr_setdetachstate(&lv_thread_attr1, PTHREAD_CREATE_DETACHED);

	(void)pthread_attr_init(&lv_thread_attr2);
		(void)pthread_attr_setdetachstate(&lv_thread_attr2, PTHREAD_CREATE_DETACHED);

	(void)pthread_attr_init(&lv_thread_attr3);
		(void)pthread_attr_setdetachstate(&lv_thread_attr3, PTHREAD_CREATE_DETACHED);

	/* Spawn Angle Calculator Thread */
	(void)pthread_create(&lv_angle_thread, &lv_thread_attr1, AngleCalc_Thread, NULL);
	/* Spawn Speed Calculator Thread Based on selected speed mode */
	(void)pthread_create(&lv_speed_thread, &lv_thread_attr2, SpeedCalc_Thread, NULL);
	/* Spawn Torque Calculator Thread */
	(void)pthread_create(&lv_torque_thread, &lv_thread_attr3, TorqueCalc_Thread, NULL);

	while(1);
}

int Torque_Calculator(void)
/**
 * Description: The function calls Angle, Speed and torque calculator functions
 * 				in sequence based on acquired values from intermediate and subsequent
 * 				functions.
 * Inputs:
 * Output:
 * Return:
 */
{
	printf("Entering thread:%s\n", __func__);
    while(1)
    {
    	int *lvResult = (int*)AngleCalc_Thread(NULL);

    	if(*lvResult != NOK) {
    		lvResult = (int*)SpeedCalc_Thread(NULL);
    	}

    	if(*lvResult != NOK) {
    		(void)TorqueCalc_Thread(NULL);
    	}

    	if(*lvResult == NOK) {
    		printf("[Error Torque Calc]...\n");
    	} else {
    		adc_value_t lvADC1 = 0.0,  lvADC2 = 0.0;
    		(void)adc_read(ADC_CHANNEL0, &lvADC1);
    		(void)adc_read(ADC_CHANNEL1, &lvADC2);

    		printf("Speed:%uKm/h Throttle Angle:%.2fDeg Torque:%.2fNm ADC1:%u ADC2:%u\n",
    				s_Speed, s_Angle, s_Torque, lvADC1, lvADC2);
    	}
    	sleep(1);
    }
	return 0;
}

int main(int argc, char *argv[])
/**
 * Description: This the entry point for the torque calculator system.
 * 				It initializes system resources based on the selected speed mode.
 * Inputs:
 * Output:
 * Return: 		error code
 */
{
  error_led_init();
  adc_init(ADC_CHANNEL0);
  adc_init(ADC_CHANNEL1);

  if(argc == 3)
  {

	if(strcmp(argv[1], "ts") == 0)
	{
		g_TwoSpeed = true;
		init_two_speed_torque_data();
		printf("Getting torque for distinct speed values\n");
	}
	else if(strcmp(argv[1], "cs") == 0)
	{
		g_TwoSpeed = false;
		printf("Getting torque for random speed values\n");
	}
	else
	{
		printf("Error Parsing 1st input\n");
		error_led_set(true);
		return -1;
	}

	if(strcmp(argv[2], "mt") == 0)
	{
		g_ThreadedImplementation = true;
		printf("Getting with multi-threaded implementation\n");
	}
	else if(strcmp(argv[1], "pl") == 0)
	{
		g_ThreadedImplementation = false;
		printf("Getting torque with plain implementation\n");
	}
	else
	{
		printf("Error Parsing 2nd input\n");
		error_led_set(true);
		return -1;
	}
  }
  else if((argc > 1) && (argc < 3))
  {
	  printf("Select the following options: [default:1 - ts, 2 - pl]\n"
			  "1 - ts or cs (ts = Two speed only selects 0 or 50 km/h values for speed)\n"
			  "	   	(cs = randomly selects between 0 and 50 km/h values for speed)\n"
			  "2 - mt or pl (mt = multi-threaded ; pl = plain implementation)\n");
	  return -1;
  }
  else
  {
	  printf("Getting torque for two distinct speed values (0 and 50)km/h\n"
			  "Using plain sequential implementation...\n");
	  init_two_speed_torque_data();
  }

  if(!g_ThreadedImplementation) {
	  Torque_Calculator();
  } else {
	  if(!(pthread_mutex_init(&s_SharedMutex, NULL)))
	  {
		  printf("Mutex Successfully initialized\n");
	  }
	  Thread_creator();
  }

  /** should not reach here anyway
   * */
  return 0;
}
