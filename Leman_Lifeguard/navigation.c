#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <chthreads.h>
#include <sensors/proximity.h>

#include <main.h>
#include <motors.h>
#include <leds.h>
#include <navigation.h>
#include <analyze_horizon.h>
#include <victory.h>


static _Bool empty_lake = 0;
static _Bool lake_scanned = 0;
static _Bool ready_to_save = 0;

static _Bool mode = 5; //initialized with a value that is not in the switch (main.c) in order not to start a random thread



//simple PI regulator implementation
// A la fin, supprimer paramètres de l'intégrateur, entraîne des comportements étranges

int16_t crawl_to_swimmer(float distance, float goal){

	volatile float err = 0;
	volatile float speed = 0;

	static float sum_error = 0;

	err = distance - goal;

	//disables the PI regulator if the error is to small
	//to avoid permanent movement
	if(fabs(err) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += err;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}
	else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * err; //
	if(speed){
	}
	else{
	}

    return (int16_t)speed;
}


static THD_WORKING_AREA(waGoToSwimmer, 512);
static THD_FUNCTION(GoToSwimmer, arg) {

	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    volatile int16_t speed = 0;
    volatile int16_t speed_correction = 0;

    static float sum_rot_error = 0;

		while(1){
			time = chVTGetSystemTime();
			if((mode==1) && !ready_to_save){

				//computes the speed to give to the motors
				//distance_cm is modified by the image processing thread

				speed = crawl_to_swimmer(get_distance_cm(), GOAL_DISTANCE);
				if (get_distance_cm() <= GOAL_DISTANCE){ //test
					speed = 0;
				}
				wait_im_ready(); // Without this, turns further than real position of ball before correcting trajectory
				set_front_led(0);

				volatile int16_t tmp = get_swimmer_position();

				//computes a correction factor to let the robot rotate to be in front of the line

				speed_correction = (tmp - (IMAGE_BUFFER_SIZE/2));
/*
				if(sum_rot_error > MAX_ROT_ERROR){
					sum_rot_error = MAX_ROT_ERROR;
					}else if(sum_rot_error < -MAX_ROT_ERROR){
						sum_rot_error = -MAX_ROT_ERROR;
					}*/

				//if the line is nearly in front of the camera, don't rotate

				if(abs(speed_correction) < ROTATION_THRESHOLD){
					speed_correction = 0;
				}

				//sum_rot_error += speed_correction;

				//applies the speed from the PI regulator and the correction for the rotation

				right_motor_set_speed(speed - ROT_KP * speed_correction - ROT_KI*sum_rot_error); // was right
				left_motor_set_speed(speed + ROT_KP * speed_correction + ROT_KI*sum_rot_error); // was left
				//set_front_led(1);

				// Test for IR implementation :
				if((speed_correction == 0) && (get_distance_cm() <= (GOAL_DISTANCE + 5))){ // à modifier
					while (get_prox(0) < 100){
						right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
					}

					right_motor_set_speed(0);
					left_motor_set_speed(0);
					ready_to_save = 1;
				}
			}
			//100Hz
			chThdSleepUntilWindowed(time, time + MS2ST(10));
		}
}


static THD_WORKING_AREA(waSearchSwimmer, 512);
static THD_FUNCTION(SearchSwimmer, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

	right_motor_set_speed(0);
	left_motor_set_speed(0);
    int turn_count = 0;
    int swimmer_found = 0;
    int initial_count = left_motor_get_pos();

    lake_scanned = 0;

    while(1){
    	time = chVTGetSystemTime();
    	if((mode==0) && !lake_scanned)
    	{
			while((!swimmer_found) && (turn_count<=2*HALF_TURN_COUNT)){
				wait_im_ready();
				right_motor_set_speed(-MOTOR_SPEED_LIMIT/6);
				left_motor_set_speed(+MOTOR_SPEED_LIMIT/6);
				turn_count = (left_motor_get_pos() - initial_count);
				swimmer_found = get_swimmer_width();
			}

			right_motor_set_speed(0);
			left_motor_set_speed(0);

		   if (swimmer_found){
				empty_lake = 0;
		   }

		   if (!swimmer_found){
				empty_lake = 1;
			}

		   lake_scanned = 1;


			//A modifier pour implémentation avec la plage

			/*_Bool facing_left_shore = 0;
			_Bool facing_right_shore = 0;
			_Bool set_target = 0;

			right_motor_set_speed(0);
			left_motor_set_speed(0);

			while(!facing_left_shore){//facing the lake = flanc montant vert/bleu

				right_motor_set_speed(MOTOR_SPEED_LIMIT/6);
				left_motor_set_speed(-MOTOR_SPEED_LIMIT/6);

				facing_left_shore = check_left_shore(); // = flanc bleu/vert, fonction dans analyze_horizon, pas encore codée
			}

			right_motor_set_speed(0);
			left_motor_set_speed(0);

			//Now the robot is facing the left side of the shore
			//It starts scanning for swimmers by turning slowly to the right

			while(!get_swimmer_position()){
				right_motor_set_speed(-MOTOR_SPEED_LIMIT/6);
				left_motor_set_speed(+MOTOR_SPEED_LIMIT/6);
				if(check_right_shore()){
					break;
				}
			}*/

		//100Hz
    	}
		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/* ================================================
 * Create the threads */

void go_to_swimmer_start(void){
	chThdCreateStatic(waGoToSwimmer, sizeof(waGoToSwimmer), NORMALPRIO, GoToSwimmer, NULL);
}

void search_swimmer_start(void){
	chThdCreateStatic(waSearchSwimmer, sizeof(waSearchSwimmer), NORMALPRIO, SearchSwimmer, NULL);
}


/* ================================================
 * Static variables return functions*/

_Bool get_empty_lake(void){
	return empty_lake;
}

_Bool get_lake_scanned(void){
	return lake_scanned;
}

_Bool get_ready_to_save(void){
	return ready_to_save;;
}

/* ================================================
 * Finite state machine management functions : chose code to execute */

void switch_to_search_swimmer(void)
{
	mode = 0;
}

void switch_to_go_to_swimmer(void)
{
	mode = 1;
}

void init_before_switch(void)
{
	mode = 5; // assign a value out of switch to make sure not to come back to another case
}

/* ================================================
 * Simple navigation functions*/

void turn_left(int turn_count){

	int initial_count = left_motor_get_pos();

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	while (abs(left_motor_get_pos() - initial_count) < turn_count)
	{
		right_motor_set_speed(MOTOR_SPEED_LIMIT/6);
		left_motor_set_speed(-MOTOR_SPEED_LIMIT/6);
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void turn_right(int turn_count){

	int initial_count = left_motor_get_pos();

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	while (abs(left_motor_get_pos() - initial_count) < turn_count)
	{
		right_motor_set_speed(-MOTOR_SPEED_LIMIT/2);
		left_motor_set_speed(+MOTOR_SPEED_LIMIT/2);
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void go_straight(float distance){

	int initial_count = left_motor_get_pos();

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	while (abs(left_motor_get_pos() - initial_count) < distance) //distance donnée par capteur de distance
	{
		right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
		left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

