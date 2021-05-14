#include "ch.h"
#include <chprintf.h>
#include <chthreads.h>
#include <sensors/proximity.h>
#include <math.h>

#include <main.h>
#include <motors.h>
#include <leds.h>
#include <navigation.h>
#include <analyze_horizon.h>
#include <victory.h>


static _Bool empty_lake = 0;
static _Bool lake_scanned = 0;
static _Bool ready_to_save = 0;

static int step_to_turn = 0;

static int mode = 5; //initialized with a value that is not in the switch (main.c) in order not to start a random thread

//simple PI regulator implementation
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

	speed = KP * err + KI * sum_error; // avec ou sans KI ?
	if(speed){
	} else {
	}

    return (int16_t)speed;
}


static THD_WORKING_AREA(waGoToSwimmer, 512);
static THD_FUNCTION(GoToSwimmer, arg) {

	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

		while(1){
			time = chVTGetSystemTime();
			if((mode==1) && (!ready_to_save)){

				//computes the speed to give to the motors
				//distance_cm is modified by the image processing thread

				speed = crawl_to_swimmer(get_distance_cm(), GOAL_DISTANCE);

				if(get_distance_cm() <= GOAL_DISTANCE){
					speed = 0;
				}

				wait_im_ready(); // Without this, turns further than real position of ball before correcting trajectory

				int16_t tmp = get_swimmer_position();

				//computes a correction factor to let the robot rotate to be in front of the line
				speed_correction = (tmp - (IMAGE_BUFFER_SIZE/2));

				//if the line is nearly in front of the camera, don't rotate
				if(abs(speed_correction) < ROTATION_THRESHOLD){
					speed_correction = 0;
				}

				//applies the speed from the PI regulator and the correction for the rotation

				right_motor_set_speed(speed - ROT_KP * speed_correction);
				left_motor_set_speed(speed + ROT_KP * speed_correction);

				if((speed_correction == 0) && (get_distance_cm() <= (GOAL_DISTANCE + 5))) // constantes à ajuster
				{
					while (get_prox(0) < 150){
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

	wait_im_ready();
	search_left_shore();


    int turn_count = 0;
    int swimmer_found = 0;
    int initial_count = left_motor_get_pos();

    lake_scanned = 0;

    while(1){
    	time = chVTGetSystemTime();
    	if((mode==0) && (!lake_scanned))
    	{
    		wait_im_ready(); // utile ici ?

    		search_left_shore();

    		// Turns left until finds left shore

    		while (!get_left_shore()){
    			wait_im_ready(); // utile ?
    			swimmer_found = 0;
    			right_motor_set_speed(+ MOTOR_SPEED_LIMIT/10);
    			left_motor_set_speed(- MOTOR_SPEED_LIMIT/10);
    		}


    		right_motor_set_speed(0);
    		left_motor_set_speed(0);

    		wait_im_ready(); // IMPORTANT ICI (avant le search right shore)

			step_to_turn = 0;
			initial_count = left_motor_get_pos();

			search_right_shore();

			if(swimmer_found && get_left_shore()){
				if(get_swimmer_position() < get_left_shore_position()){
					swimmer_found = 0;
				}
			}

			// Turns right until finds a swimmer or the right shore
			// Stops as soon as the shore line enters its field of view
			// this way, swimmers already on the beach cannot be seen

			while((!swimmer_found) && (!get_right_shore())){
				set_led(LED3, 1);

				right_motor_set_speed(-MOTOR_SPEED_LIMIT/10);
				left_motor_set_speed(+MOTOR_SPEED_LIMIT/10);
				turn_count = (left_motor_get_pos() - initial_count);
				swimmer_found = get_swimmer_width();
			}


			wait_im_ready(); // utile ?

			// test to avoid case where swimmer and shore both in field of view

			if(swimmer_found && get_right_shore()){
				if(get_swimmer_position() > get_right_shore_position()){
					swimmer_found = 0;
				}
			}

			right_motor_set_speed(0);
			left_motor_set_speed(0);

			clear_shore();
			clear_leds();


			if (swimmer_found){

				right_motor_set_speed(0); // peut ere pas nécessaire, choisir avant ou après les if
				left_motor_set_speed(0);

				set_led(LED5, 1);

				empty_lake = 0;
				step_to_turn = (HALF_TURN_COUNT/2) - turn_count;

			}

			if (!swimmer_found){

				set_led(LED7, 1);

				empty_lake = 1;
				step_to_turn = 0;
			}

			lake_scanned = 1;

    	}
    	//100Hz
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


void clear_ready_to_save(void){
	ready_to_save = 0;
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

int get_step_to_turn(){
	return step_to_turn ; //= (HALF_TURN_COUNT/2) - turn_count;
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

void turn_left(int turn_count, int div){

	int initial_count = left_motor_get_pos();

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	while (abs(left_motor_get_pos() - initial_count) < turn_count)
	{
		right_motor_set_speed(+MOTOR_SPEED_LIMIT/div);
		left_motor_set_speed(-MOTOR_SPEED_LIMIT/div);
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void turn_right(int turn_count, int div){

	int initial_count = left_motor_get_pos();

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	while (abs(left_motor_get_pos() - initial_count) < turn_count)
	{
		right_motor_set_speed(-MOTOR_SPEED_LIMIT/div);
		left_motor_set_speed(+MOTOR_SPEED_LIMIT/div);
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
		right_motor_set_speed(MOTOR_SPEED_LIMIT/4);
		left_motor_set_speed(MOTOR_SPEED_LIMIT/4);
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void clear_lake (void){
	lake_scanned = 0;
}


