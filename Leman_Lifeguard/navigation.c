#include "ch.h
#include <math.h>

#include <chprintf.h>
#include <chthreads.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <leds.h>

#include <main.h>
#include <navigation.h>
#include <analyze_horizon.h>

/* ======================================= */

static _Bool empty_lake    = 0;
static _Bool lake_scanned  = 0;
static _Bool ready_to_save = 0;
static int mode            = MOTIONLESS_STATE;

//simple PI regulator implementation
int16_t crawl_to_swimmer(float distance, float goal){

	volatile float err = 0, speed = 0;
	static float sum_error = 0;

	err = distance - goal;

	//disables the PI regulator if the error is to small
	if(fabs(err) < ERROR_THRESHOLD){
		return FALSE;
	}

	sum_error += err;

	//avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}
	else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * err + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waGoToSwimmer, 512);
static THD_FUNCTION(GoToSwimmer, arg) {

	chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed = 0, speed_correction = 0, tmp = 0;

	while(1){
		time = chVTGetSystemTime();

		if((mode==BEGIN_RESCUE_STATE) && (!ready_to_save)){
			//computes the speed to give to the motors
			//distance_cm is modified by the image processing thread

			speed = crawl_to_swimmer(get_distance_cm(), GOAL_DISTANCE);

			if(get_distance_cm() <= GOAL_DISTANCE){
				speed = 0;
			}

			wait_im_ready();
			tmp = get_swimmer_position();

			//computes a correction factor to let the robot rotate to be in front of the line
			speed_correction = (tmp - (IMAGE_BUFFER_SIZE/2));

			//if the swimmer is nearly in front of the camera, don't rotate
			if(abs(speed_correction) < ROTATION_THRESHOLD){
				speed_correction = 0;
			}

			//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(speed - ROT_KP * speed_correction);
			left_motor_set_speed(speed + ROT_KP * speed_correction);

			// when close enough and aligned, sprint using the IR sensors until close enough to catch the ball
			if((speed_correction == FALSE) && (get_distance_cm() <= (GOAL_DISTANCE + MARGIN)))
			{
				while (get_prox(IR1) < BUOY_SIZE){
					right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
					left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
				}
				halt_robot();
				ready_to_save = TRUE;
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

    int swimmer_found = 0;
    systime_t time;

    halt_robot();
	wait_im_ready();
	search_left_shore();
    lake_scanned = 0;

    while(1){
    	time = chVTGetSystemTime();
    	if((mode==SEARCH_SWIMMER_STATE) && (!lake_scanned))
    	{
    		wait_im_ready();
    		search_left_shore();

    		while (!get_left_shore()){ // Turns left until left shore in sight
    			wait_im_ready();
    			swimmer_found = 0;
    			right_motor_set_speed(+ MOTOR_SPEED_LIMIT/12);
    			left_motor_set_speed(- MOTOR_SPEED_LIMIT/12);
    		}

    		halt_robot();
    		wait_im_ready();

			if(swimmer_found && get_left_shore()){ //Musn't detect swimmers on beach
				if((get_swimmer_position() <= get_left_shore_position()) ||
					(abs(get_swimmer_position() - get_left_shore_position()) < TOO_CLOSE)){
					swimmer_found = 0;
				}
			}

			search_right_shore();

			/* Turns right until finds a swimmer or the right shore
			 * Stops as soon as the right shore line enters its sight
			 */
			while((!swimmer_found) && (!get_right_shore())){
				wait_im_ready();
				right_motor_set_speed(- MOTOR_SPEED_LIMIT/12);
				left_motor_set_speed(+ MOTOR_SPEED_LIMIT/12);
				swimmer_found = get_swimmer_width();
			}

			if(swimmer_found && get_right_shore()){
				if((get_swimmer_position() >= get_right_shore_position())  ||
							(abs(get_swimmer_position() - get_left_shore_position()) < TOO_CLOSE)){
					swimmer_found = 0;
				}
			}

			halt_robot();
			clear_shore();

			if (swimmer_found){
				empty_lake = 0;
			}

			if (!swimmer_found){
				empty_lake = 1;
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

void switch_to_search_swimmer(void){
	mode = SEARCH_SWIMMER_STATE;
}

void switch_to_go_to_swimmer(void){
	mode = BEGIN_RESCUE_STATE;
}

void init_before_switch(void){
	mode = MOTIONLESS_STATE; // assign a value out of switch to make sure not to come back to another case
}

/* ================================================
 * Simple navigation functions*/

void turn_left(int turn_count, int div){

	int initial_count = left_motor_get_pos();

	halt_robot();
	while (abs(left_motor_get_pos() - initial_count) < turn_count){
		right_motor_set_speed(+MOTOR_SPEED_LIMIT/div);
		left_motor_set_speed(-MOTOR_SPEED_LIMIT/div);
	}
	halt_robot();
}

void turn_right(int turn_count, int div){

	int initial_count = left_motor_get_pos();

	halt_robot();
	while (abs(left_motor_get_pos() - initial_count) < turn_count){
		right_motor_set_speed(-MOTOR_SPEED_LIMIT/div);
		left_motor_set_speed(+MOTOR_SPEED_LIMIT/div);
	}
	halt_robot();
}

void go_straight(float distance){

	int initial_count = left_motor_get_pos();

	halt_robot();
	while (abs(left_motor_get_pos() - initial_count) < distance){
		right_motor_set_speed(MOTOR_SPEED_LIMIT/4);
		left_motor_set_speed(MOTOR_SPEED_LIMIT/4);
	}
	halt_robot();
}

void clear_lake (void){
	lake_scanned = 0;
}

void bring_swimmer_to_beach(void){

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

	turn_left(HALF_TURN_COUNT, 4);//capture the swimmer with de buoy

	while(get_prox(IR8) > IR_THRESHOLD) {
		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	}

    do{
    	  right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
		  left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
		  messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    } while (get_prox(IR8) < IR_THRESHOLD);

   	turn_right(HALF_TURN_COUNT, 8); //liberates the swimmer
   	go_straight(MOVE_AWAY);

}

void halt_robot(void){
	right_motor_set_speed(FALSE);
	left_motor_set_speed(FALSE);
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
