#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <chthreads.h>

#include <main.h>
#include <motors.h>
#include <leds.h>
#include <navigation.h>
#include <analyze_horizon.h>
#include <victory.h>

static _Bool empty_lake = 0;

static BSEMAPHORE_DECL(lake_analyzed_sem, TRUE);

//simple PI regulator implementation
int16_t crawl_to_swimmer(float distance, float goal){

	volatile float error = 0;
	volatile float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//to avoid permanent movement
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}
	else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error; //+ KI * sum_error;

    return (int16_t)speed;
}

// Must still add IR
static THD_WORKING_AREA(waGoToSwimmer, 256);
static THD_FUNCTION(GoToSwimmer, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    volatile int16_t speed = 0;
    volatile int16_t speed_correction = 0;

    static float sum_rot_error = 0;



    while(1){

    	chBSemWait(&lake_analyzed_sem);

        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        //speed = crawl_to_swimmer(get_distance_cm(), GOAL_DISTANCE); // pour test rot seulement speed = 0
        speed = 0;
        //computes a correction factor to let the robot rotate to be in front of the line
        volatile int16_t tmp = get_swimmer_position();
        speed_correction = (tmp - (IMAGE_BUFFER_SIZE/2));


        if(sum_rot_error > MAX_ROT_ERROR){
        	sum_rot_error = MAX_ROT_ERROR;
        	}else if(sum_rot_error < -MAX_ROT_ERROR){
        		sum_rot_error = -MAX_ROT_ERROR;
        	}

        //chprintf((BaseSequentialStream *)&SD3, "sum error : %f, speed correction : %d \n,", sum_rot_error, speed_correction);

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }
        sum_rot_error += speed_correction;

        //applies the speed from the PI regulator and the correction for the rotation

		right_motor_set_speed(speed - ROT_KP * speed_correction - ROT_KI*sum_rot_error);
		left_motor_set_speed(speed + ROT_KP * speed_correction + ROT_KI*sum_rot_error);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void go_to_swimmer_start(void){
	chThdCreateStatic(waGoToSwimmer, sizeof(waGoToSwimmer), NORMALPRIO, GoToSwimmer, NULL);
}

static THD_WORKING_AREA(waSearchSwimmer, 256); // Not tested yet
static THD_FUNCTION(SearchSwimmer, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

   // clear_leds();
    int turn_count = 0;
    int swimmer_found = 0;
    int initial_count = left_motor_get_pos();


set_front_led(0);

    while(1){
		time = chVTGetSystemTime();

			while((!swimmer_found) && (turn_count<=2*HALF_TURN_COUNT) && (!empty_lake)){

				//set_led(LED5, 10);
				right_motor_set_speed(-MOTOR_SPEED_LIMIT/6);
				left_motor_set_speed(+MOTOR_SPEED_LIMIT/6);
				turn_count = (left_motor_get_pos() - initial_count);
				swimmer_found = get_swimmer_width();
			}

		//if (turn_count >= (2*HALF_TURN_COUNT)){ // condition à modifier pou cherche uniquement côté eau
			//empty_lake = 1;
		//}

		right_motor_set_speed(0);
		left_motor_set_speed(0);

	   if (swimmer_found){
			empty_lake = 0;
			set_front_led(1);
	   }

	   if (!swimmer_found){
			empty_lake = 1;
			set_front_led(0);
		}

	   //chThdYield();
	   //chThdExit(0);

		//return swimmer_found; // ou alors ne retourne rien et play victory ici

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
		}

		right_motor_set_speed(0);
		left_motor_set_speed(0);

		if(get_swimmer_position || check_right_shore()){ // nécessaire d'avoir les deux ?
			set_target = 1; // is it possible for a thread to return a value ?
		}
		// If no swimmer has been found, all the swimmers are safe
		else {
			victory_start();
		}*/


	   chBSemSignal(&lake_analyzed_sem);

		//100Hz
		//chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

}

_Bool get_empty_lake(void){
	return empty_lake;
}


void search_swimmer_start(void){
	chThdCreateStatic(waSearchSwimmer, sizeof(waSearchSwimmer), NORMALPRIO, SearchSwimmer, NULL);
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

