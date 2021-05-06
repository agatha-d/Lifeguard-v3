#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	volatile float error = 0;
	volatile float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
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

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    volatile int16_t speed = 0;
    volatile int16_t speed_correction = 0;

    static float sum_rot_error = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        //speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE); pour test rot seulement speed = 0
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

		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction - ROTATION_INTEGR*sum_rot_error);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction + ROTATION_INTEGR*sum_rot_error);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

void turn_around_left(int turn_count){

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

void turn_around_right(int turn_count){

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

void search_for_swimmers(uint8_t *buffer_blue, uint8_t *buffer_green){

	int facing_the_shore = 0;

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	while(!facing_the_shore){//facing the lake = flanc montant vert/bleu

		right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
		left_motor_set_speed(-MOTOR_SPEED_LIMIT/2);

		facing_the_shore = check_if_shore(buffer_blue, buffer_green); // = flanc
	}

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	//maintenant le robot est positionné correctement

	//commencer la detection de balles rouges ici


}

_Bool check_if_shore(uint8_t *buffer_blue, uint8_t *buffer_green)
{
	_Bool shore = 0;
	uint16_t begin_blue = rising_slope(buffer_blue);

	if (begin_blue)//flanc montant de bleu
	{
		uint16_t end_green = falling_slope(buffer_green);

		if(end_green)//flanc montant de vert près du flanc montant de bleu
		{
			shore = 1;
			//enable_front_led();
		}
	}
	return shore;
}

