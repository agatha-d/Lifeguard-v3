#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

/* Define PI controller coefficients for rotation
 * and going straight here in order to be easily tuned
 */
#define KP						18.0f
#define KI 						0.001f
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define ROTATION_THRESHOLD		10
#define ROT_KP					0.5
#define ERROR_THRESHOLD			1.0f
#define IR_THRESHOLD			79//78
#define IR8						7
#define IR1						0
#define HALF_TURN_COUNT			660 // For 180° turns
#define MOVE_AWAY				700
#define TOO_CLOSE				50
#define BUOY_SIZE 				150
#define MARGIN 					8

/* ======================================= */

//Mode of the final state machine:
#define SEARCH_SWIMMER_STATE	0
#define BEGIN_RESCUE_STATE		1
#define MOTIONLESS_STATE		5//2 plutot non??


/* ======================================= */

/**
* @brief   			Initialize the thread that searches the swimmers
*/
void search_swimmer_start(void);

/**
* @brief   			Initialize the thread that make the e-puck navigate toward a detected swimmer
*/
void go_to_swimmer_start(void);

/**
* @brief   			Turn the robot to the right
* @param turn_count Turn by "turn_count" steps
* @param div    	Turns at the speed MOTOR_SPEED_LIMIT/div
*/
void turn_right(int turn_count, int div);

/**
* @brief   			Turn the robot to the left
* @param turn_count Turn by "turn_count" steps
* @param div    	Turns at the speed MOTOR_SPEED_LIMIT/div
*/
void turn_left(int turn_count, int div);

/**
* @brief   			Turn the robot to the right
* @param distance  	In steps
*/
void go_straight(float distance);

/**
* @brief   			Effects a left half turn to capture the swimmer, brings him near the beach
* 					until the walls are detected by the IR, liberates him by making a right half turn,
* 					then move away from him to better analyze it's environment
*/
void bring_swimmer_to_beach(void);

/**
* @brief   			Implements the PI to go to the ball
* @param distance	Calculated in process image, in cm;
* @param goal    	Constante of 5 cm
* @return 			Speed in step/s
*/
int16_t crawl_to_swimmer(float distance, float goal);

/**
* @brief   			Stops the robot
*/
void halt_robot(void);

/**
* @brief   			Returns the state of the lake
* @return 			Last state (empty or not empty) of the lake
*/
_Bool get_empty_lake(void);

/**
* @brief   		Returns the advancement of the scanning process of the lake
* @return 		Last state (scanned or not scanned) of the scanning process
*/
_Bool get_lake_scanned(void);

/**
* @brief   		Returns the state of the lake
* @return 		Last state (empty or not empty) of the lake
*/
_Bool get_empty_lake(void);

/**
* @brief   			Returns if the robot is ready to bring to swimmer to the beach
* @return 			TRUE if the robot is close enough to catch the swimmer with a 180° rotation
*/
_Bool get_ready_to_save(void);

/**
* @brief   			Change the mode of the robot to not search or go to the swimmer
*/
void init_before_switch(void);

/**
* @brief   			Change the mode of the robot to search the swimmer
*/
void switch_to_search_swimmer(void);

/**
* @brief   			Change the mode of the robot to go to the swimmer
*/
void switch_to_go_to_swimmer(void);

/**
* @brief   			Change the state of the scanning of the scanning of the lake : it's not completed
*/
void clear_lake (void);

/**
* @brief   			The robot isn't close anymore to a swimmer to save him
*/
void clear_ready_to_save(void);

#endif /* PI_REGULATOR_H */
