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
#define IR_THRESHOLD			78
#define IR8						7
#define HALF_TURN_COUNT			660 // For 180° turns
#define MOVE_AWAY				700

/* ======================================= */

//Mode of the final state machine:
#define SEARCH_SWIMMER_STATE	0
#define BEGIN_RESCUE_STATE		1
#define MOTIONLESS_STATE		5//2 plutot non??


/* ======================================= */

// Searches for a swimmer to save
void search_swimmer_start(void);

// Start the PI regulator thread to go to swimmer
void go_to_swimmer_start(void);

//Navigation functions
void turn_right(int turn_count, int div);
void turn_left(int turn_count, int div);
void go_straight(float distance);
void bring_swimmer_to_beach(void);
int16_t crawl_to_swimmer(float distance, float goal);

/**
* @brief   		Stops the robot
*/
void halt_robot(void);

// Return static variables
/**
* @brief   		Returns the state of the lake
* @return 		Last state (empty or not empty) of the lake
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
_Bool get_ready_to_save(void);
uint16_t get_left_shore_position(void);

// Enable selection of thread to run
void init_before_switch(void);
void switch_to_search_swimmer(void);
void switch_to_go_to_swimmer(void);

void clear_lake (void);
void clear_ready_to_save(void);

#endif /* PI_REGULATOR_H */
