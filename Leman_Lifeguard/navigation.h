#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

/* Define PI controller coefficients for rotation
 * and going straight here in order to be easily tuned
 */
#define ROTATION_THRESHOLD		10
#define ROT_KP					0.5 //2
#define ROT_KI					0.005f //if integral term only, not sure yet // previous value without condition on blue : other idea :  + rouge - bleu - vert
#define MAX_ROT_ERROR			(MOTOR_SPEED_LIMIT/ROT_KI/200) // /100
#define ERROR_THRESHOLD			0.2f	//[cm] because of the noise of the camera (au départ 0.1)
#define KP						15.0f // 15 : mais vise pas trop mal avec rot KP = 0.5, threshold = 10 mais avance un peu par accoups, et ne s'arrete pas
#define KI 						0.0002f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI) // trop grand ?

#define HALF_TURN_COUNT			660 // For 180° turns

// Functions :

// Searches for a swimmer to save
void search_swimmer_start(void);

// Start the PI regulator thread to go to swimmer
void go_to_swimmer_start(void);

// Simple navigation functions
void turn_left(int turn_count);
void turn_right(int turn_count);
void go_straight(float distance);

// Return static variables
_Bool get_empty_lake(void);
_Bool get_lake_scanned(void);
_Bool get_empty_lake(void);
_Bool get_ready_to_save(void);

// Enable selection of thread to run
void init_before_switch(void);
void switch_to_search_swimmer(void);
void switch_to_go_to_swimmer(void);



#endif /* PI_REGULATOR_H */
