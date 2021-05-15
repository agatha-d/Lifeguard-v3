#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

/* Define PI controller coefficients for rotation
 * and going straight here in order to be easily tuned
 */
#define KP						18.0f //17.0f // 15
#define KI 						0.001f	//must not be zero //0.001f
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define ROTATION_THRESHOLD		10
#define ROT_KP					0.5 //0.5
#define ERROR_THRESHOLD			1.0f

#define HALF_TURN_COUNT			660 // For 180� turns

// Functions :

// Searches for a swimmer to save
void search_swimmer_start(void);

// Start the PI regulator thread to go to swimmer
void go_to_swimmer_start(void);

// Simple navigation functions
void turn_right(int turn_count, int div);
void turn_left(int turn_count, int div);
void go_straight(float distance);

void clear_ready_to_save(void);

// Return static variables
_Bool get_empty_lake(void);
_Bool get_lake_scanned(void);
_Bool get_empty_lake(void);
_Bool get_ready_to_save(void);
uint16_t get_left_shore_position(void);
int get_step_to_turn(void);

// Enable selection of thread to run
void init_before_switch(void);
void switch_to_search_swimmer(void);
void switch_to_go_to_swimmer(void);

void clear_lake (void);



#endif /* PI_REGULATOR_H */
