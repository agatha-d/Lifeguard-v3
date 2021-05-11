#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

/* Define PI controller coefficients for rotation
 * and going straight here in order to be easily tuned
 */
#define ROTATION_THRESHOLD		10
#define ROT_KP					2 //2
#define ROT_KI					0.005f //if integral term only, not sure yet // previous value without condition on blue : other idea :  + rouge - bleu - vert
#define MAX_ROT_ERROR			(MOTOR_SPEED_LIMIT/ROT_KI/200) // /100
#define ERROR_THRESHOLD			0.2f	//[cm] because of the noise of the camera (au départ 0.1)
#define KP						15.0f // 15 : pas très rapide mais vise bien, encore beaucoup d'oscillation rotation pr coeff = 2, thershold 10 (ah non en fait tourne en rond chelou)
#define KI 						0.001f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#define HALF_TURN_COUNT			660 // For 180° turns


// Functions :

// Searches for a swimmer to save
void search_swimmer_start(void);

_Bool get_empty_lake(void);

_Bool get_analyse(void);

// Start the PI regulator thread to go to swimmer
void go_to_swimmer_start(void);

// Simple navigation functions
void turn_left(int turn_count);
void turn_right(int turn_count);
void go_straight(float distance);

void switch_to_one(void);


#endif /* PI_REGULATOR_H */
