#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define IMAGE_VERTICAL_SIZE		480
#define WIDTH_SLOPE				5 // original : 5
#define MIN_LINE_WIDTH			40 // original : 40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define ROTATION_INTEGR			0.005f
#define PXTOCM					2703.8f //experimental value depending on swimmer size
#define GOAL_DISTANCE 			5.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.2f	//[cm] because of the noise of the camera (au départ 0.1)
#define KP						15.0f // 15 : pas très rapide mais vise bien, encore beaucoup d'oscillation rotation pr coeff = 2, thershold 10 (ah non en fait tourne en rond chelou)
#define KI 						0.001f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI) // normalement /KI, simple test
#define MAX_ROT_ERROR			(MOTOR_SPEED_LIMIT/ROTATION_INTEGR/100)
#define HALF_TURN_COUNT			660

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
