/* File :				Leman_Lifeguard
 *
 * Description			This project, done as part of the embedded systems and robotics course,
 * 						simulates a lifeguard (the e-puck) fetching swimmers in peril (red balls)
 * 						in Lake Leman and brings them back to the beach.
 *
 * 						The main fucntion controls the different steps of the rescue.
 *
 * Authors : 			Agatha Duranceau and Roxane M?rat
 * Last modified : 		05/16/2021 at 9pm
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

/* ======================================= */

//constants for the different parts of the project
#define IMAGE_BUFFER_SIZE		640
#define IMAGE_VERTICAL_SIZE		480
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define PXTOCM					2500.0f //experimental value depending on swimmer size
#define GOAL_DISTANCE 			5.0f
#define MAX_DISTANCE 			25.0f
#define IR8						7
#define TRUE					1
#define FALSE					0

//constants for the four states
#define ANALYZING 				0
#define BEGIN_RESCUE 			1
#define FINISH_RESCUE 			2
#define VICTORY					3

/* ======================================= */

/** Robot wide IPC bus. */
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
