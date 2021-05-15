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
#define PXTOCM					2500.0f ////2500.0f //2703.8f //experimental value depending on swimmer size
#define GOAL_DISTANCE 			5.0f   //5.0f
#define MAX_DISTANCE 			25.0f


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
