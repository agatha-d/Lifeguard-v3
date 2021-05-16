/* File :				analyze-horizon, part of project Leman_Lifeguard
 *
 * Description			Communication with the camera and interpretation of images
 * 						Detection of swimmers and borders of the lake
 *
 * Authors : 			Agatha Duranceau and Roxane Mérat
 * Last modified : 		05/16/2021 at 9pm
 *
 * Adapted from TP4_CamReg, process_image.c code given in the MICRO-315 class
 */

#include "ch.h"
#include "hal.h"

#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>
#include <camera/po8030.h>
#include <leds.h>

#include <main.h>
#include <analyze_horizon.h>

/* Declaration of static variables */ //REDUIRE LEUR NOMBRE
/* =========================================================================== */

static float distance_cm             = 0;

static uint16_t swimmer_position     = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t left_shore_position  = 0;
static uint16_t right_shore_position = 0;
static uint16_t width                = 0;

static _Bool swimmer                 = FALSE;
static _Bool left_shore              = FALSE;
static _Bool right_shore             = FALSE;
static _Bool analyzing               = TRUE;

static int shore_to_search           = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/* =========================================================================== */

void wait_im_ready(void){
	chBSemWait(&image_ready_sem);
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 228 + 229 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 213, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 8192);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint8_t *img_buff_ptr;
	uint8_t imr[IMAGE_BUFFER_SIZE]                	= {0}; // store red values
	uint8_t img[IMAGE_BUFFER_SIZE] 				  	= {0}; // store green values
	uint8_t imb[IMAGE_BUFFER_SIZE] 				  	= {0}; // store blue values
	uint8_t im_diff_red[IMAGE_BUFFER_SIZE]        	= {0}; // store 2*red - blue - green
	uint8_t im_diff_red_smooth[IMAGE_BUFFER_SIZE] 	= {0};
	int16_t tmp										= 0;
	uint16_t SwimmerWidth 							= 0;

	//_Bool send_to_computer = TRUE; // Optional : for calibrating camera

	while(1){

		//waits until an image has been captured
		chBSemWait(&image_ready_sem);

		if(analyzing){

			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();

			/* Separate RGB values for each pixel
			 * The MSB for the three colors are aligned on the bit 6
			 * Big endian format
			 * img_buff_pointer of size 8, corresponds to 1 pixel RGB values
			 * Next pixel = img_buff_pointer[i+2]
			 */
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2) {
				imr[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8)   >> 2;
				img[i/2] = (((uint8_t)img_buff_ptr[i]&0x07)  << 3) | (((uint8_t)img_buff_ptr[i+1]&0xE0) >>5);
				imb[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F) << 1;

				// Creation of the buffer to recognize swimmers
				im_diff_red[i/2] = difference(imr, imb, img, i/2, RED_NOISE_THRESHOLD);
			}

			//Smoothing the signal to reduce noise with moving average
			for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE - MOVING_AVERAGE_WINDOW ; i++) {
				tmp = 0;
				for(int j = 0 ; j<MOVING_AVERAGE_WINDOW ; j++) {
					tmp = tmp + im_diff_red[i+j];
				}
				im_diff_red_smooth[i] = tmp/(MOVING_AVERAGE_WINDOW);
			}

			for(uint16_t i = IMAGE_BUFFER_SIZE-MOVING_AVERAGE_WINDOW; i<IMAGE_BUFFER_SIZE ; i++) {
				im_diff_red_smooth[i] = im_diff_red[i]; // extremities of the buffer
			}

			for(uint16_t i = 0; i<IMAGE_BUFFER_SIZE ; i++) { // Transform swimmers into neat rectangles
				if(im_diff_red_smooth[i] > RED_NOISE_THRESHOLD) {
					im_diff_red_smooth[i] = PRESENCE_OF_RED;
				}
			}

			SwimmerWidth = extract_swimmer_width(im_diff_red_smooth);

			if(shore_to_search == LEFT_SHORE) {
				left_shore =  extract_left_shore(imb, img, imr);
			} else {
				left_shore = FALSE;
			}

			if(shore_to_search == RIGHT_SHORE) {
				right_shore = extract_right_shore(imb, img, imr);
			} else {
				right_shore = FALSE;
			}

			//converts the width into a distance between the robot and the camera
			if(SwimmerWidth) {
				distance_cm = PXTOCM/SwimmerWidth;
			}

			// Optional : use to calibrate camera settings
			/*if(send_to_computer){
				//sends to the computer the image
				SendUint8ToComputer(im_diff_red_smooth, IMAGE_BUFFER_SIZE);
			}
			//invert the bool : only shows 1 image out of 2
			send_to_computer = !send_to_computer;*/
		}
	}
}

//	Start of the threads
/* =================================================*/

void process_image_start(void) {
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
}

void capture_image_start(void) {
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}


//	Extract information from image
/* =================================================*/

uint16_t extract_swimmer_width(uint8_t *buffer) {

	uint16_t 	i    = 0, 			begin = 0, 		end           = 0;
	_Bool 		stop    = FALSE,	out   = FALSE, 	end_of_buffer = FALSE;
	uint32_t 	mean = 0;

	mean = average_buffer(buffer);

	do{
		while(stop == FALSE && (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))) { //search for a begin = rise in color intensity
			 if((buffer[i] > (mean+1)) && (buffer[i-WIDTH_SLOPE] <= mean)) {
				 begin = i;
				 stop  = TRUE;
			 }
			 i++;
		}
		if(!begin) { //if no begin was found, end of search
			swimmer = FALSE;
			end_of_buffer = TRUE;
		}

		if (begin) { //if a begin was found, search for an end = fall of color intensity

		    stop = FALSE;
		    
		    while(stop == FALSE && i < IMAGE_BUFFER_SIZE) {
		    	if((buffer[i] > mean)  && (buffer[i+WIDTH_SLOPE] <= mean)) {
		        	end     = i;
		            stop    = TRUE;
		            swimmer = TRUE;
		        }
		        i++;
		    }

		    if (!end) { //if no end was not found, no swimmer
		        swimmer = FALSE;
		    }
		}

		if(end && ((end-begin) < MIN_LINE_WIDTH)) {//if a swimmer too small has been detected, ignore it
			i       = end;
			begin   = 0;
			end     = 0;
			stop    = FALSE;
			swimmer = FALSE;
		}

		if(end && ((end-begin) > MIN_LINE_WIDTH)) {
			out = TRUE;
		}

		if(i>=IMAGE_BUFFER_SIZE) {
			end_of_buffer = TRUE;
			swimmer = FALSE;
		}

		if(!swimmer) {
			if(end_of_buffer) {
				out = TRUE;
			}
			if(!end_of_buffer) {
				out = FALSE;
			}
		}

	} while(!out);

	if(!swimmer) {
		begin = 0;
		end   = 0;
		width = 0;
	}

	if(swimmer) {
		width = (end - begin);
		swimmer_position = (begin + end)/2;
	}

	return width; // width = 0 if no swimmer
}


uint32_t average_buffer(uint8_t *buffer) {

	uint32_t mean = 0;

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++) {
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	return mean;
}

int8_t difference(uint8_t *buffer_diff, uint8_t *buffer1, uint8_t *buffer2, int i, int threshold) {

	int8_t tmp = 0;

	tmp = 2*buffer_diff[i] - buffer1[i] - buffer2[i];
	if (tmp <threshold) {
		tmp = 0;
	}

	return tmp;
}

_Bool extract_left_shore(uint8_t *buffer_blue, uint8_t *buffer_green, uint8_t *buffer_red) {

	_Bool stop 							= FALSE;
	uint16_t i 							= 0;
	uint8_t im_diff[IMAGE_BUFFER_SIZE] 	= {0};

	for(i = 0 ; i < IMAGE_BUFFER_SIZE ; i++) {
		im_diff[i] = difference(buffer_green, buffer_blue, buffer_red, i, GREEN_NOISE_THRESHOLD);
	}

	left_shore = FALSE;
	stop = FALSE;

	i = SHORE_WIDTH_SLOPE;

	while((i < IMAGE_BUFFER_SIZE)&&(!stop))	{
		if((im_diff[i]==0)&& (im_diff[i-SHORE_WIDTH_SLOPE]>0)) {
			stop = TRUE;
			left_shore = TRUE;
			left_shore_position = i;
		}
		i++;
	}
	return left_shore ;
}

_Bool extract_right_shore(uint8_t *buffer_blue, uint8_t *buffer_green, uint8_t *buffer_red) {

	_Bool stop 							= FALSE;
	uint16_t i 							= 0;
	uint8_t im_diff[IMAGE_BUFFER_SIZE] 	= {0};

	for(i = 0 ; i < IMAGE_BUFFER_SIZE ; i++) {
		im_diff[i] = difference(buffer_green, buffer_blue, buffer_red, i, GREEN_NOISE_THRESHOLD);
	}

	right_shore = FALSE;
	stop = FALSE;

	i = SHORE_WIDTH_SLOPE;

	while((i < IMAGE_BUFFER_SIZE)&&(!stop)) {
		if((im_diff[i]>0)&& (im_diff[i-SHORE_WIDTH_SLOPE]==0)) {
			stop = TRUE;
			right_shore = TRUE;
			right_shore_position = i;
		}
		i++;
	}
	return right_shore ;
}

void reset_shore(void) {
	right_shore 			= FALSE;
	left_shore  			= FALSE;
	right_shore_position 	= 0;
	left_shore_position  	= 0;
}

void clear_shore(void) {
	shore_to_search = FALSE;
}

void search_left_shore(void) {
	shore_to_search = LEFT_SHORE;
}

void search_right_shore(void) {
	shore_to_search = RIGHT_SHORE;
}

void start_analyzing (void) {
	analyzing = TRUE;
}

void stop_analyzing (void) {
	analyzing = FALSE;
}

//	Return static variables
/* =================================================*/

float get_distance_cm(void) {
	return distance_cm;
}

uint16_t get_swimmer_position(void) {
	return swimmer_position;
}

uint16_t get_swimmer_width(void) {
	return width;
}

uint16_t get_left_shore_position(void) {
	return left_shore_position;
}

uint16_t get_right_shore_position(void) {
	return left_shore_position;
}

_Bool get_left_shore(void) {
	return left_shore;
}

_Bool get_right_shore(void) {
	return right_shore;
}
