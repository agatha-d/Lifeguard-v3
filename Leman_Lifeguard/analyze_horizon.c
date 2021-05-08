#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include<leds.h>

#include <analyze_horizon.h>


static float distance_cm = 0;
static uint16_t swimmer_position = IMAGE_BUFFER_SIZE/2;	//middle

static int swimmer = 0;

static uint16_t width = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


/*
 *Returns the swimmer's width extracted from the image buffer given
 *Returns 0 if Swimmer not found
 */
uint16_t extract_swimmer_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0;
	uint32_t mean = 0;
	int out = 0, end_of_buffer = 0;
	static uint16_t last_width = PXTOCM/GOAL_DISTANCE; //utile?


	for(i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){//average
		mean += buffer[i];
	}

	i=0;

	mean /= IMAGE_BUFFER_SIZE;

	do{

		//swimmer = 1; ???

		while(stop == 0 && (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))) //search for a begin: rise in color intensity
		{

			 if((buffer[i] > mean) && (buffer[i-WIDTH_SLOPE] < mean)) //si flanc montant
			 {
				 begin = i;
				 stop = 1;

			 }
			 i++;
		}
		if(!begin)//abscence flanc montant ->sortie de do...while
		{
			swimmer = 0;
			end_of_buffer = 1;
		}

		//slope at least be WIDTH_SLOPE wide and is compared to the mean of the image
		if((buffer[i] > mean) && (buffer[i-WIDTH_SLOPE] < mean) ){
			begin = i;
		    stop = 1;

		}
		i++;

		//if ((i <= (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) && begin)
		if (begin) { //if a begin was found, search for an end : fall of color intensity

		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		    	if((buffer[i] > mean)  && (buffer[i+WIDTH_SLOPE] < mean)) //si flanc descendant
		        {
		        	end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if (i > IMAGE_BUFFER_SIZE || !end) ->a quoi sert de mettre ces 2 conditions?
		    if (!end) //if an end was not found ->sortie de la boucle
		    {
		        //swimmer_not_found = 1;
		        swimmer = 0;
		    }
		}

		if(end && (end-begin) < MIN_LINE_WIDTH){//if a swimmer too small has been detected, continues the search
		//if(!swimmer_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			swimmer = 0;
		}

		if(i>=IMAGE_BUFFER_SIZE)
		{
			end_of_buffer = 1;
			swimmer = 0;
		}

		if(!swimmer)
		{
			if(end_of_buffer){
				out = 1;
			}
			if(!end_of_buffer){
				out = 0;
			}
		}

	} while(!out);


	if(!swimmer){
		clear_leds();
		set_led(LED3, 10);;
		begin = 0;
		end = 0;
		//width = last_width;
		width =0;
	}

	if(swimmer){
		clear_leds();
		set_front_led(1);
		//last_width = width = (end - begin);
		width = (end - begin);
		swimmer_position = (begin + end)/2; //gives the swimmer position.
	}

	//chprintf((BaseSequentialStream *) &SDU1, "position : %d\n", swimmer_position);


	//sets a maximum width or returns the measured width ->à verifier
	if(width){
		if((PXTOCM/width) > MAX_DISTANCE){
			return PXTOCM/MAX_DISTANCE;
		}
	}
	else{
		return width; // width = 0, not a swimmer
	}
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons), choix ligne ???? semple voir le sol 15 cm devant lui pour l'instant => prendre plus haut
	po8030_advanced_config(FORMAT_RGB565, 0, 170, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1); // voir si 10 assez b
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


static THD_WORKING_AREA(waProcessImage, 4096);
static THD_FUNCTION(ProcessImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint8_t *img_buff_ptr;
	uint8_t imr[IMAGE_BUFFER_SIZE] = {0}; // store red values
	uint8_t img[IMAGE_BUFFER_SIZE] = {0}; // store green values
	uint8_t imb[IMAGE_BUFFER_SIZE] = {0}; // store blue values
	uint8_t im_diff[IMAGE_BUFFER_SIZE] = {0}; // red - blue - green
	uint8_t im_diff_smooth[IMAGE_BUFFER_SIZE] = {0};
	int8_t tmp = 0;
	uint16_t SwimmerWidth = 7;

	bool send_to_computer = true;

	while(1){
		//waits until an image has been captured
		chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		/* Separate RGB values for each pixel
		 * The MSB for the three colors are aligned on the bit 6
		 * Big endian format
		 * img_buff_pointer of size 8, corresponds to 1 pixel RGB values
		 * Next pixel = img_buff_pointer[i+2]
		 */

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

			imr[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8) >> 2;
			img[i/2] = (((uint8_t)img_buff_ptr[i]&0x07) << 3) | (((uint8_t)img_buff_ptr[i+1]&0xE0) >>5);
			imb[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F) << 1;

			tmp = 2*imr[i/2] - imb[i/2] -img[i/2];//abs(2*imr[i/2] - imb[i/2] -img[i/2]); // Substract blue and green values in order to cancel red in other areas than swimmer
			if (tmp <4){ // error message says it's always false
				tmp = 0;
			}
			im_diff[i/2] = tmp;
		}

		//Smoothing the signal to reduce noise with moving average
		int n = 5;
		for(int k = 0; k<IMAGE_BUFFER_SIZE-n+1 ; k++)
		{
			im_diff_smooth[k] = (im_diff[k]+im_diff[k+1]+im_diff[k+2]
									+ im_diff[k+3] + im_diff[k+4]
									+ im_diff[k+5] + im_diff[k+6]
									+ im_diff[k+7] + im_diff[k+8]
									+ im_diff[k+9] /*+ im_diff[k+10]
									+ im_diff[k+11]+ im_diff[k+12]
									+ im_diff[k+13]+ im_diff[k+14]
									+ im_diff[k+15]+ im_diff[k+16]
									+ im_diff[k+17]+ im_diff[k+18]
									+ im_diff[k+19]+ im_diff[k+20]
									+ im_diff[k+21]+ im_diff[k+22]
									+ im_diff[k+23]+ im_diff[k+24]
									+ im_diff[k+25]+ im_diff[k+26]
									+ im_diff[k+27]+ im_diff[k+28]
									+ im_diff[k+29])*//10);//moving_average (im_diff, k, n);
		}

		for(int k = IMAGE_BUFFER_SIZE-n+1; k<IMAGE_BUFFER_SIZE ; k++)
		{
					im_diff_smooth[k] = im_diff[k];
		}


		//search for a swimmer in the image and gets its width in pixels
		SwimmerWidth = extract_swimmer_width(im_diff);

		//converts the width into a distance between the robot and the camera
		if(SwimmerWidth){
			//set_led(LED3, 1);
			distance_cm = PXTOCM/SwimmerWidth; // modifier PXTOCM par rapport à la taille des balles

		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(im_diff_smooth, IMAGE_BUFFER_SIZE); //Optional, only for visialisation
		}
		//invert the bool
		send_to_computer = !send_to_computer;
	    }
	}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_swimmer_position(void){
	return swimmer_position;
}

uint16_t get_swimmer_width(void){
	return width;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	}

//   Analysis of the environnement
/* ====================================================================== */

/*
int check_sea_or_beach(uint16_t position, uint16_t size, uint8_t *buffer_b, uint8_t *buffer_g){

	int zone = 0; // water = 1, beach = 0

	// Prend une photo verticale au centre de la balle
	uint16_t vert_array = size + 50;
	if (vert_array > IMAGE_VERTICAL_SIZE) {
		vert_array = IMAGE_VERTICAL_SIZE;
	}

	po8030_advanced_config(FORMAT_RGB565, position, 1, 2 , vert_array, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    //starts a capture
	dcmi_capture_start();
	//waits for the capture to be done
	wait_image_ready();

	//détection flanc descendant et montant de bleu
	//détection flanc de vert

	uint8_t v_blue[vert_array] = {0}; // store blue values
	uint8_t v_green[vert_array] = {0}; // store green values

	//Extracts RGB intensity values for each pixels
	for(uint16_t i = 0 ; i < (2 *vert_array) ; i+=2){
			v_green[i/2] = (((uint8_t)img_buff_ptr[i]&0x07) << 3) | (((uint8_t)img_buff_ptr[i+1]&0xE0) >>5);
			v_blue[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F) << 1;
		}

	return zone;
}
*/

_Bool check_left_shore(uint8_t *buffer_blue, uint8_t *buffer_green)
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

_Bool check_right_shore(uint8_t *buffer_blue, uint8_t *buffer_green)
{
	_Bool shore = 0;
	uint16_t end_blue = falling_slope(buffer_blue);

	if (end_blue)
	{
		uint16_t begin_green = rising_slope(buffer_green);

		if(begin_green)//rising slope of green after falling slope of ble
		{
			shore = 1;
			//enable_front_led();
		}
	}
	return shore;
}

//	General geometric functions
/* =================================================*/

uint16_t rising_slope(uint8_t *buffer){

	uint32_t mean = average_buffer(buffer);

	uint16_t begin =0;
	uint16_t stop =0;
	uint16_t i =0;

	while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
	{
		//the slope must at least be WIDTH_SLOPE wide and is compared to the mean of the image
	    if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
	    {
	        begin = i;
	        stop = 1;
	    }
	    i++;
	}

	return begin;
}


uint16_t falling_slope(uint8_t *buffer){

	uint32_t mean = average_buffer(buffer);

	uint16_t end = 0;
	uint16_t stop =0;
	uint16_t i =0;

	while(stop == 0 && i < IMAGE_BUFFER_SIZE)
	{
		if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		{
		   end = i;
		   stop = 1;
		}
		i++;
	}

	return end;
}


uint32_t average_buffer(uint8_t *buffer){

	uint32_t mean = 0;

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	return mean;

}
