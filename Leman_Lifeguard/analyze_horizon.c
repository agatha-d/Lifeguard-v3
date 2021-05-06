#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <analyze_horizon.h>


static float distance_cm = 0;
static uint16_t swimmer_position = IMAGE_BUFFER_SIZE/2;	//middle

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the swimmer's width extracted from the image buffer given
 *  Returns 0 if Swimmer not found
 */
uint16_t extract_swimmer_width(uint8_t *buffer){ //ajout buffer blue ?

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, not_swimmer = 0, swimmer_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}

	mean /= IMAGE_BUFFER_SIZE;

	do{
		not_swimmer = 0;

		//search for a begin: rise in color intensity
		while(stop == 0 && (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if((buffer[i] > mean) && (buffer[i-WIDTH_SLOPE] < mean) ){ // && (buffer_blue[i] < mean_blue) && (buffer_blue[i-WIDTH_SLOPE] > mean_blue)){
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}

		//if a begin was found, search for an end : fall of color intensity
		//if begin = 0 : no begin

		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if((buffer[i] > mean)  && (buffer[i+WIDTH_SLOPE] < mean)){ //&& (buffer_blue[i] < mean_blue) && (buffer_blue[i+WIDTH_SLOPE] > mean_blue)){
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        swimmer_not_found = 1;
		    }
		}
		else {//if no begin was found
		    swimmer_not_found = 1;
		}

		//if a swimmer too small has been detected, continues the search
		if(!swimmer_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			not_swimmer = 1;
		}
	}while(not_swimmer);

	if(swimmer_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		swimmer_position = (begin + end)/2; //gives the swimmer position.
		//chprintf((BaseSequentialStream *) &SDU1, "position : %d\n", swimmer_position);
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width; // width = 0, not a swimmer
	}
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons), choix ligne ???? semple voir le sol 15 cm devant lui pour l'instant => prendre plus haut
	po8030_advanced_config(FORMAT_RGB565, 0, 7, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1); // voir si 10 assez b
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
	uint16_t SwimmerWidth = 7;


	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();


		// attention : 16 bits mais format big endian : adresse de poids fort donnée
		 //img buffer pointer : cases de 8 bits, 1 pixel =  couleurs codées sur 16 bits
		// donc pixel suivant = buffer [i+2]

		// Attention décalage : début aligné entre les trois (MSB) => meme valeur max pour les trois couleurs

		//Extracts RGB intensity values for each pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

			imr[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8) >> 2;
			img[i/2] = (((uint8_t)img_buff_ptr[i]&0x07) << 3) | (((uint8_t)img_buff_ptr[i+1]&0xE0) >>5);
			imb[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F) << 1;

			//Values main template : => plus grande intensité, peut être mieux ?
			//imr[i/2] = (int)img_buff_ptr[i]&0xF8;
			//img[i/2] = (int)(img_buff_ptr[i]&0x07)<<5 | (img_buff_ptr[i+1]&0xE0)>>3;
			//imb[i/2] = (int)(img_buff_ptr[i+1]&0x1F)<<3;

			im_diff[i/2] = abs(2*imr[i/2] - imb[i/2] -img[i/2]); //- img[i/2]
		}


		//search for a swimmer in the image and gets its width in pixels
		SwimmerWidth = extract_swimmer_width(im_diff);

		//converts the width into a distance between the robot and the camera
		if(SwimmerWidth){
			distance_cm = PXTOCM/SwimmerWidth; // modifier PXTOCM par rapport à la taille des balles
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(im_diff, IMAGE_BUFFER_SIZE); // ausis flancs montants de vert et bleu quand voit du rouge :(
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
