#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>
#include<leds.h>

#include <analyze_horizon.h>


static float distance_cm = 0;
static uint16_t swimmer_position = IMAGE_BUFFER_SIZE/2;	//middle

static int swimmer = 0;

static int left_shore = 0;
static int right_shore = 0;

static uint16_t width = 0;

uint16_t left_shore_position = 0;
uint16_t right_shore_position = 0;

int shore_to_search=0; //=1 pour le shore left et 2 pour le shore right

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

void wait_im_ready(void){
	chBSemWait(&image_ready_sem);
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 170 + 170 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 250, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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

	uint8_t im_diff_red[IMAGE_BUFFER_SIZE] = {0}; // 2*red - blue - green
	uint8_t im_diff_red_smooth[IMAGE_BUFFER_SIZE] = {0};

	uint8_t im_diff_blue[IMAGE_BUFFER_SIZE] = {0};

	int8_t tmp = 0;
	uint16_t SwimmerWidth = 0; // 6, why not zero by default ???????

	_Bool send_to_computer = true;

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

			// Creation of the buffer to recognise swimmers
			tmp = 2*imr[i/2] - imb[i/2] -img[i/2]; // Substract blue and green values in order to cancel red in other areas than swimmer
			if (tmp <4){
				tmp = 0;
			}
			im_diff_red[i/2] = tmp;
		}

		//Smoothing the signal to reduce noise with moving average
		int n = 10;

		for(int k = 0; k<IMAGE_BUFFER_SIZE-n+1 ; k++)
		{
			im_diff_red_smooth[k] = (im_diff_red[k]+im_diff_red[k+1]+im_diff_red[k+2]
									+ im_diff_red[k+3] + im_diff_red[k+4]
									+ im_diff_red[k+5] + im_diff_red[k+6]
									+ im_diff_red[k+7] + im_diff_red[k+8]
									+ im_diff_red[k+9])/10;

			//im_diff_red_smooth[k] = smoothing(im_diff_red, k, n);
		}

		for(int k = IMAGE_BUFFER_SIZE-n+1; k<IMAGE_BUFFER_SIZE ; k++)
		{
					im_diff_red_smooth[k] = im_diff_red[k]; // values for the extremities of the buffer
		}

		//search for a swimmer in the image and gets its width in pixels
		if((shore_to_search == 0) || (shore_to_search == 2)){
			SwimmerWidth = extract_swimmer_width(im_diff_red_smooth);
		} else {
			SwimmerWidth = 0; // pour lintant test au cas où variables statiques pas réinitialisées correctement
		}

		if(shore_to_search == 1){
			left_shore = extract_shore(imb, img);
		} else {
			left_shore = 0;
		}

		if(shore_to_search == 2){
			right_shore = extract_right_shore(imb, img);
		} else {
			right_shore = 0;
		}

		//converts the width into a distance between the robot and the camera
		if(SwimmerWidth){
			distance_cm = PXTOCM/SwimmerWidth; // modifier PXTOCM à la taille des balles
		}

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

				tmp = img[i/2]- imb[i/2]; // Substract blue and green values in order to cancel red in other areas than swimmer
				if (tmp <1){
					tmp = 0;
				}
				im_diff_blue[i/2] = tmp;//= a un nb >4 quand dans le vert, à 0 dans le bleu
		}

		if(send_to_computer){

			//sends to the computer the image
			SendUint8ToComputer(im_diff_red_smooth, IMAGE_BUFFER_SIZE); //Ne semble plus fonctionner après l'ajout des différetnes threads
		}
		//invert the bool : only shows 1 image out of 2
		send_to_computer = !send_to_computer;
	}
}


/*
 *Returns the swimmer's width extracted from the image buffer given
 *Returns 0 if Swimmer not found
 */
uint16_t extract_swimmer_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0;
	uint32_t mean = 0;
	int out = 0, end_of_buffer = 0;
	//static uint16_t last_width = PXTOCM/GOAL_DISTANCE; //utile?

	for(i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){ //Performs an average
		mean += buffer[i];
	}

	i=0;

	mean /= IMAGE_BUFFER_SIZE;

	//mean = average_buffer(buffer); // what difference ??

	do{

		while(stop == 0 && (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))) //search for a begin = rise in color intensity
		{
			 if((buffer[i] > (mean+1)) && (buffer[i-WIDTH_SLOPE] <= mean)) // If rise
			 {
				 begin = i;
				 stop = 1;
			 }
			 i++;
		}
		if(!begin) //If no begin was found, get out of loop
		{
			swimmer = 0;
			end_of_buffer = 1;
		}

		if (begin) { //if a begin was found, search for an end = fall of color intensity

		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		    	if((buffer[i] > (mean+1))  && (buffer[i+WIDTH_SLOPE] <= mean)) // If fall
		        {
		        	end = i;
		            stop = 1;
		            swimmer = 1;
		        }
		        i++;
		    }

		    if (!end) //if no end was not found, get out of loop
		    {
		        //swimmer_not_found = 1;
		        swimmer = 0;
		    }
		}

		if(end && ((end-begin) < MIN_LINE_WIDTH)){ //if a swimmer too small has been detected, ignore & continues the search
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			swimmer = 0;
		}

		if(end && ((end-begin) > MIN_LINE_WIDTH)){
			out = 1;
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
		//set_front_led(0);
		//set_led(LED3, 10);
		begin = 0;
		end = 0;
		//width = last_width;
		width =0;
	}

	if(swimmer){

		//clear_leds();
		//set_front_led(1);
		//last_width = width = (end - begin);
		width = (end - begin);
		swimmer_position = (begin + end)/2; //gives the swimmer position.
	}


	return width; // width = 0 -> no swimmer
}



float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_swimmer_position(void){
	return swimmer_position;
}

uint16_t get_swimmer_width(void){
	//set_front_led(1);
	return width;
}

uint16_t get_left_shore_position(void){
	return left_shore_position;
}

int get_left_shore(void){
	return left_shore;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
}

void capture_image_start(void){
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
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

/*
 * Performs the average of the values of a buffer
 */
uint32_t average_buffer(uint8_t *buffer){

	uint32_t mean = 0;

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	return mean;
}

/*
 * Applies a moving average on a buffer
 * each cell of the buffer will be attributed the value of the average of the 5 neigbouring cells.
 * The aim is to smooth a signal and therefore reduce noise
 */
uint32_t smoothing(uint8_t *buffer, int k, int n){ // n'a as l'air de fonctionner quand on prend la fonction, pourqoiiiiii ?

	uint32_t im_diff_smooth = 0;

	for(int j = 0; j<n; j++)
	{
		im_diff_smooth += buffer[k+j];
	}
	im_diff_smooth = im_diff_smooth/n;

	return im_diff_smooth;

}

int8_t difference(uint8_t *buffer_diff, uint8_t *buffer1, uint8_t *buffer2, int i){

	int8_t tmp = 0;

	tmp = 2*buffer_diff[i/2] - buffer1[i/2] -buffer2[i/2]; // Substract buffer1 and buffer2 values in order to cancel red in other areas than swimmer
	if (tmp <4){ // error message says it's always false
		tmp = 0;
	}
	return tmp;
}


//à faire : changer pour qu'il faille juste échanger les buffer_blue et green dans l'appel de fonction
int extract_shore(uint8_t *buffer_blue, uint8_t *buffer_green){

	int8_t tmp = 0;
	int stop = 0;
	uint16_t i = 0;

	uint8_t im_diff[IMAGE_BUFFER_SIZE] = {0};


	for(i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){

		tmp = buffer_green[i]- buffer_blue[i];
		if (tmp <1){
			tmp = 0;
		}
		im_diff[i] = tmp;//= a un nb >4 quand dans le vert, à 0 dans le bleu
	}

	left_shore = 0;
	stop = 0;

	i = 10;

	while((i < IMAGE_BUFFER_SIZE)&&(!stop))
	{
		if((im_diff[i]==0)&& (im_diff[i-10]>0))
		{
			stop = 1;
			left_shore = 1;
			left_shore_position = i;
		}
		i++;
	}
	return left_shore ;
}

//pour le moment deux fct différentes (temporaire)
int extract_right_shore(uint8_t *buffer_blue, uint8_t *buffer_green){

	int8_t tmp = 0;
	int stop = 0;
	uint16_t i = 0;

	uint8_t im_diff[IMAGE_BUFFER_SIZE] = {0};


	for(i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){

		tmp = buffer_blue[i]- buffer_green[i];//qd petit = il y a du vert
		if (tmp <1){
			tmp = 0;
		}
		im_diff[i] = tmp;//= a un nb >4 quand dans le vert, à 0 dans le bleu
	}

	right_shore = 0;
	stop = 0;

	i = 10;

	while((i < IMAGE_BUFFER_SIZE)&&(!stop))
	{
		if((im_diff[i]==0)&& (im_diff[i-10]>0))
		{
			stop = 1;
			right_shore = 1;
			right_shore_position = i;
		}
		i++;
	}
	return right_shore ;
}


int get_right_shore(void)
{
	return right_shore;
}


void clear_shore(void)
{
	shore_to_search = 0;
}

void search_left_shore(void)
{
	shore_to_search = 1;
}

void search_right_shore(void){
	shore_to_search = 2;
}

