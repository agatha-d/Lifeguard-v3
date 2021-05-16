#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/* ======================================= */

#define MOVING_AVERAGE_WINDOW	10
#define RED_NOISE_THRESHOLD		4
#define GREEN_NOISE_THRESHOLD	15
#define LEFT_SHORE				1
#define RIGHT_SHORE				2
#define SHORE_WIDTH_SLOPE		10
#define PRESENCE_OF_RED			50


/* Creation of threads */
/* =========================================================================== */

/**
 * @brief Initialises the capture of images by the camera
 */
void capture_image_start(void);
void process_image_start(void);

/* =========================================================================== */

/**
 * @brief Forces to wait until the camera has finished capturing an image
 */
void wait_im_ready(void);


/* Functions for image interpretation */
/* =========================================================================== */

/**
 * @brief Extracts the swimmer's width from the buffer
 *
 * Returns the swimmer's width
 * Returns 0 if Swimmer not found
 */
uint16_t extract_swimmer_width(uint8_t *buffer);

/**
 * @brief Computes the position of the right shore in the buffer
 *
 * Returns 1 if the right extremity of the beach has been found
 * Returns 0 otherwise
 */
_Bool extract_right_shore(uint8_t *buffer_blue, uint8_t *buffer_green, uint8_t *buffer_red);


/**
 * @brief Computes the position of the left shore in the buffer
 *
 * Returns 1 if the left extremity of the beach has been found
 * Returns 0 otherwise
 */
_Bool extract_left_shore(uint8_t *buffer_blue, uint8_t *buffer_green, uint8_t *buffer_red);


//test fonction:

/**
 * @brief   Returns the position of the demanded shore
 *
 * @param buffer    	imb, img, imr
 * @param shore     	RIGHT_SHORE or LEFT_SHORE
 *
 * @return				The position in pixel of the demanded shore
 */
uint16_t test_extract_shore(uint8_t *buffer_blue, uint8_t *buffer_green, uint8_t *buffer_red, int shore);

/**
 * @brief   				Returns a value containing informations about the presence of a desired color in the pixel number i
 * @param buffer_diff   	Buffer containing the color we want to analyze
 * @param buffer1 & buffer2	Buffer containing the color we don't want to analyze
 * @param i     			Value of the pixel analyzed
 * @param threshold     	Value under which we considered that there is not the desired color
 * @return          		The position in pixel of the demanded shore
 */
int8_t difference(uint8_t *buffer_diff, uint8_t *buffer1, uint8_t *buffer2, int i, int threshold);

/**
* @brief   Returns the mean of the send buffer
* @param buffer      containing the value of the intensity of the color of each pixel of the buffer
*
*
* @return          the mean of the buffer
*/
uint32_t average_buffer(uint8_t *buffer);


/* Choose what shore to search */
/* =========================================================================== */

/**
* @brief   Begin the search for the left shore (thus no search for swimmers)
*/
void search_left_shore(void);

/**
* @brief   Begin the search for the right shore (thus the search for swimmers continue)
*/
void search_right_shore(void);

/**
* @brief   Stop searching shores
*/
void clear_shore(void);

/**
* @brief   Puts to true the analyzing static variable, thus starting the search of swimmers
*/
void start_analyzing (void);

/**
* @brief   Puts to false the analyzing static variable, thus stopping the search of swimmers
*/
void stop_analyzing (void);

/**
 * @brief Forces back the shore values to zero
 */
void reset_shore(void);


/* Read static variables */
/* =========================================================================== */

/**
 * @brief distance is modified by the capture image thread
 *
 * Note: the distance is deduced from the size of the ball in the buffer
 */
float get_distance_cm(void);

/**
* @brief   Returns the last swimmer width value
*
* @return          Last swimmer width value measured in pixels
*/
uint16_t get_swimmer_width(void);

/**
* @brief   Returns the last swimmer position value
*
* @return          Last swimmer position value measured in pixels
*/
uint16_t get_swimmer_position(void);

/**
* @brief   Returns the last left shore position value
*
* @return          Last left shore position value measured in pixels
*/
uint16_t get_left_shore_position(void);

/**
* @brief   Returns the last right shore position value
*
* @return          Last right shore position value measured in pixels
*/
uint16_t get_right_shore_position(void);

//prob inutile ces deux prochaines fcts
_Bool get_left_shore(void);
_Bool get_right_shore(void);


#endif /* PROCESS_IMAGE_H */
