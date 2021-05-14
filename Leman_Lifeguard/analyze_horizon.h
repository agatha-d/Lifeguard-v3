#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

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



// Voir si on les utilise ou pas

int8_t difference(uint8_t *buffer_diff, uint8_t *buffer1, uint8_t *buffer2, int i);
uint16_t rising_slope(uint8_t *buffer);
uint16_t falling_slope(uint8_t *buffer);
uint32_t average_buffer(uint8_t *buffer);


/* Choose what shore to search */
/* =========================================================================== */

void search_left_shore(void);

void search_right_shore(void);

void clear_shore(void); // Stop searching shores

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


uint16_t get_swimmer_width(void);

uint16_t get_swimmer_position(void);

uint16_t get_left_shore_position(void);

uint16_t get_right_shore_position(void);


_Bool get_left_shore(void);

_Bool get_right_shore(void);



#endif /* PROCESS_IMAGE_H */
