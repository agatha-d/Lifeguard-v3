#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/* Creation of threads */
/* =========================================================================== */

void capture_image_start(void);

void process_image_start(void);

void wait_im_ready(void);

/* Functions for image interpretation */
/* =========================================================================== */

uint16_t extract_swimmer_width(uint8_t *buffer);

int extract_right_shore(uint8_t *buffer_blue, uint8_t *buffer_green, uint8_t *buffer_red);

int extract_left_shore(uint8_t *buffer_blue, uint8_t *buffer_green, uint8_t *buffer_red);




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


void reset_shore(void);

/* Return static variables */
/* =========================================================================== */

float get_distance_cm(void);

uint16_t get_swimmer_width(void);

uint16_t get_swimmer_position(void);


int get_left_shore(void); // adapter code pour supprimer

int get_right_shore(void);// adapter code pour supprimer

uint16_t get_left_shore_position(void);

uint16_t get_right_shore_position(void);



#endif /* PROCESS_IMAGE_H */
