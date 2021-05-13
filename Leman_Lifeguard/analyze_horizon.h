#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);

uint16_t get_swimmer_position(void);

//uint16_t get_shore_position(void);

void capture_image_start(void);

void process_image_start(void);

void wait_im_ready(void);

uint32_t smoothing(uint8_t *buffer, int k, int n);

int8_t difference(uint8_t *buffer_diff, uint8_t *buffer1, uint8_t *buffer2, int i);

uint16_t rising_slope(uint8_t *buffer);

uint16_t falling_slope(uint8_t *buffer);

uint32_t average_buffer(uint8_t *buffer);

uint16_t extract_swimmer_width(uint8_t *buffer);

uint16_t get_swimmer_width(void);

//int check_sea_or_beach(uint16_t position, uint16_t size, uint8_t *buffer_b, uint8_t *buffer_g);

//_Bool check_if_shore(uint8_t *buffer_blue, uint8_t *buffer_green);

void stop_searching_shore(void);

int extract_shore(uint8_t *buffer_blue, uint8_t *buffer_green);

//int extract_right_shore(uint8_t *buffer_blue, uint8_t *buffer_green);

int get_left_shore(void);

int get_right_shore(void);

void search_left_shore(void);

void search_right_shore(void);


#endif /* PROCESS_IMAGE_H */
