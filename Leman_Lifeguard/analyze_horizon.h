#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);

uint16_t get_swimmer_position(void);

void process_image_start(void);

uint16_t swimmer_in_danger(uint8_t *buffer);

uint32_t smoothing(uint8_t *buffer, int k, int n);

int8_t difference(uint8_t *buffer_diff, uint8_t *buffer1, uint8_t *buffer2, int i);

uint16_t rising_slope(uint8_t *buffer);

uint16_t falling_slope(uint8_t *buffer);

uint32_t average_buffer(uint8_t *buffer);

uint16_t extract_swimmer_width(uint8_t *buffer);

uint16_t get_swimmer_width(void);

//int check_sea_or_beach(uint16_t position, uint16_t size, uint8_t *buffer_b, uint8_t *buffer_g);

_Bool check_if_shore(uint8_t *buffer_blue, uint8_t *buffer_green);



#endif /* PROCESS_IMAGE_H */
