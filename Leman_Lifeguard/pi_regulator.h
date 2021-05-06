#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);

void turn_around_left(int turn_count);
void turn_around_right(int turn_count);

void go_straight(float distance);

void search_for_swimmers(uint8_t *buffer_blue, uint8_t *buffer_green);

_Bool check_if_shore(uint8_t *buffer_blue, uint8_t *buffer_green);

#endif /* PI_REGULATOR_H */
