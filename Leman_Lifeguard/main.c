#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <leds.h>

#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <audio/play_sound_file.h>

#include <main.h>

#include <navigation.h>
#include <analyze_horizon.h>
#include <navigation.h>
#include <victory.h>
#include <sensors/proximity.h>

/* ======================================= */

//Proximity captor : 10 -> 100 Hz ??? quoi ?

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Nécessaire pour programme final
void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    // Init the Inter Process Communication bus.
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Initialisation of peripherals
    serial_start();
    usb_start();

    dcmi_start(); //starts the camera
    po8030_start();
    po8030_set_ae(0); // disable auto exposure
    po8030_set_awb(0); // disable auto white balance
    po8030_set_rgb_gain(0x20, 0x20, 0x20); // same gain for every color

    dac_start();
    playMelodyStart();

    proximity_start(); //init the IR sensors
    calibrate_ir();
    motors_init();

    clear_leds();
    set_body_led(0);
    set_front_led(0);

    _Bool all_swimmers_saved = 0;

    uint8_t state = 0;

    // Make sure no random thread will start to execute
    init_before_switch();

    // Initialisation of threads for finite state machine
    capture_image_start();
    process_image_start();

    search_swimmer_start();
    go_to_swimmer_start();

    // Main loop for finite state machine management

   while(!all_swimmers_saved)
   {
		switch(state) {

			case 0: // Search for swimmer to save

				clear_leds();
				set_body_led(1);
				set_front_led(0);

				switch_to_search_swimmer();

				if(get_lake_scanned()){

					if(get_empty_lake()){	//if not swimmer found go to victory
						state = 3;
					}

					if(!get_empty_lake()){	//if swimmer found go to swimmer
						state = 1;
					}
				}
				clear_ready_to_save();
				break;

			case 1: // Go to swimmer

				set_body_led(0);

				switch_to_go_to_swimmer();

				if(get_ready_to_save()){
					state = 2;
				}
				break;

			case 2: //Save swimmer: brings swimmer back to beach

				init_before_switch(); // pause all threads

				// à faire dans une fonction

				turn_left(HALF_TURN_COUNT, 4);

			    while (get_prox(7) < 78){
					  right_motor_set_speed(MOTOR_SPEED_LIMIT);
					  left_motor_set_speed(MOTOR_SPEED_LIMIT);
			    }

			   	turn_right(HALF_TURN_COUNT, 8);
			   	go_straight(700);

			   	clear_lake();

				state = 0;

				right_motor_set_speed(0);
				left_motor_set_speed(0);

				break;

			case 3:// Victory: no more swimmers in the lake

				init_before_switch();

				set_front_led(1);
				set_body_led(0);

				right_motor_set_speed(0);
				left_motor_set_speed(0);

				playMelody(MARIO_FLAG, ML_SIMPLE_PLAY, NULL);

				clear_leds();

				all_swimmers_saved = 1;
				break;
		}
    }

    clear_leds();
    set_body_led(0);


    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
