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

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Initialisation of peripherals
    serial_start();
    usb_start();
    dcmi_start(); //starts the camera

    po8030_start();
    po8030_set_ae(0); //test disable auto exposure 0=disable, 1 = able
    po8030_set_awb(0); // test disable auto white balance0=disable, 1 = able
    po8030_set_rgb_gain(0x20, 0x20, 0x20); // test same gain =1 for every color

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

    // Initialisation of threads
    capture_image_start();
    process_image_start();
    search_swimmer_start();
    go_to_swimmer_start();

    //switch_to_search_swimmer();

    // Main loop for finite state machine management

   while(!all_swimmers_saved)
   {
		switch(state) {

			case 0: // Search for swimmer to save

				//clear_ready_to_save();

				clear_leds();
				//set_body_led(0);
				set_front_led(0);

				switch_to_search_swimmer(); // problem : can't display image anymore

				if(get_lake_scanned()){
					//if ok car dans boucle while et que state ne change que en fonction de get empty lake => reste bien toujours dans la thread
					//set_body_led(1);
					if(get_empty_lake()){	//if not swimmer found go to victory
						state = 3;
					}

					if(!get_empty_lake()){	//if swimmer found go to swimmer
						//set_front_led(1);
						state = 1;
					}
				}
				break;

			case 1: // Go to swimmer

				//clear_leds();

				switch_to_go_to_swimmer();

				if(get_ready_to_save()){
					state = 2;
				}
				break;

			case 2: //Save swimmer
				init_before_switch();

				turn_left(HALF_TURN_COUNT, 10);
				go_straight(2000);
				turn_right(HALF_TURN_COUNT, 10);

				clear_ready_to_save();//????

				state = 3;

				clear_leds();

				state = 0;
				break;

			case 3:
				right_motor_set_speed(0);
				left_motor_set_speed(0);

				//playMelody(MARIO_FLAG, ML_SIMPLE_PLAY, NULL);

				//victory
				clear_leds();
				//set_body_led(1);

				//victory_start(); //vérifier qu'elle ne joue qu'une fois
				init_before_switch(); // empecher le code de retourner dans un mode du switch
				all_swimmers_saved = 1;
				break;
		}
    }
    clear_leds();
    set_front_led(1);
    set_body_led(0);
	//Here : bring swimmers back on beach





    /* Spoon test */
	/* ===================================== */
	//turn_around_left(HALF_TURN_COUNT);
	//go_straight(2000);
	//turn_around_right(HALF_TURN_COUNT);
	//go_straight(1000);


    /* Music test */
    /* ===================================== */
	//victory_start();

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
