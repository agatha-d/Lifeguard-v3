/* File :				Leman_Lifeguard
 * Authors : 			Agatha Duranceau and Roxane Mérat
 * Last modified : 		05/16/2021 at 1pm
 *
 * Adapted from TP4_CamReg, code given in the MICRO-315 class
 */

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
#include <chprintf.h>//??//
#include <msgbus/messagebus.h>
#include <leds.h>

#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <audio/play_sound_file.h>

#include <main.h>
#include <navigation.h>
#include <analyze_horizon.h>
#include <navigation.h>
#include <sensors/proximity.h>

/* ======================================= */

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


// Optional : To use only chen calibrating camera settings
/*void SendUint8ToComputer(uint8_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}*/

static void serial_start(void) {
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void){

	 _Bool all_swimmers_saved = 0;
	 uint8_t state = 0;

    halInit();
    chSysInit();
    mpu_init();

    // Initialization of the Inter Process Communication bus.
    messagebus_init(&bus, &bus_lock, &bus_condvar);


    // Initialization of peripherals
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

    // Make sure no random thread will start to execute
    init_before_switch();

    // Initialization of threads for finite state machine
    capture_image_start();
    process_image_start();
    search_swimmer_start();
    go_to_swimmer_start();


   while(!all_swimmers_saved) { // Main loop for finite state machine management

	   switch(state) {

			case ANALYZING: // Detect swimmers in peril

				set_body_led(TRUE);
				switch_to_search_swimmer();
				if(get_lake_scanned()){
					set_body_led(FALSE);
					if(get_empty_lake()){	//if not swimmer found go to victory
						state = VICTORY;
					}
					if(!get_empty_lake()){	//if swimmer found go to swimmer
						state = BEGIN_RESCUE;
					}
				}
				clear_ready_to_save();//=>ici c après les chgmt de state<=
				break;

			case BEGIN_RESCUE: // Go to swimmer

				switch_to_go_to_swimmer();
				if(get_ready_to_save()){
					state = FINISH_RESCUE;
				}
				break;

			case FINISH_RESCUE: //Brings swimmer back to beach

				init_before_switch(); // pause all threads
				stop_analyzing();
				bring_swimmer_to_beach();
			   	clear_lake();
			   	halt_robot();
			   	start_analyzing();
				state = ANALYZING;
				break;

			case VICTORY://No more swimmers in the lake

				init_before_switch();
				set_front_led(TRUE);
				halt_robot();
				playMelody(MARIO_FLAG, ML_SIMPLE_PLAY, NULL);
				set_front_led(FALSE);
				all_swimmers_saved = 1;
				break;
		}
    }

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){
    chSysHalt("Stack smashing detected");
}
