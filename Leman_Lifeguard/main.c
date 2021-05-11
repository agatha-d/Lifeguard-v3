#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <main.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <leds.h>


#include <navigation.h>
#include <analyze_horizon.h>
#include <navigation.h>
#include <victory.h>
#include <sensors/proximity.h>

#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <audio/play_sound_file.h>

/* ======================================= */

//capteu de prox : 10 -> 100 Hz

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


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
    po8030_set_rgb_gain(0x20, 0x20, 0x30); // test same gain =1 for every color
    dac_start();
    playMelodyStart();
    proximity_start(); //init the IR sensors
    calibrate_ir();
    motors_init();

    go_to_swimmer_start();

	//Here : search for swimmers

	//stars the threads for the pi regulator and the processing of the image

    uint8_t state = 0;
    _Bool all_swimmers_saved = 0;

    clear_leds();
    set_body_led(0);
    set_front_led(0);


    process_image_start();

    while(!all_swimmers_saved)
    {
		switch(state) {
			case 0: // Search for swimmer to save
				clear_leds();
				//set_body_led(1);
				//set_led(LED6, 10);
				search_swimmer_start();

				while(!get_analyse())
				{
					//set_front_led(1);
					//attendre
				}
				//set_front_led(0);

				if(get_analyse()){
					if(get_empty_lake()){//if not swimmer found go to victory
						state = 3;
					}

					if(!get_empty_lake()){//if swimmer found go to him (state 1)
						state = 1;
					}
				}
				break;

			case 1:
				//pour le moment : arrive au bon moment ici, n'envoit plus d'image à l'ordinateur. C'est un pb car il fau
				clear_leds();
				set_body_led(0);
				//set_body_led(1);
				switch_to_one();
				while(1)
				{
					set_body_led(1);
				}
				//set_led(LED3, 1);
				//go_to_swimmer_start(); //->causes panic
				//ajouter IR ici ou dans la thread ????
				break;
			case 2:
				clear_leds();
				//set_rgb_led(LED4, 1, 0, 0);
				//go_back_to_beach ou save_swimmer
				// retour case 0
				state = 0;
				break;
			case 3: //victory
				clear_leds();

				set_rgb_led(LED4, 1, 0, 0);
				set_rgb_led(LED6, 1, 0, 0);

				set_body_led(1);

				//set_led(LED5, 1);
				//start la bonne thread
				//play victory music
				all_swimmers_saved = 1;
				break;
		}
    }
    clear_leds();
    set_front_led(0);
    set_body_led(0);
	//Here : bring swimmers back on beach


	/* ===================================== */
	/* Spoon test */
	//turn_around_left(HALF_TURN_COUNT);
	//go_straight(2000);
	//turn_around_right(HALF_TURN_COUNT);
	//go_straight(1000);


    /* ===================================== */
    /* Music test */
	//victory_start();

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	//IR sensor works only in thread or infinite loop, put it in a thread in navigation or analyze horizon
    	/*while (get_prox(0) < 100){
    			right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    			left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    		}

    		right_motor_set_speed(0);
    		left_motor_set_speed(0);*/

    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
