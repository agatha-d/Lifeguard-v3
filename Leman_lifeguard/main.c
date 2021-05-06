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

#include <pi_regulator.h>
#include <process_image.h>
#include <sensors/proximity.h>

#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <audio/play_sound_file.h>

// Melodies available
// MARIO_FLAG : éventuellement pour victoire ?)
// WE_ARE_THE_CHAMPIONS : trèèèèèès aigu aïe mes oreilles
// IMPOSSIBLE_MISSION => for searching swimmers//
// UNDERWORLD : ressemble un peu à musqiue d'attente puis défaite
// PIRATES_OF_THE_CARIBBEAN : un peu aigu aussi, et un peu lent
// SEVEN_NATION_ARMY : why not victory mais je préfère mario flag


/* ======================================= */


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
	//po8030_set_ae(1); //test disable auto exposure 0=disable, 1 = able
	//po8030_set_awb(0); // test disable auto white balance0=disable, 1 = able
	//po8030_set_rgb_gain(0x20, 0x20, 0x20); // test same gain =1 for every color
    dac_start();
    playMelodyStart();
	proximity_start(); //init the IR sensors
	calibrate_ir();
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	//process_image_start();

	/*
	unsigned int tmp = 0;
	tmp = get_prox(0); // front sensor = nb

	uint32_t  dist = sqrt(tmp/2000); // en cm ? get prox >100*/




	//turn_around_left(HALF_TURN_COUNT);
	//go_straight(2000);
	//turn_around_right(HALF_TURN_COUNT);
	//go_straight(1000);

	//ici, search for swimmers - problème avec les buffers qui sont déclaré&s dans process _image, comment appeler les fonctions avec ces buffers dans le main?
	//search_for_swimmers(buffer_blue,buffer_green)
	//puis avancer vers swimmers
	//puis retourner swimmers



    /* ===================================== */
    /* Music test */


    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	//playMelody(SEVEN_NATION_ARMY, ML_SIMPLE_PLAY, NULL); //ne fonctionne pas si pas apelé dans thread ou boucle infinie
    	while (get_prox(0) < 100){
    			right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    			left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    		}

    		right_motor_set_speed(0);
    		left_motor_set_speed(0);

    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
