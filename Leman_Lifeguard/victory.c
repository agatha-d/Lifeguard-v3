#include "ch.h"
#include "hal.h"
#include <usbcfg.h>

#include <motors.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <audio/play_sound_file.h>
#include "leds.h"

// Melodies available
// MARIO_FLAG : éventuellement pour victoire ?)
// WE_ARE_THE_CHAMPIONS : trèèèèèès aigu aïe mes oreilles
// IMPOSSIBLE_MISSION => for searching swimmers//
// UNDERWORLD : ressemble un peu à musqiue d'attente puis défaite
// PIRATES_OF_THE_CARIBBEAN : un peu aigu aussi, et un peu lent
// SEVEN_NATION_ARMY : why not victory mais je préfère mario flag



// Thread inspired from main.c of the teacher's src file for animation

static THD_WORKING_AREA(waVictoryMusic, 256);
static THD_FUNCTION(VictoryMusic, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    uint8_t rgb_state = 0, rgb_counter = 0;
    uint16_t melody_state = 0, melody_counter = 0;

    /*Mario flag pole tempo*/
    static const float victory_tempo[] = {
      15, 15, 15, 15, 15, 15,  4.5, 4.5, 15, 15,
      15, 15, 15, 15, 4.5, 4.5, 15, 15, 15,
      15, 15, 15, 4.5, 15, 15, 15, 3};


    while(1){

    	time = chVTGetSystemTime();

    	 //int i;
    	 //int led;
    	 switch(rgb_state) {
    	 case 0: // Red.
    		 /*for (i = 0 ; i < 27 ; i++){
    			 uint16_t pause = (uint16_t)(1000 / victory_tempo[i]); //ne fonctionne pas
    			 led = i%8;
    			 set_rgb_led(led, 10, 0, 0);
    			 chThdSleepMilliseconds(pause);
    		 }*/
    		 set_rgb_led(0, 10, 0, 0);
    		 set_rgb_led(1, 10, 0, 0);
    		 set_rgb_led(2, 10, 0, 0);
    		 set_rgb_led(3, 10, 0, 0);
    		 break;
    	 case 1: // Green.
    		 set_rgb_led(0, 0, 10, 0);
    		 set_rgb_led(1, 0, 10, 0);
    		 set_rgb_led(2, 0, 10, 0);
    		 set_rgb_led(3, 0, 10, 0);
    		 break;
    	 case 2: // Blue.
    		 set_rgb_led(0, 0, 0, 10);
    		 set_rgb_led(1, 0, 0, 10);
    		 set_rgb_led(2, 0, 0, 10);
    		 set_rgb_led(3, 0, 0, 10);
    		 break;
    	 }
    	 rgb_counter++;
    	 if(rgb_counter == 30) {
    		 rgb_counter = 0;
    		 rgb_state = (rgb_state+1)%3;
    		 set_body_led(2);
    		 set_front_led(2);
    	 }
    	 melody_counter++;
    	 if(melody_counter == 20) {
    		 melody_counter = 0;
    		 //melody_state = (melody_state+1)%NB_SONGS;
    		 playMelody(MARIO_FLAG, ML_SIMPLE_PLAY, NULL);
    	 }

    	 chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.

    	 //chTdhExit();
    	//chThdSleepMilliseconds(1000);
    }
}

void victory_start(void){
	chThdCreateStatic(waVictoryMusic, sizeof(waVictoryMusic), NORMALPRIO, VictoryMusic, NULL);
}
/*
static THD_WORKING_AREA(waVictoryDisco, 256);
static THD_FUNCTION(VictoryDisco, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){
    	playMelody(MARIO_FLAG, ML_SIMPLE_PLAY, NULL); //ne fonctionne pas si pas apelé dans thread ou boucle infinie
    	//chTdhExit();
        //100Hz
        //chThdSleepUntilWindowed(time, time + MS2ST(10));
    	chThdSleepMilliseconds(3000);
    }
}*/
